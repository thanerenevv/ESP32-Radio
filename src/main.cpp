#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <driver/i2s.h>
#include <esp_adc_cal.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <cstring>
#include <cmath>
#include "config.h"

static constexpr uint8_t PKT_AUDIO    = 0x01;
static constexpr uint8_t PKT_END      = 0x02;
static constexpr uint8_t PKT_KEYFRAME = 0x03;

static constexpr uint8_t  KEYFRAME_INTERVAL   = 16;
static constexpr uint8_t  MIN_PLAYBACK_FRAMES = 4;
static constexpr uint8_t  MAX_PLAYBACK_FRAMES = 12;
static constexpr uint32_t JB_GROW_THRESH_MS   = 80;
static constexpr uint32_t JB_SHRINK_THRESH_MS = 400;
static constexpr uint8_t  SIDETONE_SHIFT      = 3;

static constexpr uint32_t BAT_LOW_MV       = 3400;
static constexpr uint32_t BAT_CRITICAL_MV  = 3200;
static constexpr uint32_t DISPLAY_DIM_MS   = 15000;
static constexpr uint32_t DISPLAY_OFF_MS   = 30000;
static constexpr uint8_t  LONGPRESS_MS     = 600;
static constexpr uint8_t  REPEAT_MS        = 120;

static constexpr uint8_t APP_CRC_POLY = 0x97;

static const int16_t ADPCM_STEP_TABLE[89] = {
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,
    66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,
    371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,
    1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
    5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,
    18500,20350,22385,24623,27086,29794,32767
};

static const int8_t ADPCM_INDEX_TABLE[16] = {
    -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8
};

struct AdpcmState {
    int16_t pred;
    int8_t  idx;
};

static uint8_t adpcm_encode_sample(int16_t sample, AdpcmState &s) {
    int16_t step  = ADPCM_STEP_TABLE[s.idx];
    int32_t diff  = (int32_t)sample - s.pred;
    uint8_t code  = 0;
    if (diff < 0) { code = 8; diff = -diff; }
    if (diff >= step)        { code |= 4; diff -= step; }
    if (diff >= (step >> 1)) { code |= 2; diff -= (step >> 1); }
    if (diff >= (step >> 2)) { code |= 1; }
    int32_t diffq = step >> 3;
    if (code & 4) diffq += step;
    if (code & 2) diffq += step >> 1;
    if (code & 1) diffq += step >> 2;
    int32_t np = s.pred + ((code & 8) ? -diffq : diffq);
    s.pred = (int16_t)((np > 32767) ? 32767 : (np < -32768) ? -32768 : np);
    int8_t ni = s.idx + ADPCM_INDEX_TABLE[code & 0x7];
    s.idx = (ni < 0) ? 0 : (ni > 88) ? 88 : ni;
    return code & 0xF;
}

static int16_t adpcm_decode_sample(uint8_t code, AdpcmState &s) {
    int16_t step  = ADPCM_STEP_TABLE[s.idx];
    int32_t diffq = step >> 3;
    if (code & 4) diffq += step;
    if (code & 2) diffq += step >> 1;
    if (code & 1) diffq += step >> 2;
    int32_t np = s.pred + ((code & 8) ? -diffq : diffq);
    s.pred = (int16_t)((np > 32767) ? 32767 : (np < -32768) ? -32768 : np);
    int8_t ni = s.idx + ADPCM_INDEX_TABLE[code & 0x7];
    s.idx = (ni < 0) ? 0 : (ni > 88) ? 88 : ni;
    return s.pred;
}

static void adpcm_encode_block(const int16_t *in, uint8_t *out, uint16_t samples, AdpcmState &s) {
    uint16_t pairs = samples >> 1;
    for (uint16_t i = 0; i < pairs; i++) {
        uint8_t lo = adpcm_encode_sample(in[i*2],   s);
        uint8_t hi = adpcm_encode_sample(in[i*2+1], s);
        out[i] = lo | (hi << 4);
    }
    if (samples & 1) out[pairs] = adpcm_encode_sample(in[samples-1], s);
}

static void adpcm_decode_block(const uint8_t *in, int16_t *out, uint16_t nibbles, AdpcmState &s) {
    uint16_t bytes = nibbles >> 1;
    for (uint16_t i = 0; i < bytes; i++) {
        out[i*2]   = adpcm_decode_sample(in[i] & 0xF, s);
        out[i*2+1] = adpcm_decode_sample(in[i] >> 4,  s);
    }
    if (nibbles & 1) out[nibbles-1] = adpcm_decode_sample(in[bytes] & 0xF, s);
}

static void adpcm_conceal_block(int16_t *out, uint16_t count, AdpcmState &s, uint8_t decayShift) {
    for (uint16_t i = 0; i < count; i++) {
        s.pred = (int16_t)(((int32_t)s.pred * (32 - decayShift)) >> 5);
        out[i] = s.pred;
    }
}

static uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? ((crc << 1) ^ APP_CRC_POLY) : (crc << 1);
    }
    return crc;
}

struct __attribute__((packed)) Packet {
    uint8_t  type;
    uint8_t  seq;
    int16_t  adpcmPred;
    int8_t   adpcmIdx;
    uint8_t  dataLen;
    uint8_t  appCrc;
    uint8_t  data[NRF_PAYLOAD_SIZE - 7];
};
static_assert(sizeof(Packet) == NRF_PAYLOAD_SIZE, "Packet size mismatch");
static constexpr uint8_t MAX_DATA_LEN = NRF_PAYLOAD_SIZE - 7;

struct TxPacket {
    uint8_t raw[NRF_PAYLOAD_SIZE];
};

struct RxFrame {
    int16_t  pcm[ADPCM_SAMPLES_PER_PKT * 2];
    uint16_t count;
    bool     gap;
    bool     concealed;
};

static QueueHandle_t     txQueue;
static QueueHandle_t     rxQueue;
static QueueHandle_t     sidetoneQueue;
static SemaphoreHandle_t radioMutex;

static RF24              radio(NRF_CE_PIN, NRF_CSN_PIN);
static Adafruit_SH1106G  display(128, 64, &Wire, -1);

static esp_adc_cal_characteristics_t adcChars;

static std::atomic<bool>     gTransmitting{false};
static std::atomic<uint8_t>  gChannelIdx{DEFAULT_CHANNEL_IDX};
static std::atomic<uint8_t>  gVolume{VOLUME_DEFAULT};
static std::atomic<uint16_t> gSquelch{SQUELCH_DEFAULT};
static std::atomic<bool>     gDisplayDirty{true};

static std::atomic<uint32_t> gPktsReceived{0};
static std::atomic<uint32_t> gPktsLost{0};
static std::atomic<uint32_t> gPktsInWindow{0};
static std::atomic<uint32_t> gLostInWindow{0};

static float    gBatVoltage        = 0.0f;
static uint32_t gBatMv             = 0;
static uint8_t  gBatPct            = 0;
static bool     gBatWarning        = false;
static bool     gBatCritical       = false;
static uint8_t  gSignalBars        = 0;
static int8_t   gRssiLast          = -127;
static uint32_t gSignalWindowStart = 0;

static uint8_t gTxSeq        = 0;
static uint8_t gRxLastSeq    = 0xFF;
static uint8_t gTxPktCounter = 0;

static AdpcmState gLastDecodeState = {0, 0};
static bool       gRxFirstPacket   = true;

static int32_t gHpZ1 = 0;
static int32_t gHpZ2 = 0;
static int32_t gLpZ  = 0;

static int32_t  gDeempZ       = 0;
static int32_t  gPreempZ      = 0;

static int32_t  gAgcGain         = 256;
static int32_t  gAgcEnvelope     = 0;
static uint32_t gNgHoldCounter   = 0;
static bool     gNgOpen          = false;
static int32_t  gNgGain          = 0;
static int32_t  gNgGainTarget    = 0;

static std::atomic<bool>    gPlaybackReady{false};
static std::atomic<uint8_t> gJbTarget{MIN_PLAYBACK_FRAMES};
static uint32_t             gJbLastAdjust = 0;
static uint32_t             gLastUnderflow = 0;

static uint32_t gLastActivity   = 0;
static bool     gDisplayDimmed  = false;
static bool     gDisplayOff     = false;

static const uint8_t LOG_VOL_TABLE[VOLUME_MAX - VOLUME_MIN + 1] = {
    0,1,2,3,4,5,6,7,9,11,13,16,19,22,26,31,37,43,51,60,70,82,96,112,131,152,177,206,239,255
};

enum MenuState : uint8_t { MENU_NONE, MENU_MAIN, MENU_VOLUME, MENU_SQUELCH, MENU_CHANNEL };
static MenuState gMenu    = MENU_NONE;
static uint8_t   gMenuSel = 0;

static void i2s_speaker_init() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 6;
    cfg.dma_buf_len          = 256;
    cfg.use_apll             = true;
    cfg.tx_desc_auto_clear   = true;

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = I2S_BCLK_PIN;
    pins.ws_io_num    = I2S_WCLK_PIN;
    pins.data_out_num = I2S_DATA_PIN;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pins));
}

static void i2s_mic_init() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 6;
    cfg.dma_buf_len          = 256;
    cfg.use_apll             = true;

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = I2S_MIC_SCK_PIN;
    pins.ws_io_num    = I2S_MIC_WS_PIN;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num  = I2S_MIC_SD_PIN;

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &cfg, 0, nullptr));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pins));
}

static float read_battery_voltage() {
    uint32_t raw = 0;
    for (int i = 0; i < 16; i++) raw += analogRead(BAT_ADC_PIN);
    raw >>= 4;
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adcChars);
    return (mv / 1000.0f) * ((BAT_R1 + BAT_R2) / BAT_R2);
}

static uint8_t lipo_pct(float v) {
    if (v >= LIPO_CURVE[0].v) return 100;
    if (v <= LIPO_CURVE[LIPO_CURVE_LEN - 1].v) return 0;
    for (int i = 0; i < LIPO_CURVE_LEN - 1; i++) {
        if (v >= LIPO_CURVE[i + 1].v) {
            float span = LIPO_CURVE[i].v - LIPO_CURVE[i + 1].v;
            float frac = (v - LIPO_CURVE[i + 1].v) / span;
            return (uint8_t)(LIPO_CURVE[i + 1].pct + frac * (LIPO_CURVE[i].pct - LIPO_CURVE[i + 1].pct));
        }
    }
    return 0;
}

static uint32_t compute_energy(const uint8_t *adpcmData, uint8_t len) {
    uint32_t energy = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t lo = adpcmData[i] & 0xF;
        uint8_t hi = adpcmData[i] >> 4;
        energy += (uint32_t)(lo & 7) * (lo & 7) + (uint32_t)(hi & 7) * (hi & 7);
    }
    return energy;
}

static void remove_dc_offset(int16_t *buf, uint16_t len) {
    if (len == 0) return;
    int32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += buf[i];
    int16_t dc = (int16_t)(sum / len);
    for (uint16_t i = 0; i < len; i++) buf[i] -= dc;
}

static void apply_highpass(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t x = (int32_t)buf[i];
        int32_t y = x - gHpZ1 + (gHpZ2 * 31 >> 5);
        gHpZ1 = x;
        gHpZ2 = y;
        buf[i] = (int16_t)((y > 32767) ? 32767 : (y < -32768) ? -32768 : y);
    }
}

static void apply_lowpass(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t x = (int32_t)buf[i];
        gLpZ += (x - gLpZ) >> 2;
        buf[i] = (int16_t)((gLpZ > 32767) ? 32767 : (gLpZ < -32768) ? -32768 : gLpZ);
    }
}

static void apply_preemphasis(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t x   = (int32_t)buf[i];
        int32_t out = x - ((gPreempZ * 13) >> 4);
        gPreempZ    = x;
        buf[i] = (int16_t)((out > 32767) ? 32767 : (out < -32768) ? -32768 : out);
    }
}

static void apply_deemphasis(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t x = (int32_t)buf[i];
        gDeempZ  += (x - gDeempZ) * 13 >> 4;
        buf[i] = (int16_t)((gDeempZ > 32767) ? 32767 : (gDeempZ < -32768) ? -32768 : gDeempZ);
    }
}

static void apply_speech_eq(int16_t *buf, uint16_t len) {
    static int32_t z1 = 0, z2 = 0;
    for (uint16_t i = 0; i < len; i++) {
        int32_t x  = (int32_t)buf[i];
        int32_t hp = x - z1;
        z1 += (x - z1) >> 3;
        int32_t lp2 = z2 + ((x - z2) >> 1);
        z2 = lp2;
        int32_t bandpass = hp - (x - lp2);
        int32_t out = x + (bandpass >> 2);
        buf[i] = (int16_t)((out > 32767) ? 32767 : (out < -32768) ? -32768 : out);
    }
}

static void apply_limiter(int16_t *buf, uint16_t len) {
    static int32_t envLim = 0;
    for (uint16_t i = 0; i < len; i++) {
        int32_t s   = buf[i];
        int32_t abs = s < 0 ? -s : s;
        if (abs > envLim) envLim = abs;
        else envLim -= (envLim - abs) >> 6;
        if (envLim > 28000) {
            s = (s * 28000) / envLim;
        }
        buf[i] = (int16_t)((s > 29000) ? 29000 : (s < -29000) ? -29000 : s);
    }
}

static void apply_fade_in(int16_t *buf, uint16_t len, uint8_t fadeLen) {
    uint8_t n = (len < fadeLen) ? (uint8_t)len : fadeLen;
    for (uint8_t i = 0; i < n; i++)
        buf[i] = (int16_t)(((int32_t)buf[i] * i) / n);
}

static void apply_fade_out(int16_t *buf, uint16_t len, uint8_t fadeLen) {
    if (len < fadeLen) fadeLen = (uint8_t)len;
    uint16_t start = len - fadeLen;
    for (uint8_t i = 0; i < fadeLen; i++)
        buf[start + i] = (int16_t)(((int32_t)buf[start + i] * (fadeLen - i)) / fadeLen);
}

static void apply_agc(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t abs = buf[i] < 0 ? -buf[i] : buf[i];
        if (abs > gAgcEnvelope) gAgcEnvelope += (abs - gAgcEnvelope) >> 2;
        else                    gAgcEnvelope += (abs - gAgcEnvelope) >> 8;
    }
    int32_t targetGain = (gAgcEnvelope > 0) ? (24000L * 256) / gAgcEnvelope : 256;
    if (targetGain > 1024) targetGain = 1024;
    if (targetGain < 64)   targetGain = 64;
    if (targetGain > gAgcGain) gAgcGain += (targetGain - gAgcGain) >> 9;
    else                       gAgcGain += (targetGain - gAgcGain) >> 4;

    for (uint16_t i = 0; i < len; i++) {
        int32_t s = ((int32_t)buf[i] * gAgcGain) >> 8;
        buf[i] = (int16_t)((s > 29000) ? 29000 : (s < -29000) ? -29000 : s);
    }
}

static void apply_noise_gate(int16_t *buf, uint16_t len, uint16_t openThresh, uint16_t closeThresh) {
    int32_t rms = 0;
    for (uint16_t i = 0; i < len; i++) {
        int32_t s = buf[i];
        rms += (s * s) >> 8;
    }
    rms /= len;

    const int32_t attackStep  = 256 / 8;
    const int32_t releaseStep = 256 / 64;

    if (!gNgOpen && rms >= (int32_t)openThresh) {
        gNgOpen        = true;
        gNgHoldCounter = SAMPLE_RATE / 20;
        gNgGainTarget  = 256;
    } else if (gNgOpen) {
        gNgGainTarget = 256;
        if (rms >= (int32_t)closeThresh) {
            gNgHoldCounter = SAMPLE_RATE / 20;
        } else if (gNgHoldCounter > 0) {
            gNgHoldCounter -= (len < gNgHoldCounter) ? len : gNgHoldCounter;
        } else {
            gNgOpen       = false;
            gNgGainTarget = 16;
        }
    } else {
        gNgGainTarget = 16;
    }

    for (uint16_t i = 0; i < len; i++) {
        if (gNgGain < gNgGainTarget) gNgGain = (gNgGain + attackStep  < gNgGainTarget) ? gNgGain + attackStep  : gNgGainTarget;
        else                         gNgGain = (gNgGain - releaseStep > gNgGainTarget) ? gNgGain - releaseStep : gNgGainTarget;
        buf[i] = (int16_t)(((int32_t)buf[i] * gNgGain) >> 8);
    }
}

static void update_signal_bars() {
    uint32_t now     = millis();
    uint32_t elapsed = now - gSignalWindowStart;
    if (elapsed >= SIGNAL_WINDOW_MS) {
        uint32_t rx    = gPktsInWindow.exchange(0);
        uint32_t lost  = gLostInWindow.exchange(0);
        uint32_t total = rx + lost;

        uint8_t qualityPct = (total > 0) ? (uint8_t)((rx * 100u) / total) : 0;

        int8_t rssi = gRssiLast;
        uint8_t rssiBars = 0;
        if      (rssi > -60)  rssiBars = 4;
        else if (rssi > -75)  rssiBars = 3;
        else if (rssi > -85)  rssiBars = 2;
        else if (rssi > -95)  rssiBars = 1;

        uint8_t deliveryBars = 0;
        if      (qualityPct > 90) deliveryBars = 4;
        else if (qualityPct > 70) deliveryBars = 3;
        else if (qualityPct > 40) deliveryBars = 2;
        else if (qualityPct > 10) deliveryBars = 1;

        gSignalBars = (rssiBars < deliveryBars) ? rssiBars : deliveryBars;
        gSignalWindowStart = now;
    }
}

static void update_jitter_buffer() {
    uint32_t now      = millis();
    uint32_t qDepth   = (uint32_t)uxQueueMessagesWaiting(rxQueue);
    uint8_t  target   = (uint8_t)gJbTarget;

    if (now - gLastUnderflow < JB_GROW_THRESH_MS) {
        if (target < MAX_PLAYBACK_FRAMES) {
            target++;
            gJbTarget.store(target);
            gJbLastAdjust = now;
        }
    } else if ((now - gJbLastAdjust) > JB_SHRINK_THRESH_MS && qDepth > target + 2 && target > MIN_PLAYBACK_FRAMES) {
        target--;
        gJbTarget.store(target);
        gJbLastAdjust = now;
    }
}

static void draw_battery_icon(uint8_t x, uint8_t y, uint8_t pct) {
    display.drawRect(x, y, 18, 8, SH110X_WHITE);
    display.fillRect(x + 18, y + 2, 2, 4, SH110X_WHITE);
    uint8_t fill = (uint8_t)(pct * 16u / 100u);
    if (fill > 0) display.fillRect(x + 1, y + 1, fill, 6, SH110X_WHITE);
    if (gBatCritical && ((millis() / 500) & 1)) {
        display.fillRect(x, y, 20, 8, SH110X_BLACK);
        display.drawRect(x, y, 18, 8, SH110X_WHITE);
        display.fillRect(x + 18, y + 2, 2, 4, SH110X_WHITE);
    }
}

static void draw_signal_bars(uint8_t x, uint8_t y, uint8_t bars) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t h  = (i + 1) * 3;
        uint8_t bx = x + i * 5;
        uint8_t by = y + (12 - h);
        if (i < bars) display.fillRect(bx, by, 4, h, SH110X_WHITE);
        else          display.drawRect(bx, by, 4, h, SH110X_WHITE);
    }
}

static void draw_main_screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("CH:");
    display.print((uint8_t)gChannelIdx + 1);
    display.print(" ");
    display.print(CHANNEL_MHZ[(uint8_t)gChannelIdx]);
    display.print("M");
    draw_signal_bars(90, 0, gSignalBars);
    display.setCursor(0, 12);
    display.print("BAT:");
    display.print(gBatVoltage, 2);
    display.print("V ");
    display.print(gBatPct);
    display.print("%");
    draw_battery_icon(95, 12, gBatPct);
    if (gBatWarning) {
        display.setCursor(0, 24);
        display.print(gBatCritical ? "!! LOW BAT !!" : "! Low battery");
    } else {
        display.setCursor(0, 24);
        display.print("SQ:");
        display.print((uint16_t)gSquelch);
        display.print("  VOL:");
        display.print((uint8_t)gVolume);
    }
    display.drawFastHLine(0, 35, 128, SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 40);
    display.print(gTransmitting.load() ? "TX >>>" : "RX ...");
    if (!gTransmitting.load()) {
        display.setTextSize(1);
        display.setCursor(90, 55);
        display.print("JB:");
        display.print((uint8_t)gJbTarget);
    }
    display.display();
}

static const char * const MENU_ITEMS[] = {"Volume", "Squelch", "Channel", "Back"};
static constexpr uint8_t MENU_ITEM_COUNT = 4;

static void draw_menu_screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(28, 0);
    display.print("[ SETTINGS ]");
    display.drawFastHLine(0, 9, 128, SH110X_WHITE);
    for (uint8_t i = 0; i < MENU_ITEM_COUNT; i++) {
        display.setCursor(8, 12 + i * 13);
        if (i == gMenuSel) {
            display.fillRect(0, 11 + i * 13, 128, 12, SH110X_WHITE);
            display.setTextColor(SH110X_BLACK);
        } else {
            display.setTextColor(SH110X_WHITE);
        }
        display.print(MENU_ITEMS[i]);
    }
    display.display();
}

static void draw_value_screen(const char *label, int32_t val, int32_t minv, int32_t maxv) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print(label);
    display.setTextSize(3);
    display.setCursor(20, 20);
    display.print(val);
    int32_t range = maxv - minv;
    uint8_t barW  = (range > 0) ? (uint8_t)(((val - minv) * 120) / range) : 0;
    display.drawRect(4, 54, 120, 8, SH110X_WHITE);
    if (barW > 0) display.fillRect(4, 54, barW, 8, SH110X_WHITE);
    display.display();
}

static void draw_channel_screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Channel Select");
    display.drawFastHLine(0, 9, 128, SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.print("CH ");
    display.print((uint8_t)gChannelIdx + 1);
    display.setTextSize(1);
    display.setCursor(10, 44);
    display.print(CHANNEL_MHZ[(uint8_t)gChannelIdx]);
    display.print(" MHz");
    display.display();
}

static void update_display() {
    switch (gMenu) {
        case MENU_NONE:    draw_main_screen();  break;
        case MENU_MAIN:    draw_menu_screen();  break;
        case MENU_VOLUME:  draw_value_screen("Volume",  gVolume,  VOLUME_MIN,  VOLUME_MAX);  break;
        case MENU_SQUELCH: draw_value_screen("Squelch", gSquelch, SQUELCH_MIN, SQUELCH_MAX); break;
        case MENU_CHANNEL: draw_channel_screen(); break;
    }
}

static void radio_apply_channel(uint8_t idx) {
    xQueueReset(rxQueue);
    gPlaybackReady.store(false);
    gRxFirstPacket = true;
    gLastDecodeState = {0, 0};
    gHpZ1 = 0; gHpZ2 = 0; gLpZ = 0;
    gDeempZ = 0;
    radio.stopListening();
    radio.setChannel(CHANNEL_LIST[idx]);
    if (!gTransmitting.load()) radio.startListening();
}

static void radio_init() {
    SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
    if (!radio.begin()) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(0, 0);
        display.print("Radio FAIL");
        display.display();
        while (true) vTaskDelay(portMAX_DELAY);
    }
    radio.setPALevel(NRF_PA_LEVEL);
    radio.setDataRate(NRF_DATA_RATE);
    radio.setPayloadSize(NRF_PAYLOAD_SIZE);
    radio.setChannel(CHANNEL_LIST[(uint8_t)gChannelIdx]);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(false);
    radio.openWritingPipe(NRF_PIPE_ADDR);
    radio.openReadingPipe(1, NRF_PIPE_ADDR);
    radio.startListening();
}

struct BtnState {
    bool     lastRaw;
    uint32_t lastMs;
    uint32_t pressMs;
    bool     held;
    uint32_t lastRepeatMs;
};
static BtnState btnStates[5];
static const uint8_t BTN_PINS[5] = {
    BTN_UP_PIN, BTN_DOWN_PIN, BTN_BACK_PIN, BTN_SELECT_PIN, BTN_PTT_PIN
};

static bool btn_fell(uint8_t idx) {
    bool     raw = (digitalRead(BTN_PINS[idx]) == LOW);
    uint32_t now = millis();
    BtnState &b  = btnStates[idx];
    if (raw != b.lastRaw && (now - b.lastMs) > DEBOUNCE_MS) {
        b.lastRaw = raw;
        b.lastMs  = now;
        if (raw) b.pressMs = now;
        return raw;
    }
    return false;
}

static bool btn_repeat(uint8_t idx) {
    BtnState &b  = btnStates[idx];
    uint32_t now = millis();
    if (!b.lastRaw) return false;
    if (!b.held && (now - b.pressMs) >= LONGPRESS_MS) {
        b.held         = true;
        b.lastRepeatMs = now;
        return true;
    }
    if (b.held && (now - b.lastRepeatMs) >= REPEAT_MS) {
        b.lastRepeatMs = now;
        return true;
    }
    return false;
}

static void btn_clear_hold(uint8_t idx) {
    btnStates[idx].held = false;
}

static void handle_menu_buttons() {
    bool upFell   = btn_fell(0);
    bool downFell = btn_fell(1);
    bool backFell = btn_fell(2);
    bool selFell  = btn_fell(3);

    bool upRep   = btn_repeat(0);
    bool downRep = btn_repeat(1);

    bool upAct   = upFell   || upRep;
    bool downAct = downFell || downRep;

    gLastActivity = millis();
    if (gDisplayOff) {
        gDisplayOff    = false;
        gDisplayDimmed = false;
        display.oled_command(0xAF);
        gDisplayDirty = true;
        return;
    }
    if (gDisplayDimmed) {
        gDisplayDimmed = false;
        display.oled_command(0xAF);
    }

    if (upAct) {
        switch (gMenu) {
            case MENU_MAIN:
                if (gMenuSel > 0) { gMenuSel--; gDisplayDirty = true; }
                break;
            case MENU_VOLUME:
                if ((uint8_t)gVolume < VOLUME_MAX) { gVolume++; gDisplayDirty = true; }
                break;
            case MENU_SQUELCH:
                if ((uint16_t)gSquelch < SQUELCH_MAX) { gSquelch = (uint16_t)gSquelch + SQUELCH_STEP; gDisplayDirty = true; }
                break;
            case MENU_CHANNEL:
                if ((uint8_t)gChannelIdx < CHANNEL_COUNT - 1) {
                    gChannelIdx++;
                    xSemaphoreTake(radioMutex, portMAX_DELAY);
                    radio_apply_channel(gChannelIdx);
                    xSemaphoreGive(radioMutex);
                    gDisplayDirty = true;
                }
                break;
            default: break;
        }
        btn_clear_hold(0);
    }

    if (downAct) {
        switch (gMenu) {
            case MENU_MAIN:
                if (gMenuSel < MENU_ITEM_COUNT - 1) { gMenuSel++; gDisplayDirty = true; }
                break;
            case MENU_VOLUME:
                if ((uint8_t)gVolume > VOLUME_MIN) { gVolume--; gDisplayDirty = true; }
                break;
            case MENU_SQUELCH:
                if ((uint16_t)gSquelch >= (uint16_t)(SQUELCH_MIN + SQUELCH_STEP)) { gSquelch = (uint16_t)gSquelch - SQUELCH_STEP; gDisplayDirty = true; }
                break;
            case MENU_CHANNEL:
                if ((uint8_t)gChannelIdx > 0) {
                    gChannelIdx--;
                    xSemaphoreTake(radioMutex, portMAX_DELAY);
                    radio_apply_channel(gChannelIdx);
                    xSemaphoreGive(radioMutex);
                    gDisplayDirty = true;
                }
                break;
            default: break;
        }
        btn_clear_hold(1);
    }

    if (backFell && gMenu != MENU_NONE) {
        gMenu    = (gMenu == MENU_MAIN) ? MENU_NONE : MENU_MAIN;
        gMenuSel = 0;
        gDisplayDirty = true;
    }

    if (selFell) {
        if (gMenu == MENU_NONE) {
            gMenu    = MENU_MAIN;
            gMenuSel = 0;
        } else if (gMenu == MENU_MAIN) {
            switch (gMenuSel) {
                case 0: gMenu = MENU_VOLUME;  break;
                case 1: gMenu = MENU_SQUELCH; break;
                case 2: gMenu = MENU_CHANNEL; break;
                case 3: gMenu = MENU_NONE;    break;
            }
        } else {
            gMenu = MENU_MAIN;
        }
        gDisplayDirty = true;
    }
}

static constexpr uint32_t TX_INTERVAL_US =
    (uint32_t)(1000000ULL * ADPCM_SAMPLES_PER_PKT / SAMPLE_RATE);

static bool gTxFirstFrame = true;

static void task_capture(void *) {
    static int16_t    pcmBuf[MIC_CAPTURE_SAMPLES];
    static uint8_t    adpcmOut[MIC_CAPTURE_SAMPLES / 2 + 1];
    static AdpcmState encState = {0, 0};

    while (true) {
        if (!gTransmitting.load()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        size_t got = 0;
        esp_err_t err = i2s_read(I2S_NUM_1, pcmBuf, sizeof(pcmBuf), &got, pdMS_TO_TICKS(20));
        if (err != ESP_OK || got == 0) continue;

        uint16_t samples = (uint16_t)(got / sizeof(int16_t));
        remove_dc_offset(pcmBuf, samples);
        apply_highpass(pcmBuf, samples);
        apply_preemphasis(pcmBuf, samples);
        apply_speech_eq(pcmBuf, samples);
        apply_lowpass(pcmBuf, samples);
        apply_agc(pcmBuf, samples);
        apply_noise_gate(pcmBuf, samples, 80, 40);
        apply_limiter(pcmBuf, samples);

        if (gTxFirstFrame) {
            apply_fade_in(pcmBuf, samples, 32);
            gTxFirstFrame = false;
        }

        int16_t sidetone[MIC_CAPTURE_SAMPLES];
        memcpy(sidetone, pcmBuf, samples * sizeof(int16_t));
        for (uint16_t i = 0; i < samples; i++)
            sidetone[i] = (int16_t)(sidetone[i] >> SIDETONE_SHIFT);

        RxFrame sf = {};
        sf.count = samples;
        sf.gap   = false;
        sf.concealed = false;
        memcpy(sf.pcm, sidetone, samples * sizeof(int16_t));
        xQueueSend(sidetoneQueue, &sf, 0);

        uint16_t offset = 0;
        while (offset < samples) {
            uint16_t count = samples - offset;
            if (count > ADPCM_SAMPLES_PER_PKT) count = ADPCM_SAMPLES_PER_PKT;

            bool isKeyframe = ((gTxPktCounter % KEYFRAME_INTERVAL) == 0);
            if (isKeyframe) encState = {0, 0};

            AdpcmState snapState = encState;
            adpcm_encode_block(&pcmBuf[offset], adpcmOut, count, encState);

            uint8_t bytes = (uint8_t)((count + 1) >> 1);
            if (bytes > MAX_DATA_LEN) bytes = MAX_DATA_LEN;

            Packet pkt;
            pkt.type      = isKeyframe ? PKT_KEYFRAME : PKT_AUDIO;
            pkt.seq       = gTxSeq++;
            pkt.adpcmPred = snapState.pred;
            pkt.adpcmIdx  = snapState.idx;
            pkt.dataLen   = bytes;
            memcpy(pkt.data, adpcmOut, bytes);
            pkt.appCrc = crc8(pkt.data, bytes);

            gTxPktCounter++;

            TxPacket tp;
            memcpy(tp.raw, &pkt, NRF_PAYLOAD_SIZE);

            if (xQueueSend(txQueue, &tp, 0) != pdTRUE) {
                TxPacket dummy;
                xQueueReceive(txQueue, &dummy, 0);
                xQueueSend(txQueue, &tp, 0);
            }
            offset += count;
        }
    }
}

static void task_transmit(void *) {
    TxPacket          tp;
    TickType_t        xLastWakeTime = xTaskGetTickCount();
    const TickType_t  xInterval     = pdUS_TO_TICKS(TX_INTERVAL_US);

    while (true) {
        if (!gTransmitting.load()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        if (xQueueReceive(txQueue, &tp, pdMS_TO_TICKS(5)) == pdTRUE) {
            xSemaphoreTake(radioMutex, portMAX_DELAY);
            radio.write(tp.raw, NRF_PAYLOAD_SIZE);
            xSemaphoreGive(radioMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xInterval);
    }
}

static void task_receive(void *) {
    while (true) {
        if (gTransmitting.load()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        xSemaphoreTake(radioMutex, portMAX_DELAY);
        bool avail = radio.available();
        if (!avail) {
            xSemaphoreGive(radioMutex);
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        Packet pkt;
        radio.read(&pkt, NRF_PAYLOAD_SIZE);
        gRssiLast = radio.testRPD() ? -60 : -90;
        xSemaphoreGive(radioMutex);

        if (pkt.type == PKT_END) {
            gRxLastSeq       = 0xFF;
            gRxFirstPacket   = true;
            gLastDecodeState = {0, 0};
            gPlaybackReady.store(false);
            continue;
        }

        if ((pkt.type != PKT_AUDIO && pkt.type != PKT_KEYFRAME) ||
            pkt.dataLen == 0 || pkt.dataLen > MAX_DATA_LEN) continue;

        uint8_t expectedCrc = crc8(pkt.data, pkt.dataLen);
        if (pkt.appCrc != expectedCrc) {
            gPktsLost++;
            gLostInWindow++;
            continue;
        }

        if (pkt.type == PKT_KEYFRAME) {
            gLastDecodeState = {pkt.adpcmPred, pkt.adpcmIdx};
            gRxFirstPacket   = false;
        }

        bool gap = false;
        if (gRxLastSeq != 0xFF) {
            uint8_t expected = (uint8_t)(gRxLastSeq + 1);
            uint8_t lost     = (uint8_t)(pkt.seq - expected);
            if (lost > 0 && lost < 10) {
                gPktsLost     += lost;
                gLostInWindow += lost;

                for (uint8_t l = 0; l < lost; l++) {
                    RxFrame conceal = {};
                    conceal.count     = ADPCM_SAMPLES_PER_PKT;
                    conceal.gap       = (l == 0);
                    conceal.concealed = true;
                    adpcm_conceal_block(conceal.pcm, conceal.count, gLastDecodeState, 3);
                    if (xQueueSend(rxQueue, &conceal, 0) != pdTRUE) {
                        RxFrame dummy;
                        xQueueReceive(rxQueue, &dummy, 0);
                        xQueueSend(rxQueue, &conceal, 0);
                    }
                }
                gap = true;
            }
        }

        gRxLastSeq = pkt.seq;
        gPktsInWindow++;
        gPktsReceived++;

        uint32_t squelchThresh = (uint32_t)gSquelch;
        squelchThresh = (squelchThresh * squelchThresh) >> 4;
        uint32_t energy = compute_energy(pkt.data, pkt.dataLen);

        if (energy >= squelchThresh) {
            AdpcmState ds;
            if (gRxFirstPacket || gap || pkt.type == PKT_KEYFRAME) {
                ds             = {pkt.adpcmPred, pkt.adpcmIdx};
                gRxFirstPacket = false;
            } else {
                ds = gLastDecodeState;
            }

            RxFrame frame = {};
            frame.count     = (uint16_t)(pkt.dataLen * 2);
            frame.gap       = gap;
            frame.concealed = false;
            adpcm_decode_block(pkt.data, frame.pcm, pkt.dataLen * 2, ds);
            apply_deemphasis(frame.pcm, frame.count);
            gLastDecodeState = ds;

            if (xQueueSend(rxQueue, &frame, 0) != pdTRUE) {
                RxFrame dummy;
                xQueueReceive(rxQueue, &dummy, 0);
                xQueueSend(rxQueue, &frame, 0);
            }

            uint8_t target = (uint8_t)gJbTarget;
            if (!gPlaybackReady.load() && uxQueueMessagesWaiting(rxQueue) >= target) {
                gPlaybackReady.store(true);
            }
        }
    }
}

static void task_playback(void *) {
    static RxFrame frame;
    while (true) {
        if (gTransmitting.load()) {
            RxFrame sf;
            if (xQueueReceive(sidetoneQueue, &sf, pdMS_TO_TICKS(5)) == pdTRUE) {
                uint8_t vol     = (uint8_t)gVolume;
                uint8_t logVol  = LOG_VOL_TABLE[vol - VOLUME_MIN];
                if (logVol != 128) {
                    for (uint16_t i = 0; i < sf.count; i++) {
                        int32_t s = ((int32_t)sf.pcm[i] * logVol) >> 8;
                        sf.pcm[i] = (int16_t)((s > 32767) ? 32767 : (s < -32768) ? -32768 : s);
                    }
                }
                size_t written = 0;
                i2s_write(I2S_NUM_0, sf.pcm, sf.count * sizeof(int16_t), &written, pdMS_TO_TICKS(10));
            }
            gPlaybackReady.store(false);
            continue;
        }

        if (!gPlaybackReady.load()) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        if (xQueueReceive(rxQueue, &frame, pdMS_TO_TICKS(20)) != pdTRUE) {
            gLastUnderflow = millis();
            gPlaybackReady.store(false);
            continue;
        }

        if (frame.gap) apply_fade_in(frame.pcm, frame.count, 16);
        if (frame.concealed) apply_fade_out(frame.pcm, frame.count, 8);

        uint8_t vol    = (uint8_t)gVolume;
        uint8_t logVol = LOG_VOL_TABLE[vol - VOLUME_MIN];
        if (logVol != 128) {
            for (uint16_t i = 0; i < frame.count; i++) {
                int32_t s = ((int32_t)frame.pcm[i] * logVol) >> 7;
                frame.pcm[i] = (int16_t)((s > 32767) ? 32767 : (s < -32768) ? -32768 : s);
            }
        }

        size_t written = 0;
        i2s_write(I2S_NUM_0, frame.pcm, frame.count * sizeof(int16_t), &written, pdMS_TO_TICKS(30));
    }
}

static void task_ui(void *) {
    uint32_t lastBat     = 0;
    uint32_t lastDisplay = 0;
    bool     lastPtt     = false;

    while (true) {
        uint32_t now = millis();
        bool ptt = (digitalRead(BTN_PTT_PIN) == LOW);

        if (ptt != lastPtt) {
            lastPtt       = ptt;
            bool transmit = ptt;
            gTransmitting.store(transmit);
            gLastActivity = now;

            xSemaphoreTake(radioMutex, portMAX_DELAY);
            if (transmit) {
                gRxLastSeq       = 0xFF;
                gLastDecodeState = {0, 0};
                gHpZ1            = 0;
                gHpZ2            = 0;
                gLpZ             = 0;
                gAgcGain         = 256;
                gAgcEnvelope     = 0;
                gNgOpen          = false;
                gNgHoldCounter   = 0;
                gNgGain          = 0;
                gNgGainTarget    = 0;
                gPreempZ         = 0;
                gTxFirstFrame    = true;
                gTxPktCounter    = 0;
                xQueueReset(txQueue);
                xQueueReset(rxQueue);
                xQueueReset(sidetoneQueue);
                radio.stopListening();
            } else {
                Packet endPkt = {};
                endPkt.type  = PKT_END;
                endPkt.seq   = gTxSeq++;
                radio.write(&endPkt, NRF_PAYLOAD_SIZE);
                radio.txStandBy();
                xQueueReset(txQueue);
                xQueueReset(rxQueue);
                xQueueReset(sidetoneQueue);
                gRxFirstPacket = true;
                gPlaybackReady.store(false);
                gDeempZ = 0;
                radio.startListening();
            }
            xSemaphoreGive(radioMutex);
            gDisplayDirty = true;
        }

        handle_menu_buttons();
        update_signal_bars();
        update_jitter_buffer();

        if (now - lastBat >= BAT_READ_MS) {
            lastBat     = now;
            float newV  = read_battery_voltage();
            gBatVoltage = (gBatVoltage == 0.0f) ? newV : (0.9f * gBatVoltage + 0.1f * newV);
            gBatPct     = lipo_pct(gBatVoltage);
            gBatMv      = (uint32_t)(gBatVoltage * 1000.0f);
            gBatWarning  = (gBatMv < BAT_LOW_MV);
            gBatCritical = (gBatMv < BAT_CRITICAL_MV);
            gDisplayDirty = true;
        }

        uint32_t idleMs = now - gLastActivity;
        if (!gDisplayOff && !gDisplayDimmed && idleMs >= DISPLAY_DIM_MS) {
            gDisplayDimmed = true;
            display.oled_command(0xAE);
            display.oled_command(0x81);
            display.oled_command(0x10);
        }
        if (!gDisplayOff && gDisplayDimmed && idleMs >= DISPLAY_OFF_MS) {
            gDisplayOff    = true;
            gDisplayDimmed = false;
            display.oled_command(0xAE);
        }

        if (gBatCritical) gDisplayDirty = true;

        if (gDisplayDirty.load() && !gDisplayOff && (now - lastDisplay >= DISPLAY_UPDATE_MS)) {
            lastDisplay   = now;
            gDisplayDirty = false;
            update_display();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    ESP_ERROR_CHECK(esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
        ADC_WIDTH_BIT_12, 1100, &adcChars) == ESP_ADC_CAL_VAL_NOT_SUPPORTED
        ? ESP_FAIL : ESP_OK);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    for (uint8_t i = 0; i < 5; i++) {
        pinMode(BTN_PINS[i], INPUT_PULLUP);
        btnStates[i] = {};
    }

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(400000);
    display.begin(OLED_ADDR, true);
    display.clearDisplay();
    display.display();

    radio_init();
    i2s_speaker_init();
    i2s_mic_init();

    gBatVoltage        = read_battery_voltage();
    gBatPct            = lipo_pct(gBatVoltage);
    gBatMv             = (uint32_t)(gBatVoltage * 1000.0f);
    gBatWarning        = (gBatMv < BAT_LOW_MV);
    gBatCritical       = (gBatMv < BAT_CRITICAL_MV);
    gSignalWindowStart = millis();
    gLastActivity      = millis();
    gJbLastAdjust      = millis();
    gNgGain            = 0;
    gNgGainTarget      = 0;

    txQueue       = xQueueCreate(TX_RING_PKTS, sizeof(TxPacket));
    rxQueue       = xQueueCreate(RX_RING_PKTS, sizeof(RxFrame));
    sidetoneQueue = xQueueCreate(4,            sizeof(RxFrame));
    radioMutex    = xSemaphoreCreateMutex();

    configASSERT(txQueue);
    configASSERT(rxQueue);
    configASSERT(sidetoneQueue);
    configASSERT(radioMutex);

    xTaskCreatePinnedToCore(task_capture,  "cap",  4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(task_transmit, "tx",   3072, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(task_receive,  "rx",   3072, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(task_playback, "play", 3072, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(task_ui,       "ui",   4096, nullptr, 2, nullptr, 0);

    update_display();
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}