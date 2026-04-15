#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include "config.h"

static const int16_t ADPCM_STEP_TABLE[89] = {
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,
    66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,
    371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,
    1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
    5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767
};
static const int8_t ADPCM_INDEX_TABLE[16] = {
    -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8
};

struct AdpcmState { int16_t pred; int8_t idx; };

static uint8_t adpcm_encode_sample(int16_t sample, AdpcmState &s) {
    int16_t step = ADPCM_STEP_TABLE[s.idx];
    int32_t diff = (int32_t)sample - s.pred;
    uint8_t code = 0;
    if (diff < 0) { code = 8; diff = -diff; }
    if (diff >= step)        { code |= 4; diff -= step; }
    if (diff >= (step >> 1)) { code |= 2; diff -= step >> 1; }
    if (diff >= (step >> 2)) { code |= 1; }
    int32_t diffq = (step >> 3);
    if (code & 4) diffq += step;
    if (code & 2) diffq += step >> 1;
    if (code & 1) diffq += step >> 2;
    s.pred += (code & 8) ? -diffq : diffq;
    s.pred = (int16_t)constrain(s.pred, -32768, 32767);
    s.idx  = (int8_t)constrain(s.idx + ADPCM_INDEX_TABLE[code & 0x7], 0, 88);
    return code & 0xF;
}

static int16_t adpcm_decode_sample(uint8_t code, AdpcmState &s) {
    int16_t step = ADPCM_STEP_TABLE[s.idx];
    int32_t diffq = (step >> 3);
    if (code & 4) diffq += step;
    if (code & 2) diffq += step >> 1;
    if (code & 1) diffq += step >> 2;
    s.pred += (code & 8) ? -diffq : diffq;
    s.pred = (int16_t)constrain(s.pred, -32768, 32767);
    s.idx  = (int8_t)constrain(s.idx + ADPCM_INDEX_TABLE[code & 0x7], 0, 88);
    return s.pred;
}

static void adpcm_encode(const int16_t *in, uint8_t *out, uint16_t samples, AdpcmState &s) {
    for (uint16_t i = 0; i < samples; i += 2) {
        uint8_t lo = adpcm_encode_sample(in[i],     s);
        uint8_t hi = (i+1 < samples) ? adpcm_encode_sample(in[i+1], s) : 0;
        out[i >> 1] = lo | (hi << 4);
    }
}

static void adpcm_decode(const uint8_t *in, int16_t *out, uint16_t nibbles, AdpcmState &s) {
    for (uint16_t i = 0; i < nibbles; i++) {
        out[i*2]   = adpcm_decode_sample(in[i] & 0xF,  s);
        out[i*2+1] = adpcm_decode_sample(in[i] >> 4,   s);
    }
}

struct __attribute__((packed)) Packet {
    uint8_t  type;
    uint8_t  seq;
    int16_t  adpcmPred;
    int8_t   adpcmIdx;
    uint8_t  dataLen;
    uint8_t  data[NRF_PAYLOAD_SIZE - 6];
};
static_assert(sizeof(Packet) == NRF_PAYLOAD_SIZE, "Packet size mismatch");

struct TxPacket { uint8_t raw[NRF_PAYLOAD_SIZE]; };
struct RxFrame {
    int16_t  pcm[ADPCM_SAMPLES_PER_PKT * 2];
    uint16_t count;
    bool     gap;
};

static QueueHandle_t    txQueue;
static QueueHandle_t    rxQueue;
static SemaphoreHandle_t radioMutex;

static RF24               radio(NRF_CE_PIN, NRF_CSN_PIN);
static Adafruit_SH1106G   display(128, 64, &Wire, -1);
static esp_adc_cal_characteristics_t adcChars;

static std::atomic<bool>    gTransmitting{false};
static std::atomic<uint8_t> gChannelIdx{DEFAULT_CHANNEL_IDX};
static std::atomic<uint8_t> gVolume{VOLUME_DEFAULT};
static std::atomic<uint16_t> gSquelch{SQUELCH_DEFAULT};
static volatile bool     gDisplayDirty = true;

static float    gBatVoltage  = 0.0f;
static uint8_t  gBatPct      = 0;
static uint8_t  gSignalBars  = 0;
static uint32_t gPktsReceived = 0;
static uint32_t gPktsLost     = 0;
static uint32_t gSignalWindowStart = 0;
static uint32_t gPktsInWindow = 0;
static uint32_t gLostInWindow = 0;

static uint8_t  gTxSeq    = 0;
static uint8_t  gRxLastSeq = 0xFF;

enum MenuState { MENU_NONE, MENU_MAIN, MENU_VOLUME, MENU_SQUELCH, MENU_CHANNEL };
static volatile MenuState gMenu    = MENU_NONE;
static volatile uint8_t   gMenuSel = 0;

static void i2s_speaker_init() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 4;
    cfg.dma_buf_len          = 128;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = I2S_BCLK_PIN;
    pins.ws_io_num    = I2S_WCLK_PIN;
    pins.data_out_num = I2S_DATA_PIN;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;

    i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pins);
}

static void i2s_mic_init() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 4;
    cfg.dma_buf_len          = 128;
    cfg.use_apll             = false;

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = I2S_MIC_SCK_PIN;
    pins.ws_io_num    = I2S_MIC_WS_PIN;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num  = I2S_MIC_SD_PIN;

    i2s_driver_install(I2S_NUM_1, &cfg, 0, nullptr);
    i2s_set_pin(I2S_NUM_1, &pins);
}

static float read_battery_voltage() {
    uint32_t raw = 0;
    for (int i = 0; i < 16; i++) raw += analogRead(BAT_ADC_PIN);
    raw >>= 4;
    uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adcChars);
    float vadc = mv / 1000.0f;
    return vadc * (BAT_R1 + BAT_R2) / BAT_R2;
}

static uint8_t lipo_pct(float v) {
    if (v >= LIPO_CURVE[0].v) return 100;
    if (v <= LIPO_CURVE[LIPO_CURVE_LEN-1].v) return 0;
    for (int i = 0; i < LIPO_CURVE_LEN-1; i++) {
        if (v >= LIPO_CURVE[i+1].v) {
            float span = LIPO_CURVE[i].v - LIPO_CURVE[i+1].v;
            float pos  = v - LIPO_CURVE[i+1].v;
            float frac = pos / span;
            return (uint8_t)(LIPO_CURVE[i+1].pct + frac * (LIPO_CURVE[i].pct - LIPO_CURVE[i+1].pct));
        }
    }
    return 0;
}

static uint16_t compute_rms(const int16_t *buf, uint16_t len) {
    uint64_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += (int32_t)buf[i] * buf[i];
    return (uint16_t)sqrtf((float)(sum / len));
}

static void remove_dc_offset(int16_t *buf, uint16_t len) {
    int32_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += buf[i];
    int16_t dc = (int16_t)(sum / len);
    for (uint16_t i = 0; i < len; i++) buf[i] -= dc;
}

static void apply_limiter(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        if      (buf[i] >  29000) buf[i] =  29000;
        else if (buf[i] < -29000) buf[i] = -29000;
    }
}

static void apply_fade_in(int16_t *buf, uint16_t len, uint8_t fadeLen) {
    uint8_t n = (len < fadeLen) ? len : fadeLen;
    for (uint8_t i = 0; i < n; i++) {
        buf[i] = (int16_t)(((int32_t)buf[i] * i) / n);
    }
}

static void update_signal_bars() {
    uint32_t now     = millis();
    uint32_t elapsed = now - gSignalWindowStart;
    if (elapsed >= SIGNAL_WINDOW_MS) {
        uint32_t total = gPktsInWindow + gLostInWindow;
        uint8_t quality = (total > 0) ? (uint8_t)((gPktsInWindow * 100) / total) : 0;
        if      (quality > 90) gSignalBars = 4;
        else if (quality > 70) gSignalBars = 3;
        else if (quality > 40) gSignalBars = 2;
        else if (quality > 10) gSignalBars = 1;
        else                   gSignalBars = 0;
        gPktsInWindow      = 0;
        gLostInWindow      = 0;
        gSignalWindowStart = now;
    }
}

static void draw_battery_icon(uint8_t x, uint8_t y, uint8_t pct) {
    display.drawRect(x, y, 18, 8, SH110X_WHITE);
    display.fillRect(x+18, y+2, 2, 4, SH110X_WHITE);
    uint8_t fill = (uint8_t)(pct / 100.0f * 16.0f);
    if (fill > 0) display.fillRect(x+1, y+1, fill, 6, SH110X_WHITE);
}

static void draw_signal_bars(uint8_t x, uint8_t y, uint8_t bars) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t h  = (i+1)*3;
        uint8_t bx = x + i*5;
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
    display.setCursor(0, 24);
    display.print("SQ:");
    display.print((uint16_t)gSquelch);
    display.print("  VOL:");
    display.print((uint8_t)gVolume);
    display.drawFastHLine(0, 35, 128, SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 40);
    if (gTransmitting) {
        display.print("TX \x10\x10\x10");
    } else {
        display.print("RX \xb7\xb7\xb7");
    }
    display.display();
}

static const char* MENU_ITEMS[] = {"Volume", "Squelch", "Channel", "Back"};
#define MENU_ITEM_COUNT 4

static void draw_menu_screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(28, 0);
    display.print("[ SETTINGS ]");
    display.drawFastHLine(0, 9, 128, SH110X_WHITE);
    for (uint8_t i = 0; i < MENU_ITEM_COUNT; i++) {
        display.setCursor(8, 12 + i*13);
        if (i == gMenuSel) {
            display.fillRect(0, 11 + i*13, 128, 12, SH110X_WHITE);
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
    uint8_t barW = (uint8_t)(((float)(val-minv)/(float)(maxv-minv)) * 120.0f);
    display.drawRect(4, 54, 120, 8, SH110X_WHITE);
    display.fillRect(4, 54, barW, 8, SH110X_WHITE);
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
        case MENU_NONE:    draw_main_screen(); break;
        case MENU_MAIN:    draw_menu_screen(); break;
        case MENU_VOLUME:  draw_value_screen("Volume",  gVolume,  VOLUME_MIN,  VOLUME_MAX);  break;
        case MENU_SQUELCH: draw_value_screen("Squelch", gSquelch, SQUELCH_MIN, SQUELCH_MAX); break;
        case MENU_CHANNEL: draw_channel_screen(); break;
    }
}

static void radio_apply_channel(uint8_t idx) {
    radio.stopListening();
    radio.setChannel(CHANNEL_LIST[idx]);
    if (!gTransmitting) radio.startListening();
}

static void radio_init() {
    SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
    radio.begin();
    radio.setPALevel(NRF_PA_LEVEL);
    radio.setDataRate(NRF_DATA_RATE);
    radio.setPayloadSize(NRF_PAYLOAD_SIZE);
    radio.setChannel(CHANNEL_LIST[gChannelIdx]);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.setRetries(2, 3);
    radio.openWritingPipe(NRF_PIPE_ADDR);
    radio.openReadingPipe(1, NRF_PIPE_ADDR);
    radio.startListening();
}

struct BtnState { uint8_t last; uint32_t lastMs; };
static BtnState btnStates[5];
static const uint8_t BTN_PINS[5] = {BTN_UP_PIN,BTN_DOWN_PIN,BTN_BACK_PIN,BTN_SELECT_PIN,BTN_PTT_PIN};

static bool btn_fell(uint8_t idx) {
    uint8_t raw = (digitalRead(BTN_PINS[idx]) == LOW) ? 1 : 0;
    uint32_t now = millis();
    if (raw != btnStates[idx].last && (now - btnStates[idx].lastMs) > DEBOUNCE_MS) {
        btnStates[idx].last   = raw;
        btnStates[idx].lastMs = now;
        return raw == 1;
    }
    return false;
}

static void handle_menu_buttons() {
    if (btn_fell(0)) {
        switch (gMenu) {
            case MENU_MAIN:
                if (gMenuSel > 0) gMenuSel--;
                break;
            case MENU_VOLUME:
                if ((uint8_t)gVolume < VOLUME_MAX) gVolume++;
                break;
            case MENU_SQUELCH:
                if ((uint16_t)gSquelch < SQUELCH_MAX) gSquelch = gSquelch + SQUELCH_STEP;
                break;
            case MENU_CHANNEL:
                if ((uint8_t)gChannelIdx < CHANNEL_COUNT-1) {
                    gChannelIdx++;
                    xSemaphoreTake(radioMutex, portMAX_DELAY);
                    radio_apply_channel(gChannelIdx);
                    xSemaphoreGive(radioMutex);
                }
                break;
            default: break;
        }
        gDisplayDirty = true;
    }
    if (btn_fell(1)) {
        switch (gMenu) {
            case MENU_MAIN:
                if (gMenuSel < MENU_ITEM_COUNT-1) gMenuSel++;
                break;
            case MENU_VOLUME:
                if ((uint8_t)gVolume > VOLUME_MIN) gVolume--;
                break;
            case MENU_SQUELCH:
                if ((uint16_t)gSquelch >= (uint16_t)(SQUELCH_MIN + SQUELCH_STEP)) gSquelch = gSquelch - SQUELCH_STEP;
                break;
            case MENU_CHANNEL:
                if ((uint8_t)gChannelIdx > 0) {
                    gChannelIdx--;
                    xSemaphoreTake(radioMutex, portMAX_DELAY);
                    radio_apply_channel(gChannelIdx);
                    xSemaphoreGive(radioMutex);
                }
                break;
            default: break;
        }
        gDisplayDirty = true;
    }
    if (btn_fell(2)) {
        if (gMenu != MENU_NONE) {
            gMenu = (gMenu == MENU_MAIN) ? MENU_NONE : MENU_MAIN;
            gMenuSel = 0;
            gDisplayDirty = true;
        }
    }
    if (btn_fell(3)) {
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

static void task_capture(void *) {
    static int16_t    pcmBuf[MIC_CAPTURE_SAMPLES];
    static AdpcmState encState = {0, 0};
    static uint8_t    adpcmOut[MIC_CAPTURE_SAMPLES / 2];

    while (true) {
        if (!gTransmitting) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        size_t got = 0;
        i2s_read(I2S_NUM_1, pcmBuf, MIC_CAPTURE_SAMPLES * sizeof(int16_t), &got, pdMS_TO_TICKS(10));
        uint16_t samples = got / sizeof(int16_t);
        if (samples == 0) continue;

        remove_dc_offset(pcmBuf, samples);
        apply_limiter(pcmBuf, samples);

        uint16_t offset = 0;
        while (offset < samples) {
            uint16_t count = (samples - offset) < ADPCM_SAMPLES_PER_PKT
                             ? (samples - offset) : ADPCM_SAMPLES_PER_PKT;
            uint16_t bytes = (count + 1) / 2;

            AdpcmState snapState = encState;
            adpcm_encode(&pcmBuf[offset], adpcmOut, count, encState);

            Packet pkt;
            pkt.type      = PKT_AUDIO;
            pkt.seq       = gTxSeq++;
            pkt.adpcmPred = snapState.pred;
            pkt.adpcmIdx  = snapState.idx;
            pkt.dataLen   = (uint8_t)bytes;
            memcpy(pkt.data, adpcmOut, bytes);

            TxPacket tp;
            memcpy(tp.raw, &pkt, NRF_PAYLOAD_SIZE);

            if (xQueueSend(txQueue, &tp, 0) != pdTRUE) {
                uint8_t dummy[sizeof(TxPacket)];
                xQueueReceive(txQueue, dummy, 0);
                xQueueSend(txQueue, &tp, 0);
            }
            offset += count;
        }
    }
}

static void task_transmit(void *) {
    TxPacket tp;
    while (true) {
        if (!gTransmitting) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (xQueueReceive(txQueue, &tp, pdMS_TO_TICKS(10)) == pdTRUE) {
            xSemaphoreTake(radioMutex, portMAX_DELAY);
            radio.write(tp.raw, NRF_PAYLOAD_SIZE);
            xSemaphoreGive(radioMutex);
        }
    }
}

static void task_receive(void *) {
    while (true) {
        if (gTransmitting) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        bool avail = radio.available();
        if (avail) {
            Packet pkt;
            radio.read(&pkt, NRF_PAYLOAD_SIZE);
            xSemaphoreGive(radioMutex);

            if (pkt.type == PKT_AUDIO && pkt.dataLen > 0 &&
                pkt.dataLen <= (NRF_PAYLOAD_SIZE - 6)) {

                if (gRxLastSeq != 0xFF) {
                    uint8_t expected = (uint8_t)(gRxLastSeq + 1);
                    uint8_t lost     = (uint8_t)(pkt.seq - expected);
                    if (lost > 0 && lost < 10) {
                        gPktsLost    += lost;
                        gLostInWindow += lost;
                        RxFrame silence;
                        silence.count = ADPCM_SAMPLES_PER_PKT;
                        silence.gap   = false;
                        memset(silence.pcm, 0, sizeof(silence.pcm));
                        xQueueSend(rxQueue, &silence, 0);
                    }
                }
                gRxLastSeq = pkt.seq;
                gPktsInWindow++;
                gPktsReceived++;

                AdpcmState ds = {pkt.adpcmPred, pkt.adpcmIdx};
                RxFrame frame;
                frame.count = pkt.dataLen * 2;
                frame.gap   = (gRxLastSeq != (uint8_t)(pkt.seq));
                adpcm_decode(pkt.data, frame.pcm, pkt.dataLen, ds);

                uint16_t rms = compute_rms(frame.pcm, frame.count);
                if (rms >= (uint16_t)gSquelch) {
                    if (xQueueSend(rxQueue, &frame, 0) != pdTRUE) {
                        RxFrame dummy;
                        xQueueReceive(rxQueue, &dummy, 0);
                        xQueueSend(rxQueue, &frame, 0);
                    }
                }
            }
        } else {
            xSemaphoreGive(radioMutex);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

static void task_playback(void *) {
    RxFrame frame;
    while (true) {
        if (gTransmitting) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (xQueueReceive(rxQueue, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (frame.gap) {
                apply_fade_in(frame.pcm, frame.count, 8);
            }
            uint8_t vol = gVolume;
            for (uint16_t i = 0; i < frame.count; i++) {
                frame.pcm[i] = (int16_t)constrain(
                    ((int32_t)frame.pcm[i] * vol) / VOLUME_DEFAULT,
                    -32768, 32767);
            }
            size_t written = 0;
            i2s_write(I2S_NUM_0, frame.pcm, frame.count * sizeof(int16_t), &written, pdMS_TO_TICKS(20));
        }
    }
}

static void task_ui(void *) {
    uint32_t lastBat     = 0;
    uint32_t lastDisplay = 0;

    while (true) {
        uint32_t now = millis();
        bool ptt = (digitalRead(BTN_PTT_PIN) == LOW);

        if (ptt != (bool)gTransmitting) {
            gTransmitting = ptt;
            xSemaphoreTake(radioMutex, portMAX_DELAY);
            if (ptt) {
                gRxLastSeq = 0xFF;
                xQueueReset(txQueue);
                radio.stopListening();
            } else {
                Packet endPkt;
                endPkt.type      = PKT_END;
                endPkt.seq       = gTxSeq++;
                endPkt.adpcmPred = 0;
                endPkt.adpcmIdx  = 0;
                endPkt.dataLen   = 0;
                radio.write(&endPkt, NRF_PAYLOAD_SIZE);
                radio.txStandBy();
                radio.startListening();
                xQueueReset(rxQueue);
            }
            xSemaphoreGive(radioMutex);
            gDisplayDirty = true;
        }

        handle_menu_buttons();
        update_signal_bars();

        if (now - lastBat >= BAT_READ_MS) {
            lastBat     = now;
            gBatVoltage = read_battery_voltage();
            gBatPct     = lipo_pct(gBatVoltage);
            gDisplayDirty = true;
        }

        if (gDisplayDirty && (now - lastDisplay >= DISPLAY_UPDATE_MS)) {
            lastDisplay   = now;
            gDisplayDirty = false;
            update_display();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adcChars);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    for (uint8_t i = 0; i < 5; i++) {
        pinMode(BTN_PINS[i], INPUT_PULLUP);
        btnStates[i] = {0, 0};
    }

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    display.begin(OLED_ADDR, true);
    display.clearDisplay();
    display.display();

    radio_init();
    i2s_speaker_init();
    i2s_mic_init();

    gBatVoltage = read_battery_voltage();
    gBatPct     = lipo_pct(gBatVoltage);
    gSignalWindowStart = millis();

    txQueue    = xQueueCreate(TX_RING_PKTS, sizeof(TxPacket));
    rxQueue    = xQueueCreate(RX_RING_PKTS, sizeof(RxFrame));
    radioMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(task_capture,  "cap",  4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(task_transmit, "tx",   3072, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(task_receive,  "rx",   3072, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(task_playback, "play", 3072, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(task_ui,       "ui",   4096, nullptr, 2, nullptr, 0);

    update_display();
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}