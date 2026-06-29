#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <cstring>
#include "config.h"
#include "crypto.h"

// ---- Over-the-air packet types (kept out of config.h to avoid macro clashes) ----
static constexpr uint8_t  PKT_AUDIO    = 0x01;
static constexpr uint8_t  PKT_END      = 0x02;
static constexpr uint8_t  PKT_KEYFRAME = 0x03;

static constexpr uint8_t  KEYFRAME_INTERVAL   = 16;
static constexpr uint8_t  MIN_PLAYBACK_FRAMES = 4;
static constexpr uint8_t  MAX_PLAYBACK_FRAMES = 12;
static constexpr uint32_t JB_GROW_THRESH_MS   = 80;
static constexpr uint32_t JB_SHRINK_THRESH_MS = 400;
static constexpr uint8_t  SIDETONE_SHIFT       = 3;

static constexpr uint32_t BAT_LOW_MV       = 3400;
static constexpr uint32_t BAT_CRITICAL_MV  = 3200;
static constexpr uint32_t DISPLAY_DIM_MS   = 15000;
static constexpr uint32_t DISPLAY_OFF_MS   = 30000;
static constexpr uint32_t LONGPRESS_MS     = 600;
static constexpr uint32_t REPEAT_MS        = 120;

static constexpr uint8_t  APP_CRC_POLY = 0x97;

// Persist the TX nonce counter to NVS every this many packets so a reboot never
// reuses keystream (stream-cipher nonce reuse leaks plaintext). On boot we jump
// the counter forward by one stride to skip any window that wasn't persisted.
static constexpr uint32_t NONCE_PERSIST_STRIDE = 32768;
static constexpr uint32_t SETTINGS_SAVE_DELAY_MS = 3000;

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

// 1 kHz sine wave sampled at 8000 Hz (8 samples/period, amplitude ~16000)
static const int16_t TONE_1KHZ_TABLE[8] = {
    0, 11314, 16000, 11314, 0, -11314, -16000, -11314
};

struct AdpcmState { int16_t pred; int8_t idx; };

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
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? ((crc << 1) ^ APP_CRC_POLY) : (crc << 1);
    }
    return crc;
}

// Wire format — 11-byte header + 21 data bytes. nonce is a clear, monotonic
// per-packet counter used both as the ChaCha20 nonce and for diagnostics.
struct __attribute__((packed)) Packet {
    uint8_t  type;
    uint8_t  seq;
    uint32_t nonce;
    int16_t  adpcmPred;
    int8_t   adpcmIdx;
    uint8_t  dataLen;
    uint8_t  appCrc;
    uint8_t  data[NRF_PAYLOAD_SIZE - 11];
};
static_assert(sizeof(Packet) == NRF_PAYLOAD_SIZE, "Packet size mismatch");
static constexpr uint8_t MAX_DATA_LEN = NRF_PAYLOAD_SIZE - 11;
static_assert(MAX_DATA_LEN == ADPCM_BYTES_PER_PKT, "ADPCM_BYTES_PER_PKT must match payload");
static_assert(ADPCM_SAMPLES_PER_PKT == ADPCM_BYTES_PER_PKT * 2, "samples must be 2x bytes");
static_assert(MIC_CAPTURE_SAMPLES % ADPCM_SAMPLES_PER_PKT == 0, "capture must be whole packets");

struct TxPacket { uint8_t raw[NRF_PAYLOAD_SIZE]; };

// pcm sized to the largest user: sidetone copies a whole mic capture buffer
// (MIC_CAPTURE_SAMPLES), which is larger than a decoded audio frame.
struct RxFrame {
    int16_t  pcm[MIC_CAPTURE_SAMPLES];
    uint16_t count;
    bool     gap;
    bool     concealed;
};

static QueueHandle_t     txQueue;
static QueueHandle_t     rxQueue;
static QueueHandle_t     sidetoneQueue;
static SemaphoreHandle_t radioMutex;

static RF24             radio(NRF_CE_PIN, NRF_CSN_PIN);
static Adafruit_SSD1306  display(128, 64, &Wire, -1);
static esp_adc_cal_characteristics_t adcChars;
static Preferences      prefs;

static std::atomic<bool>     gTransmitting{false};
static std::atomic<bool>     gPttHeld{false};
static std::atomic<bool>     gVoxActive{false};
static std::atomic<bool>     gVoxEnable{VOX_ENABLE_DEFAULT};
static std::atomic<bool>     gTotLatched{false};
static std::atomic<uint32_t> gTxStartMs{0};
static std::atomic<uint8_t>  gChannelIdx{DEFAULT_CHANNEL_IDX};
static std::atomic<uint8_t>  gVolume{VOLUME_DEFAULT};
static std::atomic<uint16_t> gSquelch{SQUELCH_DEFAULT};
static std::atomic<bool>     gDisplayDirty{true};

static std::atomic<uint32_t> gPktsReceived{0};
static std::atomic<uint32_t> gPktsLost{0};
static std::atomic<uint32_t> gPktsInWindow{0};
static std::atomic<uint32_t> gLostInWindow{0};
static std::atomic<uint32_t> gTxNonce{0};

static float    gBatVoltage  = 0.0f;
static uint32_t gBatMv       = 0;
static uint8_t  gBatPct      = 0;
static bool     gBatWarning  = false;
static bool     gBatCritical = false;
static uint8_t  gSignalBars  = 0;
static std::atomic<int8_t> gRssiLast{-127};
static uint32_t gSignalWindowStart = 0;

// audio level meter (bottom-left, 4 bars). Audio tasks publish a peak amplitude;
// task_ui applies a fast-attack/slow-decay envelope and maps it to 0..4 bars.
static std::atomic<uint16_t> gAudioPeak{0};
static uint16_t gAudioMeterEnv = 0;
static uint8_t  gAudioBars     = 0;

static uint8_t gTxSeq        = 0;
static uint8_t gRxLastSeq    = 0xFF;
static uint8_t gTxPktCounter = 0;

// audio pipeline state — all reset together on channel switch and TX/RX transitions
static AdpcmState gLastDecodeState = {0, 0};
static bool       gRxFirstPacket   = true;
static int32_t    gHpZ1 = 0, gHpZ2 = 0;
static int32_t    gLpZ  = 0;
static int32_t    gDeempZ  = 0;
static int32_t    gPreempZ = 0;
static int32_t    gEqZ1 = 0, gEqZ2 = 0;
static int32_t    gLimEnv = 0;
static bool       gTxFirstFrame = true;   // re-armed on every TX start for the onset fade-in
static std::atomic<bool>    gSoundTestActive{false};

static int32_t  gAgcGain       = 256;
static int32_t  gAgcEnvelope   = 0;
static uint32_t gNgHoldCounter = 0;
static bool     gNgOpen        = false;
static int32_t  gNgGain        = 0;
static int32_t  gNgGainTarget  = 0;

static std::atomic<bool>    gPlaybackReady{false};
static std::atomic<uint8_t> gJbTarget{MIN_PLAYBACK_FRAMES};
static uint32_t             gJbLastAdjust  = 0;
static uint32_t             gLastUnderflow = 0;

static uint32_t gLastActivity  = 0;
static bool     gDisplayDimmed = false;
static bool     gDisplayOff    = false;

// settings persistence
static bool     gSettingsDirty   = false;
static uint32_t gSettingsDirtyMs = 0;
static uint32_t gNonceSaved      = 0;

// VOX
static uint32_t gLastVoiceMs   = 0;
static uint32_t gVoxNoiseFloor = 300;   // tracked ambient mic energy (EMA while idle)
static uint8_t  gVoxAttack     = 0;     // consecutive loud frames before keying

// channel scan (task_ui only)
static uint8_t  gScanCh         = 0;
static uint32_t gScanDwellStart = 0;
static uint32_t gScanBaseRx     = 0;

// perceptual volume curve; value 128 = unity for >> 7 scaling, up to ~2x at max
static const uint8_t LOG_VOL_TABLE[VOLUME_MAX - VOLUME_MIN + 1] = {
    0,1,2,3,4,5,6,7,9,11,13,16,19,22,26,31,37,43,51,60,70,82,96,112,131,152,177,206,239,255
};

enum MenuState : uint8_t { MENU_NONE, MENU_MAIN, MENU_VOLUME, MENU_SQUELCH, MENU_CHANNEL, MENU_SCAN, MENU_SOUNDTEST };
static MenuState gMenu    = MENU_NONE;
static uint8_t   gMenuSel = 0;

static const char * const MENU_ITEMS[] = {"Channel", "Volume", "Squelch", "VOX", "Scan", "Snd Test"};
static constexpr uint8_t MENU_ITEM_COUNT    = 6;
static constexpr uint8_t MENU_IDX_CHANNEL   = 0;
static constexpr uint8_t MENU_IDX_VOLUME    = 1;
static constexpr uint8_t MENU_IDX_SQUELCH   = 2;
static constexpr uint8_t MENU_IDX_VOX       = 3;
static constexpr uint8_t MENU_IDX_SCAN      = 4;
static constexpr uint8_t MENU_IDX_SOUNDTEST = 5;
static constexpr uint8_t MENU_IDX_BACK      = 6;

// push to queue, drop oldest frame if full so audio tasks never block
template<typename T>
static inline void queue_push(QueueHandle_t q, const T &item) {
    if (xQueueSend(q, &item, 0) != pdTRUE) {
        T dummy;
        xQueueReceive(q, &dummy, 0);
        xQueueSend(q, &item, 0);
    }
}

// ---- Settings (NVS) -------------------------------------------------------

static void settings_mark_dirty() {
    gSettingsDirty   = true;
    gSettingsDirtyMs = millis();
}

static void settings_load() {
    prefs.begin("wt", false);
    uint8_t  vol = prefs.getUChar("vol", VOLUME_DEFAULT);
    uint16_t sq  = prefs.getUShort("sq", SQUELCH_DEFAULT);
    uint8_t  ch  = prefs.getUChar("ch", DEFAULT_CHANNEL_IDX);
    bool     vox = prefs.getBool("vox", VOX_ENABLE_DEFAULT);

    if (vol < VOLUME_MIN)  vol = VOLUME_MIN;
    if (vol > VOLUME_MAX)  vol = VOLUME_MAX;
    if (sq  > SQUELCH_MAX) sq  = SQUELCH_MAX;
    if (ch  >= CHANNEL_COUNT) ch = DEFAULT_CHANNEL_IDX;

    gVolume.store(vol);
    gSquelch.store(sq);
    gChannelIdx.store(ch);
    gVoxEnable.store(vox);

    // advance nonce past any unpersisted window to guarantee no keystream reuse
    uint32_t n = prefs.getUInt("nonce", 0) + NONCE_PERSIST_STRIDE;
    gTxNonce.store(n);
    gNonceSaved = n;
    prefs.putUInt("nonce", n);
}

// All NVS writes happen from task_ui to keep the Preferences handle single-owner.
static void settings_flush(uint32_t now) {
    if (gSettingsDirty && (now - gSettingsDirtyMs) >= SETTINGS_SAVE_DELAY_MS) {
        prefs.putUChar("vol", (uint8_t)gVolume.load());
        prefs.putUShort("sq", (uint16_t)gSquelch.load());
        prefs.putUChar("ch", (uint8_t)gChannelIdx.load());
        prefs.putBool("vox", gVoxEnable.load());
        gSettingsDirty = false;
    }
    uint32_t n = gTxNonce.load();
    if ((n - gNonceSaved) >= NONCE_PERSIST_STRIDE) {
        prefs.putUInt("nonce", n);
        gNonceSaved = n;
    }
}

// ---- I2S ------------------------------------------------------------------

static void i2s_speaker_init() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 4;     // ~64 ms max buffering — lower latency than 6x256
    cfg.dma_buf_len          = 128;
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
    cfg.dma_buf_count        = 4;
    cfg.dma_buf_len          = 128;
    cfg.use_apll             = true;

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = I2S_MIC_SCK_PIN;
    pins.ws_io_num    = I2S_MIC_WS_PIN;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num  = I2S_MIC_SD_PIN;

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &cfg, 0, nullptr));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pins));
}

// ---- Battery --------------------------------------------------------------

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

// ---- Audio processing -----------------------------------------------------

// use nibble magnitude as a cheap energy proxy for squelch (range ~0..2058)
static uint32_t compute_energy(const uint8_t *adpcmData, uint8_t len) {
    uint32_t energy = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t lo = adpcmData[i] & 0xF;
        uint8_t hi = adpcmData[i] >> 4;
        energy += (uint32_t)(lo & 7) * (lo & 7) + (uint32_t)(hi & 7) * (hi & 7);
    }
    return energy;
}

// mean-square/256 of PCM — used by VOX and matches the noise-gate metric
static uint32_t pcm_energy(const int16_t *buf, uint16_t len) {
    if (len == 0) return 0;
    int32_t acc = 0;
    for (uint16_t i = 0; i < len; i++) {
        int32_t s = buf[i];
        acc += (s * s) >> 8;
    }
    return (uint32_t)(acc / len);
}

// Publish the peak |sample| of a buffer for the on-screen audio meter. Cheap max
// against the current pending peak; task_ui consumes and clears it. TX and RX are
// half-duplex so only one producer is active at a time.
static void publish_audio_peak(const int16_t *buf, uint16_t len) {
    int32_t peak = 0;
    for (uint16_t i = 0; i < len; i++) {
        int32_t a = buf[i] < 0 ? -buf[i] : buf[i];
        if (a > peak) peak = a;
    }
    uint16_t p = (uint16_t)peak;
    if (p > gAudioPeak.load(std::memory_order_relaxed))
        gAudioPeak.store(p, std::memory_order_relaxed);
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
        gDeempZ  += ((x - gDeempZ) * 13) >> 4;
        buf[i] = (int16_t)((gDeempZ > 32767) ? 32767 : (gDeempZ < -32768) ? -32768 : gDeempZ);
    }
}

// narrow bandpass boost around 800–2400 Hz for speech intelligibility
static void apply_speech_eq(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t x        = (int32_t)buf[i];
        int32_t hp       = x - gEqZ1;
        gEqZ1           += (x - gEqZ1) >> 3;
        int32_t lp2      = gEqZ2 + ((x - gEqZ2) >> 1);
        gEqZ2            = lp2;
        int32_t bandpass = hp - (x - lp2);
        int32_t out      = x + (bandpass >> 2);
        buf[i] = (int16_t)((out > 32767) ? 32767 : (out < -32768) ? -32768 : out);
    }
}

static void apply_limiter(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t s   = buf[i];
        int32_t abs = s < 0 ? -s : s;
        if (abs > gLimEnv) gLimEnv = abs;
        else gLimEnv -= (gLimEnv - abs) >> 6;
        if (gLimEnv > 28000) s = (s * 28000) / gLimEnv;
        buf[i] = (int16_t)((s > 29000) ? 29000 : (s < -29000) ? -29000 : s);
    }
}

// Saturating left-shift: boosts quiet mic signals before the AGC/DSP chain.
// Clamps to [-32768, 32767] so loud inputs don't wrap — the limiter downstream
// then smooths any peaks that still hit the ceiling.
static void apply_pregain(int16_t *buf, uint16_t len, uint8_t shift) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t s = (int32_t)buf[i] << shift;
        buf[i] = (int16_t)((s > 32767) ? 32767 : (s < -32768) ? -32768 : s);
    }
}

static void apply_fade_in(int16_t *buf, uint16_t len, uint16_t fadeLen) {
    uint16_t n = (len < fadeLen) ? len : fadeLen;
    for (uint16_t i = 0; i < n; i++)
        buf[i] = (int16_t)(((int32_t)buf[i] * i) / n);
}

static void apply_fade_out(int16_t *buf, uint16_t len, uint16_t fadeLen) {
    if (len < fadeLen) fadeLen = len;
    uint16_t start = len - fadeLen;
    for (uint16_t i = 0; i < fadeLen; i++)
        buf[start + i] = (int16_t)(((int32_t)buf[start + i] * (fadeLen - i)) / fadeLen);
}

static void apply_agc(int16_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        int32_t abs = buf[i] < 0 ? -buf[i] : buf[i];
        if (abs > gAgcEnvelope) gAgcEnvelope += (abs - gAgcEnvelope) >> 2;
        else                    gAgcEnvelope += (abs - gAgcEnvelope) >> 8;
    }
    int32_t targetGain = (gAgcEnvelope > 0) ? (24000L * 256) / gAgcEnvelope : 256;
    if (targetGain > 4096) targetGain = 4096;   // 16x ceiling (was 4x) for INMP441
    if (targetGain < 64)   targetGain = 64;
    if (targetGain > gAgcGain) gAgcGain += (targetGain - gAgcGain) >> 6;   // attack (was >>9)
    else                       gAgcGain += (targetGain - gAgcGain) >> 4;   // fast release

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
    const int32_t releaseStep  = 256 / 64;

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

// ---- Signal quality -------------------------------------------------------

// Called every task_ui tick (~10 ms). Instant attack to the latest peak, gradual
// decay so the meter falls smoothly back to empty when audio stops.
static void update_audio_meter() {
    uint16_t peak = gAudioPeak.exchange(0, std::memory_order_relaxed);
    if (peak > gAudioMeterEnv) {
        gAudioMeterEnv = peak;
    } else if (gAudioMeterEnv > 0) {
        uint16_t d = (uint16_t)((gAudioMeterEnv >> 3) + 16);   // ~12.5%/tick + floor
        gAudioMeterEnv = (d >= gAudioMeterEnv) ? 0 : (uint16_t)(gAudioMeterEnv - d);
    }

    uint8_t bars;
    if      (gAudioMeterEnv > 18000) bars = 4;
    else if (gAudioMeterEnv >  9000) bars = 3;
    else if (gAudioMeterEnv >  3500) bars = 2;
    else if (gAudioMeterEnv >  1200) bars = 1;
    else                             bars = 0;

    if (bars != gAudioBars) {
        gAudioBars = bars;
        if (gMenu == MENU_NONE) gDisplayDirty = true;   // only the main screen shows it
    }
}

static void update_signal_bars() {
    uint32_t now     = millis();
    uint32_t elapsed = now - gSignalWindowStart;
    if (elapsed < SIGNAL_WINDOW_MS) return;

    uint32_t rx    = gPktsInWindow.exchange(0);
    uint32_t lost  = gLostInWindow.exchange(0);
    uint32_t total = rx + lost;

    uint8_t qualityPct = (total > 0) ? (uint8_t)((rx * 100u) / total) : 0;

    int8_t rssi = gRssiLast.load();
    uint8_t rssiBars = 0;
    if      (rssi > -60) rssiBars = 4;
    else if (rssi > -75) rssiBars = 3;
    else if (rssi > -85) rssiBars = 2;
    else if (rssi > -95) rssiBars = 1;

    uint8_t deliveryBars = 0;
    if      (qualityPct > 90) deliveryBars = 4;
    else if (qualityPct > 70) deliveryBars = 3;
    else if (qualityPct > 40) deliveryBars = 2;
    else if (qualityPct > 10) deliveryBars = 1;

    // worst of RSSI and packet delivery rate; show 0 when idle (no traffic)
    gSignalBars        = (total == 0) ? 0 : ((rssiBars < deliveryBars) ? rssiBars : deliveryBars);
    gSignalWindowStart = now;
}

static void update_jitter_buffer() {
    uint32_t now    = millis();
    uint32_t qDepth = (uint32_t)uxQueueMessagesWaiting(rxQueue);
    uint8_t  target = (uint8_t)gJbTarget;

    if (now - gLastUnderflow < JB_GROW_THRESH_MS) {
        if (target < MAX_PLAYBACK_FRAMES) {
            target++;
            gJbTarget.store(target);
            gJbLastAdjust = now;
        }
    } else if ((now - gJbLastAdjust) > JB_SHRINK_THRESH_MS && qDepth > (uint32_t)(target + 2) && target > MIN_PLAYBACK_FRAMES) {
        target--;
        gJbTarget.store(target);
        gJbLastAdjust = now;
    }
}

// ---- Display --------------------------------------------------------------

static void draw_battery_icon(uint8_t x, uint8_t y, uint8_t pct) {
    display.drawRect(x, y, 18, 8, SSD1306_WHITE);
    display.fillRect(x + 18, y + 2, 2, 4, SSD1306_WHITE);
    uint8_t fill = (uint8_t)(pct * 16u / 100u);
    if (fill > 0) display.fillRect(x + 1, y + 1, fill, 6, SSD1306_WHITE);
    if (gBatCritical && ((millis() / 500) & 1)) {
        display.fillRect(x, y, 20, 8, SSD1306_BLACK);
        display.drawRect(x, y, 18, 8, SSD1306_WHITE);
        display.fillRect(x + 18, y + 2, 2, 4, SSD1306_WHITE);
    }
}

static void draw_signal_bars(uint8_t x, uint8_t y, uint8_t bars) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t h  = (i + 1) * 3;
        uint8_t bx = x + i * 5;
        uint8_t by = y + (12 - h);
        if (i < bars) display.fillRect(bx, by, 4, h, SSD1306_WHITE);
        else          display.drawRect(bx, by, 4, h, SSD1306_WHITE);
    }
}

// Small audio level meter: 4 ascending bars filled up to `bars`. baseY is the
// bottom (exclusive) row, so bars grow upward from there.
static void draw_meter_bars(uint8_t x, uint8_t baseY, uint8_t bars) {
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t h  = (uint8_t)((i + 1) * 3);
        uint8_t bx = x + i * 5;
        uint8_t by = baseY - h;
        if (i < bars) display.fillRect(bx, by, 4, h, SSD1306_WHITE);
        else          display.drawRect(bx, by, 4, h, SSD1306_WHITE);
    }
}

static void draw_splash_screen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(4, 10);
    display.print(FW_NAME);
    display.setTextSize(1);
    display.setCursor(4, 34);
    display.print("Firmware v");
    display.print(FW_VERSION);
    display.setCursor(4, 48);
    display.print(RADIO_ENCRYPTION ? "Encrypted link" : "Open link");
    display.display();
}

static void draw_main_screen() {
    uint8_t ch = (uint8_t)gChannelIdx;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Top left: battery icon + percentage
    draw_battery_icon(0, 0, gBatPct);
    display.setCursor(22, 0);
    if (gBatWarning) display.print("!");
    display.print(gBatPct);
    display.print("%");

    // VOX enable indicator
    if (gVoxEnable.load()) {
        display.setCursor(70, 0);
        display.print("V");
    }

    // Top right: signal bars
    draw_signal_bars(108, 0, gSignalBars);

    // Center: channel number (large)
    char chBuf[8];
    snprintf(chBuf, sizeof(chBuf), "CH %d", ch + 1);
    uint8_t chLen = (uint8_t)strlen(chBuf);
    display.setTextSize(3);
    display.setCursor((128 - chLen * 18) / 2, 16);
    display.print(chBuf);

    // Below center: frequency
    char freqBuf[12];
    snprintf(freqBuf, sizeof(freqBuf), "%u MHz", CHANNEL_MHZ[ch]);
    uint8_t freqLen = (uint8_t)strlen(freqBuf);
    display.setTextSize(1);
    display.setCursor((128 - freqLen * 6) / 2, 44);
    display.print(freqBuf);

    // Bottom left: audio level meter
    draw_meter_bars(2, 64, gAudioBars);

    // Bottom right: status
    const char *status;
    if (gTotLatched.load())        status = "TIMEOUT";
    else if (gTransmitting.load()) status = "TX >>>";
    else                           status = "RX ...";
    display.setCursor(128 - (uint8_t)strlen(status) * 6, 56);
    display.print(status);

    display.display();
}

// Returns the current value string for a menu item (empty string = no value).
static void menu_item_value(uint8_t idx, char *buf, uint8_t bufLen) {
    switch (idx) {
        case MENU_IDX_CHANNEL:
            snprintf(buf, bufLen, "CH%d", (uint8_t)gChannelIdx + 1);
            break;
        case MENU_IDX_VOLUME:
            snprintf(buf, bufLen, "%d", (uint8_t)gVolume);
            break;
        case MENU_IDX_SQUELCH:
            snprintf(buf, bufLen, "%d", (uint16_t)gSquelch);
            break;
        case MENU_IDX_VOX:
            snprintf(buf, bufLen, "%s", gVoxEnable.load() ? "ON" : "OFF");
            break;
        default:
            buf[0] = '\0';
            break;
    }
}

static void draw_menu_screen() {
    display.clearDisplay();
    display.setTextSize(1);

    // Header
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 1);
    display.print("SETTINGS");
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

    const uint8_t VIS = 5;
    const uint8_t ROW = 10;    // px per row
    const uint8_t TOP = 11;    // y of first row
    uint8_t first = (gMenuSel >= VIS) ? (gMenuSel - VIS + 1) : 0;

    for (uint8_t r = 0; r < VIS && (first + r) < MENU_ITEM_COUNT; r++) {
        uint8_t i   = first + r;
        uint8_t y   = TOP + r * ROW;
        bool    sel = (i == gMenuSel);

        if (sel) {
            display.fillRect(0, y, 128, ROW, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
        } else {
            display.setTextColor(SSD1306_WHITE);
        }

        // Label
        display.setCursor(6, y + 1);
        display.print(MENU_ITEMS[i]);

        // Current value, right-aligned
        char val[8];
        menu_item_value(i, val, sizeof(val));
        if (val[0]) {
            display.setCursor(128 - (uint8_t)(strlen(val) * 6) - 4, y + 1);
            display.print(val);
        }
    }
    display.display();
}

static void draw_value_screen(const char *label, int32_t val, int32_t minv, int32_t maxv) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(label);
    display.setTextSize(3);
    display.setCursor(20, 20);
    display.print(val);
    int32_t range = maxv - minv;
    uint8_t barW  = (range > 0) ? (uint8_t)(((val - minv) * 120) / range) : 0;
    display.drawRect(4, 54, 120, 8, SSD1306_WHITE);
    if (barW > 0) display.fillRect(4, 54, barW, 8, SSD1306_WHITE);
    display.display();
}

static void draw_channel_screen() {
    uint8_t ch = (uint8_t)gChannelIdx;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Channel Select");
    display.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 18);
    display.print("CH ");
    display.print(ch + 1);
    display.setTextSize(1);
    display.setCursor(10, 38);
    display.print(CHANNEL_NAMES[ch]);
    display.setCursor(10, 50);
    display.print(CHANNEL_MHZ[ch]);
    display.print(" MHz");
    display.display();
}

static void draw_scan_screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Scanning...");
    display.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.print("CH ");
    display.print(gScanCh + 1);
    display.setTextSize(1);
    display.setCursor(10, 44);
    display.print(CHANNEL_MHZ[gScanCh]);
    display.print(" MHz");
    display.setCursor(0, 56);
    display.print("BACK = stop");
    display.display();
}

static void draw_soundtest_screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(16, 0);
    display.print("[ SOUND TEST ]");
    display.drawFastHLine(0, 9, 128, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(10, 16);
    display.print("1kHz Tone");
    display.setTextSize(1);
    display.setCursor(0, 44);
    display.print("VOL: ");
    display.print((uint8_t)gVolume);
    display.setCursor(60, 44);
    display.print("UP/DN=vol");
    display.setCursor(28, 56);
    display.print("BACK = stop");
    display.display();
}

static void update_display() {
    switch (gMenu) {
        case MENU_NONE:    draw_main_screen();  break;
        case MENU_MAIN:    draw_menu_screen();  break;
        case MENU_VOLUME:  draw_value_screen("Volume",  gVolume,  VOLUME_MIN,  VOLUME_MAX);  break;
        case MENU_SQUELCH: draw_value_screen("Squelch", gSquelch, SQUELCH_MIN, SQUELCH_MAX); break;
        case MENU_CHANNEL:   draw_channel_screen();   break;
        case MENU_SCAN:      draw_scan_screen();      break;
        case MENU_SOUNDTEST: draw_soundtest_screen(); break;
    }
}

// ---- Radio ----------------------------------------------------------------

static void radio_apply_channel(uint8_t idx) {
    xQueueReset(rxQueue);
    gPlaybackReady.store(false);
    gRxFirstPacket   = true;
    gLastDecodeState = {0, 0};
    gHpZ1 = gHpZ2 = gLpZ = 0;
    gDeempZ = gPreempZ = 0;
    gEqZ1 = gEqZ2 = 0;
    gLimEnv = 0;
    radio.stopListening();
    radio.setChannel(CHANNEL_LIST[idx]);
    if (!gTransmitting.load()) radio.startListening();
}

static void radio_init() {
    SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
    if (!radio.begin()) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("Radio FAIL");
        display.setCursor(0, 12);
        display.print("Check NRF24 wiring");
        display.setCursor(0, 24);
        display.print("& 3V3 power");
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

// caller must hold radioMutex. Performs the full TX<->RX transition + state flush.
static void radio_set_tx(bool tx) {
    if (tx) {
        gRxLastSeq       = 0xFF;
        gLastDecodeState = {0, 0};
        gHpZ1 = gHpZ2 = gLpZ = 0;
        gPreempZ = gEqZ1 = gEqZ2 = 0;
        gLimEnv = 0;
        gAgcGain       = 256;
        gAgcEnvelope   = 0;
        gNgOpen        = false;
        gNgHoldCounter = 0;
        gNgGain        = 0;
        gNgGainTarget  = 0;
        gTxPktCounter  = 0;
        gTxFirstFrame  = true;
        xQueueReset(txQueue);
        xQueueReset(rxQueue);
        xQueueReset(sidetoneQueue);
        radio.stopListening();
        gTxStartMs.store(millis());
    } else {
        // tell the receiver we stopped; send a few times since END may be lost
        Packet endPkt = {};
        endPkt.type  = PKT_END;
        endPkt.seq   = gTxSeq++;
        endPkt.nonce = gTxNonce.fetch_add(1);
        for (int i = 0; i < 3; i++) radio.write(&endPkt, NRF_PAYLOAD_SIZE);
        radio.txStandBy();
        xQueueReset(txQueue);
        xQueueReset(rxQueue);
        xQueueReset(sidetoneQueue);
        gRxFirstPacket = true;
        gPlaybackReady.store(false);
        gDeempZ = 0;
        radio.startListening();
    }
}

// Re-evaluate desired TX state from PTT/VOX/timeout and apply if it changed.
static void request_tx_state() {
    bool desired = (gPttHeld.load() || gVoxActive.load()) && !gTotLatched.load();
    if (desired == gTransmitting.load()) return;
    Serial.printf("[TX] %s  (ptt=%d vox=%d tot=%d)\n",
                  desired ? "ON" : "OFF",
                  gPttHeld.load(), gVoxActive.load(), gTotLatched.load());
    xSemaphoreTake(radioMutex, portMAX_DELAY);
    if (desired != gTransmitting.load()) {
        gTransmitting.store(desired);
        radio_set_tx(desired);
    }
    xSemaphoreGive(radioMutex);
    gDisplayDirty = true;
}

// ---- Buttons --------------------------------------------------------------

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
        else     b.held    = false;   // clear hold on release so next press doesn't auto-repeat
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

static void btn_clear_hold(uint8_t idx) { btnStates[idx].held = false; }

static void change_channel(int8_t delta) {
    int16_t ch = (int16_t)(uint8_t)gChannelIdx + delta;
    if (ch < 0 || ch >= CHANNEL_COUNT) return;
    gChannelIdx.store((uint8_t)ch);
    xSemaphoreTake(radioMutex, portMAX_DELAY);
    radio_apply_channel((uint8_t)ch);
    xSemaphoreGive(radioMutex);
    settings_mark_dirty();
    gDisplayDirty = true;
}

static void start_scan() {
    gMenu           = MENU_SCAN;
    gScanCh         = (uint8_t)gChannelIdx;
    gScanDwellStart = millis();
    gScanBaseRx     = gPktsReceived.load();
    gDisplayDirty   = true;
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

    if (upFell || downFell || backFell || selFell) gLastActivity = millis();

    if (gDisplayOff && (upFell || downFell || backFell || selFell)) {
        gDisplayOff    = false;
        gDisplayDimmed = false;
        display.ssd1306_command(0xAF);        // display on
        display.ssd1306_command(0x81);        // restore full contrast
        display.ssd1306_command(0xFF);
        gDisplayDirty = true;
        return;
    }
    if (gDisplayDimmed && (upFell || downFell || backFell || selFell)) {
        gDisplayDimmed = false;
        display.ssd1306_command(0x81);
        display.ssd1306_command(0xFF);
    }

    // Scan mode: only BACK is interactive (cancel). Stepping is in task_ui.
    if (gMenu == MENU_SCAN) {
        if (backFell) { gMenu = MENU_MAIN; gDisplayDirty = true; }
        return;
    }

    if (upAct) {
        switch (gMenu) {
            case MENU_NONE:
                change_channel(+1);
                break;
            case MENU_MAIN:
                if (gMenuSel > 0) { gMenuSel--; gDisplayDirty = true; }
                break;
            case MENU_VOLUME:
                if ((uint8_t)gVolume < VOLUME_MAX) { gVolume++; settings_mark_dirty(); gDisplayDirty = true; }
                break;
            case MENU_SQUELCH:
                if ((uint16_t)gSquelch < SQUELCH_MAX) {
                    uint16_t v = (uint16_t)gSquelch + SQUELCH_STEP;
                    gSquelch.store(v > SQUELCH_MAX ? SQUELCH_MAX : v);
                    settings_mark_dirty(); gDisplayDirty = true;
                }
                break;
            case MENU_CHANNEL:
                change_channel(+1);
                break;
            case MENU_SOUNDTEST:
                if ((uint8_t)gVolume < VOLUME_MAX) { gVolume++; settings_mark_dirty(); gDisplayDirty = true; }
                break;
            default: break;
        }
        btn_clear_hold(0);
    }

    if (downAct) {
        switch (gMenu) {
            case MENU_NONE:
                change_channel(-1);
                break;
            case MENU_MAIN:
                if (gMenuSel < MENU_ITEM_COUNT - 1) { gMenuSel++; gDisplayDirty = true; }
                break;
            case MENU_VOLUME:
                if ((uint8_t)gVolume > VOLUME_MIN) { gVolume--; settings_mark_dirty(); gDisplayDirty = true; }
                break;
            case MENU_SQUELCH:
                if ((uint16_t)gSquelch >= (uint16_t)(SQUELCH_MIN + SQUELCH_STEP)) {
                    gSquelch.store((uint16_t)gSquelch - SQUELCH_STEP);
                    settings_mark_dirty(); gDisplayDirty = true;
                }
                break;
            case MENU_CHANNEL:
                change_channel(-1);
                break;
            case MENU_SOUNDTEST:
                if ((uint8_t)gVolume > VOLUME_MIN) { gVolume--; settings_mark_dirty(); gDisplayDirty = true; }
                break;
            default: break;
        }
        btn_clear_hold(1);
    }

    if (backFell) {
        if (gMenu == MENU_NONE) {
            gMenu    = MENU_MAIN;   // BACK on home screen = open settings
            gMenuSel = 0;
        } else {
            if (gMenu == MENU_SOUNDTEST) gSoundTestActive.store(false);
            if (gMenu == MENU_MAIN) {
                gMenu    = MENU_NONE;
                gMenuSel = 0;       // reset cursor only when closing the whole menu
            } else {
                gMenu = MENU_MAIN;  // back to list, cursor stays on the item they came from
            }
        }
        gDisplayDirty = true;
    }

    if (selFell) {
        if (gMenu == MENU_NONE) {
            gMenu    = MENU_MAIN;
            gMenuSel = 0;
        } else if (gMenu == MENU_MAIN) {
            switch (gMenuSel) {
                case MENU_IDX_CHANNEL:  gMenu = MENU_CHANNEL;  break;
                case MENU_IDX_VOLUME:   gMenu = MENU_VOLUME;   break;
                case MENU_IDX_SQUELCH:  gMenu = MENU_SQUELCH;  break;
                case MENU_IDX_VOX: {
                    bool en = !gVoxEnable.load();
                    gVoxEnable.store(en);
                    if (!en && gVoxActive.load()) { gVoxActive.store(false); request_tx_state(); }
                    settings_mark_dirty();
                    break;
                }
                case MENU_IDX_SCAN:    start_scan(); break;
                case MENU_IDX_SOUNDTEST:
                    if (!gTransmitting.load()) {
                        gMenu = MENU_SOUNDTEST;
                        gSoundTestActive.store(true);
                    }
                    break;
            }
        }
        gDisplayDirty = true;
    }
}

static void scan_step() {
    uint32_t now = millis();
    if (gPktsReceived.load() != gScanBaseRx) {     // valid traffic on this channel -> stop
        settings_mark_dirty();                      // radio already on gChannelIdx; just persist it
        gMenu = MENU_NONE;
        gDisplayDirty = true;
        return;
    }
    if (now - gScanDwellStart >= SCAN_DWELL_MS) {
        gScanCh = (uint8_t)((gScanCh + 1) % CHANNEL_COUNT);
        gChannelIdx.store(gScanCh);
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        radio_apply_channel(gScanCh);
        xSemaphoreGive(radioMutex);
        gScanDwellStart = now;
        gScanBaseRx     = gPktsReceived.load();
        gDisplayDirty   = true;
    }
}

// ---- Tasks ----------------------------------------------------------------

static void task_capture(void *) {
    static int16_t    pcmBuf[MIC_CAPTURE_SAMPLES];
    static uint8_t    adpcmOut[MIC_CAPTURE_SAMPLES / 2 + 1];
    static AdpcmState encState = {0, 0};

    while (true) {
        bool tx      = gTransmitting.load();
        // Keep monitoring the mic during a TX-timeout latch so VOX can release
        // (clear gVoxActive) once the operator actually stops talking.
        bool monitor = gVoxEnable.load() && !gPttHeld.load();

        if (!tx && !monitor) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        size_t got = 0;
        if (i2s_read(I2S_NUM_1, pcmBuf, sizeof(pcmBuf), &got, pdMS_TO_TICKS(20)) != ESP_OK || got == 0)
            continue;

        uint16_t samples = (uint16_t)(got / sizeof(int16_t));
        remove_dc_offset(pcmBuf, samples);
        apply_highpass(pcmBuf, samples);

        // VOX: decide TX state from clean (pre-AGC) mic energy.
        // Open threshold adapts to the background level so a noisy supply (e.g. USB)
        // raises the bar instead of false-keying; voice must clear both an absolute
        // floor and a multiple of the recently-measured noise floor, for a few frames.
        if (gVoxEnable.load() && !gPttHeld.load()) {
            uint32_t e          = pcm_energy(pcmBuf, samples);
            uint32_t openThresh = gVoxNoiseFloor * VOX_NOISE_RATIO;
            if (openThresh < VOX_OPEN_ENERGY) openThresh = VOX_OPEN_ENERGY;

            if (e >= openThresh) {
                gLastVoiceMs = millis();
                if (gVoxAttack < VOX_ATTACK_FRAMES) gVoxAttack++;
                if (gVoxAttack >= VOX_ATTACK_FRAMES && !gVoxActive.load()) {
                    gVoxActive.store(true);
                    request_tx_state();
                }
            } else {
                gVoxAttack = 0;
                if (gVoxActive.load()) {
                    if ((millis() - gLastVoiceMs) >= VOX_HANG_MS) {
                        gVoxActive.store(false);
                        request_tx_state();
                    }
                } else {
                    // Idle and quiet: slowly track the ambient noise floor (EMA, 1/8 weight).
                    if (e > gVoxNoiseFloor) gVoxNoiseFloor += (e - gVoxNoiseFloor) >> 3;
                    else                    gVoxNoiseFloor -= (gVoxNoiseFloor - e) >> 3;
                }
            }
        }

        if (!gTransmitting.load()) continue;   // monitoring only, not actually transmitting

        apply_pregain(pcmBuf, samples, MIC_PREGAIN_SHIFT);
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

        publish_audio_peak(pcmBuf, samples);   // TX: mic level for the on-screen meter

        // sidetone: attenuated copy of mic so the operator hears themselves
        RxFrame sf = {};
        sf.count = samples;
        for (uint16_t i = 0; i < samples; i++)
            sf.pcm[i] = (int16_t)(pcmBuf[i] >> SIDETONE_SHIFT);
        queue_push(sidetoneQueue, sf);

        uint8_t  channel = (uint8_t)gChannelIdx;
        uint16_t offset  = 0;
        while (offset < samples) {
            uint16_t count = samples - offset;
            if (count > ADPCM_SAMPLES_PER_PKT) count = ADPCM_SAMPLES_PER_PKT;

            bool isKeyframe = ((gTxPktCounter % KEYFRAME_INTERVAL) == 0);
            if (isKeyframe) encState = {0, 0};      // reset predictor at keyframe boundary

            AdpcmState snapState = encState;
            adpcm_encode_block(&pcmBuf[offset], adpcmOut, count, encState);

            uint8_t bytes = (uint8_t)((count + 1) >> 1);
            if (bytes > MAX_DATA_LEN) bytes = MAX_DATA_LEN;

            Packet pkt = {};
            pkt.type      = isKeyframe ? PKT_KEYFRAME : PKT_AUDIO;
            pkt.seq       = gTxSeq++;
            pkt.nonce     = gTxNonce.fetch_add(1);
            pkt.adpcmPred = snapState.pred;
            pkt.adpcmIdx  = snapState.idx;
            pkt.dataLen   = bytes;
            memcpy(pkt.data, adpcmOut, bytes);
            pkt.appCrc    = crc8(pkt.data, bytes);      // CRC over plaintext
#if RADIO_ENCRYPTION
            crypto_xor(RADIO_KEY, pkt.nonce, channel, pkt.data, bytes);
#endif
            gTxPktCounter++;

            TxPacket tp;
            memcpy(tp.raw, &pkt, NRF_PAYLOAD_SIZE);
            queue_push(txQueue, tp);

            offset += count;
        }
    }
}

static void task_transmit(void *) {
    TxPacket tp;
    while (true) {
        if (!gTransmitting.load()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        // capture paces production at the mic rate, so just drain and send
        if (xQueueReceive(txQueue, &tp, pdMS_TO_TICKS(10)) == pdTRUE) {
            xSemaphoreTake(radioMutex, portMAX_DELAY);
            radio.write(tp.raw, NRF_PAYLOAD_SIZE);
            xSemaphoreGive(radioMutex);
        }
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
        gRssiLast.store(radio.testRPD() ? -60 : -90);   // nRF24 only gives a threshold, not true RSSI
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

#if RADIO_ENCRYPTION
        crypto_xor(RADIO_KEY, pkt.nonce, (uint8_t)gChannelIdx, pkt.data, pkt.dataLen);
#endif
        if (pkt.appCrc != crc8(pkt.data, pkt.dataLen)) {   // corrupt or wrong key
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
            uint8_t lost     = (uint8_t)(pkt.seq - expected);   // wraps correctly
            if (lost > 0 && lost < 10) {
                gPktsLost     += lost;
                gLostInWindow += lost;
                for (uint8_t l = 0; l < lost; l++) {
                    RxFrame conceal = {};
                    conceal.count     = ADPCM_SAMPLES_PER_PKT;
                    conceal.gap       = (l == 0);
                    conceal.concealed = true;
                    adpcm_conceal_block(conceal.pcm, conceal.count, gLastDecodeState, 3);
                    queue_push(rxQueue, conceal);
                }
                gap = true;
            }
        }

        gRxLastSeq = pkt.seq;
        gPktsInWindow++;
        gPktsReceived++;

        if (compute_energy(pkt.data, pkt.dataLen) >= (uint32_t)gSquelch) {
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

            queue_push(rxQueue, frame);

            if (!gPlaybackReady.load() && uxQueueMessagesWaiting(rxQueue) >= (UBaseType_t)gJbTarget)
                gPlaybackReady.store(true);
        }
    }
}

static void task_playback(void *) {
    static RxFrame  frame;
    static int16_t  toneBuf[128];
    static uint8_t  tonePhase = 0;

    while (true) {
        if (gTransmitting.load()) {
            RxFrame sf;
            if (xQueueReceive(sidetoneQueue, &sf, pdMS_TO_TICKS(5)) == pdTRUE) {
                uint8_t logVol = LOG_VOL_TABLE[(uint8_t)gVolume - VOLUME_MIN];
                if (logVol != 0) {
                    for (uint16_t i = 0; i < sf.count; i++) {
                        // sidetone already attenuated by SIDETONE_SHIFT; >> 8 keeps it quiet
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

        if (gSoundTestActive.load()) {
            uint8_t logVol = LOG_VOL_TABLE[(uint8_t)gVolume - VOLUME_MIN];
            for (uint16_t i = 0; i < 128; i++) {
                int32_t s = ((int32_t)TONE_1KHZ_TABLE[tonePhase] * logVol) >> 7;
                toneBuf[i] = (int16_t)((s > 32767) ? 32767 : (s < -32768) ? -32768 : s);
                tonePhase  = (uint8_t)((tonePhase + 1) & 7);
            }
            size_t written = 0;
            i2s_write(I2S_NUM_0, toneBuf, sizeof(toneBuf), &written, pdMS_TO_TICKS(30));
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

        if (frame.gap && !frame.concealed) apply_fade_in(frame.pcm, frame.count, 16);
        if (frame.concealed)               apply_fade_out(frame.pcm, frame.count, 8);

        publish_audio_peak(frame.pcm, frame.count);   // RX: received audio level for the meter

        uint8_t logVol = LOG_VOL_TABLE[(uint8_t)gVolume - VOLUME_MIN];
        if (logVol != 128) {   // 128 = unity for >> 7 scaling
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
    bool     pttArmed    = false;   // ignore PTT until the pin reads released once

    while (true) {
        uint32_t now = millis();

        // ---- PTT (physical) ----
        // Defensive: don't honor PTT until the pin reads released (HIGH) at least once,
        // so a button held during boot (or a pin held low externally) can't key TX.
        bool ptt = (digitalRead(BTN_PTT_PIN) == LOW);
        if (!pttArmed) {
            if (!ptt) pttArmed = true;
            ptt = false;
        }
        if (ptt != gPttHeld.load()) {
            gPttHeld.store(ptt);
            gLastActivity = now;
            if (ptt && gMenu == MENU_SCAN) gMenu = MENU_NONE;   // PTT cancels scan
            if (!ptt) gTotLatched.store(false);                 // releasing PTT clears timeout
            request_tx_state();
        }

        // ---- TX timeout (anti-stuck-PTT / channel hog) ----
        // Re-read millis() here instead of reusing `now` from the top of the loop:
        // gTxStartMs is stored later in this same iteration (inside request_tx_state())
        // so `now - gTxStartMs` can underflow (gTxStartMs > now) and wrap to a huge
        // value, latching TOT instantly on the first PTT press.
        if (TOT_MS > 0 && gTransmitting.load() && !gTotLatched.load()) {
            uint32_t txElapsed = millis() - gTxStartMs.load();
            if (txElapsed >= TOT_MS) {
                gTotLatched.store(true);
                Serial.printf("[TOT] timeout after %lu ms TX (ptt=%d vox=%d)\n",
                              (unsigned long)txElapsed,
                              gPttHeld.load(), gVoxActive.load());
                request_tx_state();           // forces RX
                gDisplayDirty = true;
            }
        }
        if (gTotLatched.load() && !gPttHeld.load() && !gVoxActive.load())
            gTotLatched.store(false);

        handle_menu_buttons();
        if (gMenu == MENU_SCAN) scan_step();

        update_signal_bars();
        update_jitter_buffer();
        update_audio_meter();

        if (now - lastBat >= BAT_READ_MS) {
            lastBat     = now;
            float newV  = read_battery_voltage();
            gBatVoltage = (gBatVoltage == 0.0f) ? newV : (0.9f * gBatVoltage + 0.1f * newV);
            gBatPct      = lipo_pct(gBatVoltage);
            gBatMv       = (uint32_t)(gBatVoltage * 1000.0f);
            gBatWarning  = (gBatMv < BAT_LOW_MV);
            gBatCritical = (gBatMv < BAT_CRITICAL_MV);
            gDisplayDirty = true;
        }

        uint32_t idleMs = now - gLastActivity;
        if (!gDisplayOff && !gDisplayDimmed && idleMs >= DISPLAY_DIM_MS) {
            gDisplayDimmed = true;
            display.ssd1306_command(0x81);
            display.ssd1306_command(0x10);
        }
        if (!gDisplayOff && gDisplayDimmed && idleMs >= DISPLAY_OFF_MS) {
            gDisplayOff    = true;
            gDisplayDimmed = false;
            display.ssd1306_command(0xAE);
        }

        if (gBatCritical) gDisplayDirty = true;   // keep the icon blinking

        if (gDisplayDirty.load() && !gDisplayOff && (now - lastDisplay >= DISPLAY_UPDATE_MS)) {
            lastDisplay   = now;
            gDisplayDirty = false;
            update_display();
        }

        settings_flush(now);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---- Setup / main loop ----------------------------------------------------

void setup() {
    Serial.begin(115200);
    Serial.printf("\n%s firmware v%s  (encryption=%d)\n", FW_NAME, FW_VERSION, RADIO_ENCRYPTION);

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adcChars);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    for (uint8_t i = 0; i < 5; i++) {
        pinMode(BTN_PINS[i], INPUT_PULLUP);
        btnStates[i] = {};
    }

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(400000);
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    draw_splash_screen();

    settings_load();
    radio_init();
    i2s_speaker_init();
    i2s_mic_init();

    gBatVoltage = read_battery_voltage();
    gBatPct     = lipo_pct(gBatVoltage);
    gBatMv      = (uint32_t)(gBatVoltage * 1000.0f);
    gBatWarning  = (gBatMv < BAT_LOW_MV);
    gBatCritical = (gBatMv < BAT_CRITICAL_MV);

    gSignalWindowStart = millis();
    gLastActivity      = millis();
    gJbLastAdjust      = millis();

    txQueue       = xQueueCreate(TX_RING_PKTS, sizeof(TxPacket));
    rxQueue       = xQueueCreate(RX_RING_PKTS, sizeof(RxFrame));
    sidetoneQueue = xQueueCreate(4,            sizeof(RxFrame));
    radioMutex    = xSemaphoreCreateMutex();

    configASSERT(txQueue);
    configASSERT(rxQueue);
    configASSERT(sidetoneQueue);
    configASSERT(radioMutex);

    // cap/tx/rx on core 1 (radio-bound), play/ui on core 0 (audio DMA-bound)
    xTaskCreatePinnedToCore(task_capture,  "cap",  4096, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(task_transmit, "tx",   3072, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(task_receive,  "rx",   4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(task_playback, "play", 3072, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(task_ui,       "ui",   4096, nullptr, 2, nullptr, 0);

    vTaskDelay(pdMS_TO_TICKS(800));   // let the splash linger briefly
    gDisplayDirty = true;
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
