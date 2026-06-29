#pragma once
#include <Arduino.h>
#include <RF24.h>   // for RF24_PA_* / RF24_*MBPS enums used below

// ============================================================================
//  ESP32 Walkie-Talkie — build configuration
//  Target: ESP32 (classic, dual-core) on the esp32dev board, Arduino framework.
//  All over-the-air format constants must match on BOTH units. Reflash together.
// ============================================================================

#define FW_VERSION          "2.0.0"
#define FW_NAME             "ESP32 WT"

// ---- Pin map ---------------------------------------------------------------
// NOTE (hardware): BTN_BACK (GPIO2) is an ESP32 boot strapping pin. Holding BACK
// while powering on can change boot behaviour. PTT was moved off GPIO0 (BOOT
// button / USB auto-reset line) to GPIO16 so a tethered USB port can no longer
// hold it low and false-key TX. If you respin the PCB, also move BACK to a
// non-strapping GPIO. See README "Hardware notes".

#define NRF_CE_PIN          4
#define NRF_CSN_PIN         5
#define NRF_SCK_PIN         18
#define NRF_MOSI_PIN        23
#define NRF_MISO_PIN        19

#define OLED_SDA_PIN        21
#define OLED_SCL_PIN        22
#define OLED_ADDR           0x3C

#define I2S_BCLK_PIN        26
#define I2S_WCLK_PIN        25
#define I2S_DATA_PIN        27

#define I2S_MIC_SD_PIN      32
#define I2S_MIC_WS_PIN      33
#define I2S_MIC_SCK_PIN     14

#define BTN_UP_PIN          13
#define BTN_DOWN_PIN        12
#define BTN_BACK_PIN        2
#define BTN_SELECT_PIN      15
#define BTN_PTT_PIN         16

#define BAT_ADC_PIN         34
#define BAT_R1              100000.0f
#define BAT_R2              100000.0f
#define BAT_VREF            3.3f
#define BAT_ADC_MAX         4095.0f

// ---- Channels --------------------------------------------------------------
// CHANNEL_LIST holds the nRF24 RF channel register values (2400 + n MHz).
// CHANNEL_NAMES is optional free text shown in the UI (keep <= 10 chars).
static const uint8_t  CHANNEL_LIST[] = {0,10,20,30,40,50,60,70,80,90,100};
static const uint16_t CHANNEL_MHZ[]  = {2400,2410,2420,2430,2440,2450,2460,2470,2480,2490,2500};
static const char * const CHANNEL_NAMES[] = {
    "Calling","Ch 2","Ch 3","Ch 4","Ch 5","Ch 6","Ch 7","Ch 8","Ch 9","Ch 10","Emergency"
};
#define CHANNEL_COUNT       11
#define DEFAULT_CHANNEL_IDX 0

// ---- Radio link ------------------------------------------------------------
#define NRF_PAYLOAD_SIZE    32
#define NRF_PA_LEVEL        RF24_PA_MAX     // lower (e.g. RF24_PA_HIGH) if your 3V3 rail is weak
#define NRF_DATA_RATE       RF24_2MBPS
#define NRF_PIPE_ADDR       0xE8E8F0F0E1LL

// ---- Microphone gain -------------------------------------------------------
// The INMP441 outputs low amplitudes (~100-300 for normal speech at arm's
// length). This left-shift is applied before the DSP chain and AGC so the AGC
// operates on a well-conditioned signal. Each step doubles the level:
//   0 = no boost  1 = 2x  2 = 4x  3 = 8x  4 = 16x
// Start at 3 (8x). If the mic still clips your loud voice, reduce to 2.
#define MIC_PREGAIN_SHIFT   3

// ---- Audio / codec ---------------------------------------------------------
// Packet wire layout (see Packet struct in main.cpp), 11-byte header:
//   type(1) seq(1) nonce(4) adpcmPred(2) adpcmIdx(1) dataLen(1) appCrc(1)
// leaving NRF_PAYLOAD_SIZE-11 = 21 data bytes => 42 ADPCM samples per packet.
// MIC_CAPTURE_SAMPLES is a clean multiple of ADPCM_SAMPLES_PER_PKT so every
// transmitted packet is full (no tiny tail packets, no predictor churn).
#define SAMPLE_RATE             8000
#define ADPCM_BYTES_PER_PKT     21
#define ADPCM_SAMPLES_PER_PKT   42
#define MIC_CAPTURE_SAMPLES     126     // 3 * ADPCM_SAMPLES_PER_PKT

// ---- Over-the-air encryption ----------------------------------------------
// 1 = encrypt audio payload with ChaCha20 keystream (privacy from casual
// listeners on the same channel/address). 0 = clear audio.
// BOTH units must use the SAME key and the SAME setting. CHANGE THIS KEY before
// shipping — the default below is public and provides no security.
#define RADIO_ENCRYPTION    1
static const uint8_t RADIO_KEY[32] = {
    0x9e,0x37,0x79,0xb9,0x7f,0x4a,0x7c,0x15,
    0xf3,0x9c,0xc0,0x60,0x5c,0xed,0xc8,0x34,
    0x10,0x82,0x27,0x6b,0xf3,0xa2,0x72,0xd9,
    0xe3,0xb2,0x67,0x12,0x95,0x0f,0x9c,0x4a
};

// ---- Timing ----------------------------------------------------------------
#define DEBOUNCE_MS         50
#define BAT_READ_MS         8000
#define DISPLAY_UPDATE_MS   150
#define SIGNAL_WINDOW_MS    2000

// ---- Queues ----------------------------------------------------------------
#define RX_RING_PKTS        16
#define TX_RING_PKTS        16

// ---- Squelch ---------------------------------------------------------------
// Compared directly against compute_energy() (sum of squared ADPCM nibble
// magnitudes over the packet, range ~0..2058). Linear; higher = harder to open.
#define SQUELCH_DEFAULT     150
#define SQUELCH_MIN         0
#define SQUELCH_MAX         2000
#define SQUELCH_STEP        50

// ---- Volume ----------------------------------------------------------------
// 1..30 maps through LOG_VOL_TABLE (perceptual curve) in main.cpp.
#define VOLUME_DEFAULT      18
#define VOLUME_MIN          1
#define VOLUME_MAX          30

// ---- VOX (voice-activated transmit) ---------------------------------------
// Off by default. WARNING: with an open speaker this is feedback-prone; VOX is
// best with a headset. Energy is mean-square/256 of mic PCM (matches noise gate).
#define VOX_ENABLE_DEFAULT  false
#define VOX_OPEN_ENERGY     2500    // absolute mic-energy floor to start TX (mean-square/256)
#define VOX_NOISE_RATIO     4       // ...and must also exceed the tracked noise floor by this factor
#define VOX_ATTACK_FRAMES   3       // require this many consecutive loud frames (~47 ms) before keying
#define VOX_HANG_MS         800     // keep transmitting this long after voice stops

// ---- TX timeout (anti-stuck-PTT / channel hog) -----------------------------
#define TOT_MS              120000  // force RX after 120 s of continuous TX (0 = disabled)

// ---- Channel scan ----------------------------------------------------------
#define SCAN_DWELL_MS       150     // listen time per channel while scanning

// ---- LiPo discharge curve --------------------------------------------------
static const struct { float v; uint8_t pct; } LIPO_CURVE[] = {
    {4.20f,100},{4.15f,95},{4.11f,90},{4.05f,85},{4.00f,80},
    {3.95f,75}, {3.90f,70},{3.85f,65},{3.80f,60},{3.75f,55},
    {3.70f,50}, {3.65f,45},{3.60f,40},{3.55f,35},{3.50f,30},
    {3.45f,25}, {3.40f,20},{3.30f,15},{3.20f,10},{3.10f,5},{3.00f,0}
};
#define LIPO_CURVE_LEN 21
