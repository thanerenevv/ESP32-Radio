#pragma once
#include <Arduino.h>

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
#define BTN_BACK_PIN        15
#define BTN_SELECT_PIN      2
#define BTN_PTT_PIN         0

#define BAT_ADC_PIN         34
#define BAT_R1              100000.0f
#define BAT_R2              100000.0f
#define BAT_VREF            3.3f
#define BAT_ADC_MAX         4095.0f

static const uint8_t  CHANNEL_LIST[] = {0,10,20,30,40,50,60,70,80,90,100};
static const uint16_t CHANNEL_MHZ[]  = {2400,2410,2420,2430,2440,2450,2460,2470,2480,2490,2500};
#define CHANNEL_COUNT       11
#define DEFAULT_CHANNEL_IDX 0

#define NRF_PAYLOAD_SIZE    32
#define NRF_PA_LEVEL        RF24_PA_MAX
#define NRF_DATA_RATE       RF24_2MBPS
#define NRF_PIPE_ADDR       0xE8E8F0F0E1LL

#define SAMPLE_RATE             8000
#define MIC_CAPTURE_SAMPLES     128
#define ADPCM_SAMPLES_PER_PKT   56
#define ADPCM_BYTES_PER_PKT     28

#define PKT_AUDIO           0x01
#define PKT_END             0x02

#define DEBOUNCE_MS         50
#define BAT_READ_MS         8000
#define DISPLAY_UPDATE_MS   150
#define SIGNAL_WINDOW_MS    2000

#define RX_RING_PKTS        16
#define TX_RING_PKTS        16

#define SQUELCH_DEFAULT     400
#define SQUELCH_MIN         0
#define SQUELCH_MAX         2000
#define SQUELCH_STEP        100

#define VOLUME_DEFAULT      8
#define VOLUME_MIN          1
#define VOLUME_MAX          16

static const struct { float v; uint8_t pct; } LIPO_CURVE[] = {
    {4.20f,100},{4.15f,95},{4.11f,90},{4.05f,85},{4.00f,80},
    {3.95f,75}, {3.90f,70},{3.85f,65},{3.80f,60},{3.75f,55},
    {3.70f,50}, {3.65f,45},{3.60f,40},{3.55f,35},{3.50f,30},
    {3.45f,25}, {3.40f,20},{3.30f,15},{3.20f,10},{3.10f,5},{3.00f,0}
};
#define LIPO_CURVE_LEN 21