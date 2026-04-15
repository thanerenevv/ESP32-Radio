# ESP32 DIY Walkie-Talkie

A DIY walkie-talkie built on the ESP32 microcontroller featuring real-time voice communication using NRF24L01 radio modules, ADPCM audio compression, and an OLED display interface.

## Features

- **Real-time Voice Communication**: Full-duplex capable voice transmission using ADPCM compression for efficient bandwidth usage
- **11 Channels**: Frequencies from 2400 MHz to 2500 MHz
- **OLED Display**: 128x64 SH1106 display showing channel, battery, signal strength, and settings
- **Menu System**: Adjust volume, squelch, and channel selection
- **Battery Monitoring**: Real-time LiPo battery voltage and percentage display
- **Signal Quality Indicator**: 4-bar signal strength based on packet loss
- **FreeRTOS Multitasking**: Dedicated tasks for capture, transmission, reception, playback, and UI

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| ESP32 Dev Board | Any ESP32 development board |
| NRF24L01+ | 2.4GHz radio module (PA/LNA version recommended for better range) |
| SH1106 OLED | 128x64 I2C OLED display |
| I2S Microphone | INMP441 or similar I2S MEMS microphone |
| I2S Amplifier | MAX98357A or similar I2S DAC/amp |
| Speaker | 4-8Ω speaker (match to amplifier) |
| LiPo Battery | 3.7V LiPo cell |
| Push Buttons | 5x momentary push buttons |

## Wiring Guide

### NRF24L01 Radio Module

| NRF24L01 Pin | ESP32 GPIO | Description |
|--------------|-----------|-------------|
| VCC | 3.3V | Power (3.3V only!) |
| GND | GND | Ground |
| CE | GPIO 4 | Chip Enable |
| CSN | GPIO 5 | SPI Chip Select |
| SCK | GPIO 18 | SPI Clock |
| MOSI | GPIO 23 | SPI Data In |
| MISO | GPIO 19 | SPI Data Out |

> **Note**: For better range, use an NRF24L01+PA/LNA module with external antenna.

### SH1106 OLED Display (I2C)

| OLED Pin | ESP32 GPIO | Description |
|----------|-----------|-------------|
| VCC | 3.3V | Power |
| GND | GND | Ground |
| SCL | GPIO 22 | I2C Clock |
| SDA | GPIO 21 | I2C Data |

I2C Address: `0x3C`

### I2S Speaker Amplifier (MAX98357A)

| Amplifier Pin | ESP32 GPIO | Description |
|---------------|-----------|-------------|
| VCC | 5V or 3.3V | Power (check module specs) |
| GND | GND | Ground |
| BCLK | GPIO 26 | Bit Clock |
| LRC | GPIO 25 | Word Select / LR Clock |
| DIN | GPIO 27 | Data Input |

### I2S Microphone (INMP441)

| Microphone Pin | ESP32 GPIO | Description |
|----------------|-----------|-------------|
| VDD | 3.3V | Power |
| GND | GND | Ground |
| SCK | GPIO 14 | Serial Clock |
| WS | GPIO 33 | Word Select |
| SD | GPIO 32 | Serial Data |
| L/R | GND | Left channel (or VDD for right) |

### Push Buttons

All buttons are active LOW (connect to GND when pressed).

| Button | ESP32 GPIO | Function |
|--------|-----------|----------|
| UP | GPIO 13 | Navigate up / Increase value |
| DOWN | GPIO 12 | Navigate down / Decrease value |
| BACK | GPIO 15 | Go back / Exit menu |
| SELECT | GPIO 2 | Enter menu / Confirm selection |
| PTT | GPIO 0 | Push-to-Talk (hold to transmit) |

### Battery Monitoring

Uses a voltage divider to measure LiPo battery voltage.

| Component | Connection |
|-----------|------------|
| ADC Input | GPIO 34 |
| R1 | 100kΩ (Battery+ to ADC) |
| R2 | 100kΩ (ADC to GND) |

```
Battery+ ──[R1 100kΩ]──┬──[R2 100kΩ]── GND
                       │
                    GPIO 34 (ADC)
```

This divider gives a 2:1 ratio, allowing measurement of up to ~6.6V with 3.3V ADC reference.

## Software Setup

### Prerequisites

- [PlatformIO](https://platformio.org/) installed
- ESP32 board support package

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd diyradioesp32
   ```

2. Build and flash:
   ```bash
   pio run -t upload
   ```

3. Monitor serial output (optional):
   ```bash
   pio device monitor
   ```

### Configuration

Edit `include/config.h` to customize:

- **Radio Settings**: Channel list, PA level, data rate
- **Audio Settings**: Sample rate (8000 Hz default), ADPCM parameters
- **Pin Assignments**: All GPIO pins can be remapped
- **Default Values**: Volume, squelch, default channel

## Usage

### Basic Operation

1. Power on the device
2. The display shows current channel, battery status, and RX mode
3. Hold the **PTT** button to transmit (TX indicator shows)
4. Release PTT to receive

### Menu Navigation

1. Press **SELECT** to enter the main menu
2. Use **UP/DOWN** to navigate between options:
   - **Volume**: Adjust speaker volume (1-16)
   - **Squelch**: Set signal threshold (0-2000)
   - **Channel**: Select radio channel (1-11)
   - **Back**: Exit menu
3. Press **SELECT** to enter a submenu
4. Use **UP/DOWN** to adjust values
5. Press **BACK** to return to previous menu

### Channel Frequencies

| Channel | Frequency (MHz) |
|---------|-----------------|
| 1 | 2400 |
| 2 | 2410 |
| 3 | 2420 |
| 4 | 2430 |
| 5 | 2440 |
| 6 | 2450 |
| 7 | 2460 |
| 8 | 2470 |
| 9 | 2480 |
| 10 | 2490 |
| 11 | 2500 |

## Technical Details

### Audio Processing

- **Sample Rate**: 8000 Hz (suitable for voice)
- **Compression**: ADPCM (Adaptive Differential Pulse Code Modulation)
- **Compression Ratio**: 4:1 (16-bit to 4-bit)
- **Effective Bitrate**: 32 kbps

### Radio Protocol

- **Data Rate**: 2 Mbps
- **Payload Size**: 32 bytes per packet
- **CRC**: 16-bit
- **Auto-Ack**: Enabled with retries

### Packet Structure

```
| Type (1B) | Seq (1B) | ADPCM Pred (2B) | ADPCM Idx (1B) | Len (1B) | Data (26B) |
```

### FreeRTOS Tasks

| Task | Priority | Core | Function |
|------|----------|------|----------|
| capture | 4 | 1 | Microphone capture & ADPCM encoding |
| transmit | 5 | 1 | Radio transmission |
| receive | 5 | 1 | Radio reception & decoding |
| playback | 4 | 1 | I2S audio playback |
| ui | 2 | 0 | Display, buttons, battery monitoring |

## Troubleshooting

### No Audio Output
- Check I2S connections (BCLK, WCLK, DATA)
- Verify speaker connection to amplifier
- Check volume setting (increase from menu)

### Cannot Transmit/Receive
- Verify NRF24L01 connections
- Ensure both units are on same channel
- Check power supply (NRF24L01 needs stable 3.3V)

### Display Not Working
- Verify I2C connections (SDA, SCL)
- Check I2C address (try 0x3C or 0x3D)
- Ensure display has proper power

### Poor Range
- Use NRF24L01+PA/LNA with external antenna
- Lower PA level if power supply is limited
- Check for 2.4GHz interference (WiFi, Bluetooth)

## Acknowledgments

- [RF24 Library](https://github.com/nRF24/RF24) - NRF24L01 driver
- [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110X) - OLED display driver
- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - Graphics library