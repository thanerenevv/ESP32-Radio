# ESP32 DIY Walkie-Talkie

A DIY walkie-talkie built on the ESP32 microcontroller featuring real-time, half-duplex (push-to-talk) voice communication using NRF24L01 radio modules, ADPCM audio compression, optional ChaCha20 link encryption, and an OLED display interface.

## Features

- **Real-time Voice**: Half-duplex push-to-talk voice with ADPCM compression (4:1) and a packet-loss-concealing jitter buffer
- **Encrypted Link** *(optional)*: ChaCha20 keystream encryption of the audio payload so casual listeners on the same channel can't eavesdrop (see [Security](#security))
- **11 Channels** with editable names: 2400–2500 MHz
- **Channel Scan**: hop the channel list and stop on the first active conversation
- **VOX** *(optional)*: hands-free voice-activated transmit (best with a headset)
- **TX Timeout**: automatically drops back to RX after 120 s of continuous transmit (anti-stuck-PTT / channel hog)
- **Persistent Settings**: volume, squelch, channel and VOX are saved to NVS and restored on boot
- **OLED Display**: 128x64 SH1106 with channel, battery, signal strength, scrolling menu, and a boot splash
- **Battery Monitoring**: LiPo voltage, percentage (discharge-curve based) and low/critical warnings
- **Signal Quality Indicator**: 4-bar strength from RSSI threshold + packet delivery rate
- **FreeRTOS Multitasking**: dedicated tasks for capture, transmit, receive, playback, and UI across both cores

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

> **Note**: For better range, use an NRF24L01+PA/LNA module with external antenna. or even better, an E01C-2G4M27D for just 1.5$-2$ more.

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

- **Encryption**: `RADIO_ENCRYPTION` (on/off) and `RADIO_KEY` (256-bit pre-shared key — **change it**)
- **Radio Settings**: Channel list, channel names, PA level, data rate, pipe address
- **Audio Settings**: Sample rate (8000 Hz default), ADPCM packet sizing
- **VOX**: `VOX_ENABLE_DEFAULT`, `VOX_OPEN_ENERGY`, `VOX_HANG_MS`
- **TX Timeout**: `TOT_MS` (set `0` to disable)
- **Pin Assignments**: All GPIO pins can be remapped (mind the strapping-pin note above)
- **Default Values**: Volume (1–30), squelch (0–2000), default channel

## Usage

### Basic Operation

1. Power on the device
2. The display shows current channel, battery status, and RX mode
3. Hold the **PTT** button to transmit (TX indicator shows)
4. Release PTT to receive

### Menu Navigation

1. Press **SELECT** to enter the main menu
2. Use **UP/DOWN** to navigate between options:
   - **Volume**: Adjust speaker volume (1-30)
   - **Squelch**: Set signal threshold (0-2000)
   - **Channel**: Select radio channel (1-11)
   - **Scan**: Sweep all channels and stop on the first active conversation (**BACK** to stop)
   - **VOX**: Toggle hands-free voice-activated transmit on/off
   - **Back**: Exit menu
3. Press **SELECT** to enter a submenu
4. Use **UP/DOWN** to adjust values (hold to repeat)
5. Press **BACK** to return to previous menu

All settings (volume, squelch, channel, VOX) are saved to NVS automatically a few
seconds after the last change and restored on the next power-up.

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
- **Payload Size**: 32 bytes per packet (fixed)
- **Link CRC**: 16-bit (nRF24 hardware)
- **Auto-Ack**: Disabled — audio is broadcast one-to-many with no per-packet retries; lost packets are concealed at the receiver
- **App CRC**: 8-bit over the (plaintext) audio payload, used for corruption / wrong-key detection

### Packet Structure (11-byte header + 21 data bytes)

```
| Type (1B) | Seq (1B) | Nonce (4B) | ADPCM Pred (2B) | ADPCM Idx (1B) | Len (1B) | CRC (1B) | Data (21B) |
```

- **Nonce**: per-packet monotonic counter (clear). Doubles as the ChaCha20 nonce, so every packet is independently decryptable — packet loss never desyncs the cipher. Persisted to NVS so reboots don't reuse keystream.
- **Data**: 21 ADPCM bytes = 42 samples/packet. Encrypted in place when `RADIO_ENCRYPTION = 1`.

### FreeRTOS Tasks

| Task | Priority | Core | Function |
|------|----------|------|----------|
| capture | 4 | 1 | Microphone capture, DSP, ADPCM encoding, VOX detection |
| transmit | 5 | 1 | Radio transmission |
| receive | 5 | 1 | Radio reception, decryption & decoding |
| playback | 4 | 0 | I2S audio playback (RX audio + TX sidetone) |
| ui | 2 | 0 | Display, buttons, battery, scan, settings persistence |

## Security

When `RADIO_ENCRYPTION` is `1` (default) in `include/config.h`, the audio payload of
every packet is encrypted with a **ChaCha20** keystream keyed by a 256-bit pre-shared
key (`RADIO_KEY`) and the per-packet nonce. This keeps the audio unintelligible to a
casual listener with another nRF24 on the same channel and address.

> **Change `RADIO_KEY` before shipping or relying on privacy.** The default key in the
> repo is public and provides no security. Every unit that needs to talk to each other
> must share the **same key** and the **same `RADIO_ENCRYPTION` setting**.

Scope and limits (be honest with your users):

- This is **confidentiality only**, not authenticated encryption. The 8-bit app CRC
  gives ~1/256 wrong-key/tamper rejection, not a real MAC — a determined attacker could
  still inject or replay packets.
- The nonce space is 32-bit and persisted across reboots to avoid keystream reuse.
- `adpcmPred`/`adpcmIdx` and headers are sent in clear (predictor state, not speech).

Set `RADIO_ENCRYPTION` to `0` for an open (unencrypted) link that interoperates with
plain builds.

## Hardware notes

- **Strapping pins**: `BTN_PTT` is on **GPIO0** and `BTN_SELECT` is on **GPIO2** — both
  are ESP32 boot-strapping pins. Holding **PTT** (or SELECT) while powering on can put
  the chip into download mode instead of booting. This cannot be fixed in firmware; if
  you respin the PCB, move PTT/SELECT to non-strapping GPIOs (e.g. 16/17).
- **NRF24L01 power**: the PA/LNA modules draw large current spikes. Use a solid 3.3 V
  supply with a 10–100 µF cap across the module's VCC/GND, or you'll see brown-out
  resets and corrupt packets. Lower `NRF_PA_LEVEL` if your rail is weak.

## Troubleshooting

### No Audio Output
- Check I2S connections (BCLK, WCLK, DATA)
- Verify speaker connection to amplifier
- Check volume setting (increase from menu)

### Cannot Transmit/Receive
- Verify NRF24L01 connections
- Ensure both units are on same channel
- Ensure both units have the **same `RADIO_KEY` and `RADIO_ENCRYPTION` setting** (a mismatch decodes as constant packet loss / silence)
- Check power supply (NRF24L01 needs stable 3.3V)

### Won't Boot / Stuck in Download Mode
- Don't hold **PTT** (GPIO0) while powering on — it's a strapping pin (see [Hardware notes](#hardware-notes))

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
