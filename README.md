# GPS Synchronized Clock

[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Hardware: CERN-OHL-W](https://img.shields.io/badge/hardware-CERN--OHL--W--v2-blue.svg)](LICENSE-hardware)
[![Platform: ATmega328P](https://img.shields.io/badge/platform-ATmega328P-orange.svg)](#hardware)

A GPS-disciplined digital clock for the Dasduino CORE (ATmega328P). A GPS PPS
signal keeps time, with a DS3231 RTC as backup. The 8×32 LED matrix shows
Croatia local time, temperature and humidity.

![Front side](docs/images/front_side.jpg)

## Features

- 8×32 LED matrix display, brightness adjustable in 15 steps
- Croatia local time with automatic CET/CEST conversion
- GPS PPS synchronization with DS3231 RTC fallback
- Temperature and relative humidity pages from the SHTC3 sensor
- Onboard red LED indicates PPS activity; the user button toggles serial diagnostics

## Repository layout

```
firmware/    Arduino sketch + custom MAX7219 library, PlatformIO config
hardware/    bill of materials
docs/        schematic, datasheets, pinout, photos, project report
```

## Hardware

Built on the **Dasduino CORE** (ATmega328P, Arduino-compatible). Full parts
list: [hardware/bom.csv](hardware/bom.csv).

| Qty | Part | Notes |
|-----|------|-------|
| 1 | Dasduino CORE (ATmega328P) | MCU board |
| 1 | NEO-6M | GPS module |
| 1 | DS3231 | RTC (battery-backed) |
| 1 | MAX7219 8×32 matrix | Display |
| 1 | Rotary encoder (Soldered) | Menu input |
| 1 | SHTC3 (Soldered) | Temperature sensor |
| 1 | Active GPS antenna + SMA→u.FL | — |
| 1 | Speaker | Alarm |
| 1 | 100 µF + 4× 10 µF + 4× 100 nF | Decoupling |

Datasheets: [DS3231](docs/datasheets/DS3231_datasheet.pdf) ·
[NEO-6M](docs/datasheets/NEO-6M_datasheet.pdf) ·
[MAX7219](docs/datasheets/MAX7219_datasheet.pdf) ·
[MAX7219 matrix](docs/datasheets/MAX7219_LED_matrix_datasheet.pdf) ·
[SHTC3](docs/datasheets/Soldered_SHTC3_datasheet.pdf) ·
[Dasduino CORE](docs/datasheets/Dasduino_CORE_datasheet.pdf)

### Dasduino CORE pinout

![Dasduino CORE pinout](docs/images/Dasduino_CORE_pinout.png)

### Board

| Perfboard (bottom) |
|--------------------|
| ![PCB bottom](docs/images/pcb_bottom_size.jpeg) |

### Schematic

![Schematic](docs/images/schematic.png)

## Firmware

Wire and SoftwareSerial ship with the Arduino AVR core. The firmware also uses
RTClib, TinyGPSPlus, MD_MAX72XX, Adafruit NeoPixel and Adafruit SHTC3. Build and
upload instructions: see [firmware/README.md](firmware/README.md). The
PlatformIO target uses the Arduino Nano / ATmega328P profile for the Dasduino
CORE.

## Usage

| Front | Back |
|-------|------|
| ![front](docs/images/front_side.jpg) | ![back](docs/images/back_side.jpg) |

Inside:

<img alt="inside" src="docs/images/internals.jpeg" width="100%" />

- The display cycles through Croatia local time, temperature and relative humidity.
- Press the onboard button to toggle serial diagnostics; diagnostic sends briefly sound the buzzer.

## Measurements

Maximum current consumption:

![Maximum current consumption](docs/images/maximum_current_consumption.jpeg)

## Troubleshooting

Prototype on the breadboard:

![Breadboard prototype](docs/images/troubleshooting.jpeg)

| Symptom | Fix |
|---------|-----|
| GPS won't communicate with the board | Swap TX/RX pins or change baud rate |
| PPS LED on the GPS doesn't blink | Check antenna connection; reposition the antenna |
| GPS time is a few seconds fast | Wait for the receiver to download ephemeris data |
| GPS receives no data | With a clear sky view, use a higher-gain antenna |
| GPS unreliable | Check supply; add 10 µF + 100 nF close to its power pins |
| RTC drifts, loses a second every few minutes | Check supply; add 10 µF + 100 nF close to its power pins |
| Matrix shows random characters | SPI wiring problem — check connections |
| Matrix causes interference | Add 10 µF + 100 nF to each matrix module's power pins |
| Matrix stuck glowing (won't exit TEST) | Check config in code and wiring |
| Dasduino CORE errors on upload | Use Arduino Nano + ATmega328P (Old Bootloader) |

## Creating fonts

Fonts were made with the [MD Parola Font Editor](https://pjrp.github.io/MDParolaFontEditor),
which supports variable-width characters. Background reading:
[Parola fonts A to Z](https://arduinoplusplus.wordpress.com/2016/11/08/parola-fonts-a-to-z-defining-fonts/).

## License

- **Firmware:** MIT — see [LICENSE](LICENSE)
- **Hardware** (schematic, layout): CERN-OHL-W v2 — see [LICENSE-hardware](LICENSE-hardware)
- **Docs & media** (photos, pinout, report): CC-BY-4.0 — see [docs/LICENSE](docs/LICENSE)
