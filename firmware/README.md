# Firmware

Arduino sketch for the GPS synchronized clock, plus a custom `MAX7219` library
for driving the 8×32 matrix.

```
gps_clock/
├── gps_clock.cpp   main program (commented; PlatformIO-ready)
├── MAX7219.h       custom matrix driver (header)
└── MAX7219.cpp     custom matrix driver (implementation)
```

The main program is a `.cpp` (not `.ino`) so it builds under PlatformIO without
relying on the Arduino IDE's automatic prototype generation — it carries its own
forward declarations. To open it in the Arduino IDE instead, copy it to
`gps_clock.ino`; the `#include <Arduino.h>` line is harmless there.

## Required libraries

Bundled with the Arduino AVR core (no install needed): **Wire**, **EEPROM**,
**SoftwareSerial**.

Install these:

- **RTClib** (Adafruit) — `adafruit/RTClib`
- **TinyGPSPlus** (Mikal Hart) — `mikalhart/TinyGPSPlus`
- **Rotary encoder (Soldered, easyC)** — Soldered's Arduino library
- **SHTC3 (Soldered)** — Soldered's Arduino library

The `MAX7219` library is included here in `gps_clock/` — nothing to install.

## Build & upload — Arduino IDE

1. Install the libraries above via Library Manager (the Soldered ones pull in
   their easyC dependency too).
2. Copy `gps_clock/gps_clock.cpp` to `gps_clock/gps_clock.ino` and open it.
3. Select **Arduino Nano**, processor **ATmega328P (Old Bootloader)**, correct port.
4. Upload.

## Build & upload — PlatformIO

`platformio.ini` targets the Nano (old bootloader) profile and pulls in all
dependencies automatically: RTClib and TinyGPSPlus from the registry, and the
Soldered rotary-encoder and SHTC3 libraries from their GitHub repositories. If a
build reports a missing easyC/generic header, add Soldered's generic library as
noted in `platformio.ini`.

```sh
cd firmware
pio run                 # build
pio run -t upload       # flash
```
