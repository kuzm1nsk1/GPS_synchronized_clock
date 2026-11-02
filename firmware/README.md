# Firmware

PlatformIO firmware for the Dasduino CORE (ATmega328P) GPS synchronized clock.
The GPS receiver supplies UTC over serial and a 1 Hz PPS edge; the DS3231 RTC
keeps the display running when GPS time is unavailable. The 8×32 MAX7219 matrix
cycles through Croatia local time, temperature and relative humidity.

## Hardware connections

- D2: DS3231 1 Hz square wave
- D3: GPS PPS
- D6: GPS TX to MCU RX
- D7: onboard user button (toggles serial diagnostics)
- D8: onboard NeoPixel status LED
- D9: buzzer (brief tone for debug telemetry)
- D10: MAX7219 LOAD/CS; D11/D13: SPI DIN/CLK
- A4/A5: I2C SDA/SCL for DS3231 and SHTC3

## Build and upload

Install [PlatformIO](https://platformio.org/) and run these commands from this
directory:

```sh
pio run
pio run -t upload
```

The project uses the Arduino AVR platform and declares its libraries in
`platformio.ini`. Select the connected upload port in PlatformIO if automatic
port detection does not find it.
