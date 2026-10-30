/**
 * @file main.cpp
 * @brief PPS-synchronized GPS clock firmware for an ATmega328P Arduino.
 *
 * @details
 * The GPS receiver supplies UTC over SoftwareSerial and a 1 Hz PPS edge on
 * D3. The PPS interrupt only captures a timestamp and sets a flag. The main
 * loop services that flag before routine work, updates the MAX7219 display,
 * and then synchronizes the DS3231 backup clock as needed. When GPS time is
 * unavailable, the DS3231 1 Hz square wave drives the display instead.
 *
 * The display shows Croatia local time with automatic CET/CEST conversion,
 * then temperature and relative humidity. The onboard red LED is steady when
 * no recent PPS is present and briefly goes dark for each fresh PPS. The user
 * button toggles serial diagnostics; the buzzer sounds only for debug sends.
 *
 * Hardware map: D2=DS3231 SQW, D3=GPS PPS, D6=GPS TX to MCU RX,
 * D7=onboard button (SoftwareSerial TX is unused), D8=onboard NeoPixel,
 * D9=buzzer, D10=MAX7219 LOAD/CS, D11/D13=SPI DIN/CLK, A4/A5=I2C SDA/SCL.
 * The DS3231 and SHTC3 share the I2C bus.
 *
 * Timing contract: interrupt handlers must remain short; clock-edge service
 * must run before GPS parsing, sensor reads, or diagnostics; periodic telemetry
 * is emitted only after the corresponding display refresh.
 *
 * @note This file intentionally keeps the small firmware in one translation
 * unit to make the AVR timing path and shared state easy to audit.
 */
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SHTC3.h>
#include <MD_MAX72xx.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>

/** @name Hardware and timing configuration
 *  @{ */
constexpr uint8_t MAX7219_LOAD_PIN = 10;  // LOAD / CS; D11 DIN and D13 CLK use SPI
constexpr uint8_t MODULE_COUNT = 4;
constexpr uint8_t DISPLAY_WIDTH = MODULE_COUNT * 8;
constexpr uint8_t TIME_TEXT_WIDTH = 31;
constexpr uint8_t MAX7219_INTENSITY = 3;  // MAX7219 range: 0 (dim) to 15 (bright)
constexpr uint8_t BOARD_RGB_LED_PIN = 8;  // Dasduino CORE onboard WS2812B
constexpr uint8_t BOARD_LED_RED_LEVEL = 255;
constexpr uint16_t GPS_PPS_HIGH_MS = 100; // NEO-6M default pulse width
constexpr uint8_t RTC_SQW_PIN = 2;  // DS3231 1 Hz square wave -> INT0
constexpr uint8_t GPS_PPS_PIN = 3;  // GPS PPS -> INT1, rising edge
constexpr uint8_t GPS_RX_PIN = 6;   // GPS TX -> MCU RX
constexpr uint8_t GPS_TX_PIN = 7;   // Also Dasduino CORE user button; GPS TX is unused
constexpr uint8_t USER_BUTTON_PIN = 7;
constexpr uint8_t DEBUG_BUZZER_PIN = 9;
constexpr uint32_t SENSOR_READ_INTERVAL_MS = 5000;
constexpr uint16_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint8_t GPS_RX_BATCH_SIZE = 12;
constexpr uint8_t DISPLAY_CYCLE_SECONDS = 19;
constexpr uint8_t CLOCK_PAGE_SECONDS = 15;
constexpr uint8_t TEMPERATURE_PAGE_SECONDS = 2;
/** @} */

/** @name Device drivers
 *  @{ */
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
RTC_DS3231 rtc;
Adafruit_SHTC3 shtc3;
MD_MAX72XX display(MD_MAX72XX::FC16_HW, MAX7219_LOAD_PIN, MODULE_COUNT);
Adafruit_NeoPixel boardLed(1, BOARD_RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
/** @} */

/** @name Interrupt-shared clock state
 *  @brief Values written by ISRs are copied atomically in the main loop.
 *  @{ */
volatile bool gpsPpsPending = false;
volatile bool rtcTickPending = false;
volatile bool haveSeenGpsPps = false;
volatile uint32_t lastGpsPpsMillis = 0;
volatile uint32_t lastRtcTickMillis = 0;
/** @} */

/** @name Clock and display state
 *  @{ */
bool rtcAvailable = false;
bool rtcTimeValid = false;
bool usingGps = false;
uint32_t lastRtcSyncEpoch = 0;
uint8_t shownHour = 0;
uint8_t shownMinute = 0;
uint8_t shownSecond = 0;
uint32_t lastClockTickMillis = 0;
bool haveShownTime = false;
bool colonsVisible = false;
enum DisplayPage : uint8_t { CLOCK_PAGE, TEMPERATURE_PAGE, HUMIDITY_PAGE };
DisplayPage displayPage = CLOCK_PAGE;
/** @} */

/** @name GPS status and indicator state
 *  @{ */
bool gpsHasFix = false;
bool ppsLedOffIntervalActive = false;
uint32_t ppsLedOffStartMillis = 0;
bool boardLedIsOn = false;
/** @} */

/** @name Environmental sensor state
 *  @{ */
bool shtc3Available = false;
bool sensorReadingValid = false;
float roomTemperatureC = 0.0f;
float roomHumidityPercent = 0.0f;
uint32_t lastSensorReadMillis = 0;
/** @} */

/** @name Debug button state
 *  @{ */
bool debugMode = false;
bool buttonLastReading = true;
bool buttonStableState = true;
uint32_t buttonLastChangeMillis = 0;
/** @} */

/** @name GSV C/N0 parser state
 *  @{ */
char gsvSentence[96];
uint8_t gsvSentenceLength = 0;
uint16_t gsvSignalSum = 0;
uint8_t gsvSignalCount = 0;
uint8_t averageCn0 = 0;
bool cn0Valid = false;
uint32_t lastCn0UpdateMillis = 0;
/** @} */

// Seven rows per digit. Each 3-bit row is drawn left to right, most-significant bit first.
const uint8_t DIGIT_FONT[10][7] PROGMEM = {
    {0b111, 0b101, 0b101, 0b101, 0b101, 0b101, 0b111},  // 0
    {0b010, 0b110, 0b010, 0b010, 0b010, 0b010, 0b111},  // 1
    {0b111, 0b001, 0b001, 0b111, 0b100, 0b100, 0b111},  // 2
    {0b111, 0b001, 0b001, 0b111, 0b001, 0b001, 0b111},  // 3
    {0b101, 0b101, 0b101, 0b111, 0b001, 0b001, 0b001},  // 4
    {0b111, 0b100, 0b100, 0b111, 0b001, 0b001, 0b111},  // 5
    {0b111, 0b100, 0b100, 0b111, 0b101, 0b101, 0b111},  // 6
    {0b111, 0b001, 0b001, 0b010, 0b010, 0b010, 0b010},  // 7
    {0b111, 0b101, 0b101, 0b111, 0b101, 0b101, 0b111},  // 8
    {0b111, 0b101, 0b101, 0b111, 0b001, 0b001, 0b111},  // 9
};

/**
 * @brief Capture a GPS PPS edge for main-loop processing.
 * @details ISR callback; it records only the edge time and a pending flag.
 *          Do not add serial, I2C, display, or allocation work here.
 * @note No parameters. Invoked by the D3 rising-edge interrupt.
 * @return Nothing.
 * @throws None.
 */
void onGpsPps() {
  lastGpsPpsMillis = millis();
  haveSeenGpsPps = true;
  gpsPpsPending = true;
}

/**
 * @brief Capture a DS3231 1 Hz square-wave edge for fallback timekeeping.
 * @details ISR callback. The main loop performs the RTC read after observing
 *          the pending flag.
 * @note No parameters. Invoked by the D2 falling-edge interrupt.
 * @return Nothing.
 * @throws None.
 */
void onRtcSquareWave() {
  lastRtcTickMillis = millis();
  rtcTickPending = true;
}

/**
 * @brief Set the onboard red indicator, avoiding redundant NeoPixel transfers.
 * @param on `true` to illuminate the LED; `false` to turn it off.
 * @return Nothing.
 * @throws None.
 */
void setBoardLed(bool on) {
  if (boardLedIsOn == on) return;
  boardLedIsOn = on;
  boardLed.setPixelColor(0, on ? boardLed.Color(BOARD_LED_RED_LEVEL, 0, 0) : 0);
  boardLed.show();
}

/**
 * @brief Refresh GPS position-fix diagnostics and show steady LED without PPS.
 * @details PPS, rather than position validity, controls the LED blink because
 *          PPS is the clock synchronization signal. Position validity remains
 *          available separately for diagnostics.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void updateGpsStatus() {
  const bool hasFix = gps.location.isValid() && gps.location.age() <= 2500;
  gpsHasFix = hasFix;

  // Match the GPS module indicator: steady red without fresh PPS, with the
  // observed inverse phase while PPS is present. Position-fix status is only
  // diagnostic; PPS controls both time and this indicator.
  noInterrupts();
  const bool ppsSeen = haveSeenGpsPps;
  const uint32_t ppsMillis = lastGpsPpsMillis;
  interrupts();
  const bool ppsFresh = ppsSeen && (uint32_t)(millis() - ppsMillis) <= 1500;
  if (!ppsFresh) {
    ppsLedOffIntervalActive = false;
    setBoardLed(true);
  }
}

/**
 * @brief Convert a left-to-right display x coordinate to MD_MAX72XX indexing.
 * @param leftToRightX Horizontal pixel coordinate in `[0, DISPLAY_WIDTH)`.
 * @return The corresponding right-to-left driver column.
 * @throws None.
 */
uint16_t displayColumn(uint8_t leftToRightX) {
  return DISPLAY_WIDTH - 1 - leftToRightX;
}

/**
 * @brief Render a compact time string centered on the four-module display.
 * @param timeText Null-terminated sequence of decimal digits and colons.
 * @param colonsOn Whether the two-pixel colon marks should be illuminated.
 * @return Nothing.
 * @throws None.
 */
void drawTime(const char *timeText, bool colonsOn) {
  // Wider two-column gaps flank colons; other characters have one-column gaps.
  uint8_t x = (DISPLAY_WIDTH - TIME_TEXT_WIDTH) / 2;

  display.update(MD_MAX72XX::OFF);
  display.clear();

  for (uint8_t i = 0; timeText[i] != '\0'; ++i) {
    const char character = timeText[i];
    if (character == ':') {
      if (colonsOn) {
        display.setPoint(2, displayColumn(x), true);
        display.setPoint(4, displayColumn(x), true);
      }
      ++x;
    } else if (character >= '0' && character <= '9') {
      const uint8_t digit = character - '0';
      for (uint8_t row = 0; row < 7; ++row) {
        const uint8_t pixels = pgm_read_byte(&DIGIT_FONT[digit][row]);
        for (uint8_t column = 0; column < 3; ++column) {
          const bool on = pixels & (1 << (2 - column));
          display.setPoint(row, displayColumn(x + column), on);
        }
      }
      x += 3;
    }

    if (timeText[i + 1] != '\0') {
      // Give each colon an extra blank column on both sides.
      x += (character == ':' || timeText[i + 1] == ':') ? 2 : 1;
    }
  }

  display.update();
}

/**
 * @brief Format and render an RTC/GPS time as `HH:MM:SS`.
 * @param time DateTime value whose hour, minute, and second are displayed.
 * @param colonsOn Whether the colon pixels should be illuminated.
 * @return Nothing.
 * @throws None.
 */
void drawDateTime(const DateTime &time, bool colonsOn) {
  char text[9];
  text[0] = '0' + time.hour() / 10;
  text[1] = '0' + time.hour() % 10;
  text[2] = ':';
  text[3] = '0' + time.minute() / 10;
  text[4] = '0' + time.minute() % 10;
  text[5] = ':';
  text[6] = '0' + time.second() / 10;
  text[7] = '0' + time.second() % 10;
  text[8] = '\0';
  drawTime(text, colonsOn);
}

/**
 * @brief Read one row of a compact display glyph.
 * @param character Digit or supported label character (`C`, `T`, `R`, `H`,
 *                  `r`, `%`, or `-`).
 * @param row Glyph row index, from 0 through 6.
 * @return Bit mask for pixels in that row; zero for unsupported characters.
 * @throws None.
 */
uint8_t glyphRows(char character, uint8_t row) {
  if (character >= '0' && character <= '9') {
    return pgm_read_byte(&DIGIT_FONT[character - '0'][row]);
  }

  // Compact 3x7 glyphs for the environmental readouts.
  static const uint8_t cGlyph[7] PROGMEM = {0b111, 0b100, 0b100, 0b100,
                                            0b100, 0b100, 0b111};
  static const uint8_t tGlyph[7] PROGMEM = {0b111, 0b010, 0b010, 0b010,
                                            0b010, 0b010, 0b010};
  static const uint8_t rGlyph[7] PROGMEM = {0b110, 0b101, 0b100, 0b100,
                                            0b100, 0b100, 0b100};
  static const uint8_t upperHGlyph[7] PROGMEM = {0b101, 0b101, 0b101, 0b111,
                                                 0b101, 0b101, 0b101};
  static const uint8_t upperRGlyph[7] PROGMEM = {0b110, 0b101, 0b101, 0b110,
                                                 0b101, 0b101, 0b101};
  static const uint8_t percentGlyph[7] PROGMEM = {0b11001, 0b11010, 0b00010,
                                                  0b00100, 0b01000, 0b01011,
                                                  0b10011};
  static const uint8_t minusGlyph[7] PROGMEM = {0b000, 0b000, 0b000, 0b111,
                                                0b000, 0b000, 0b000};
  if (character == 'C') return pgm_read_byte(&cGlyph[row]);
  if (character == 'T') return pgm_read_byte(&tGlyph[row]);
  if (character == 'R') return pgm_read_byte(&upperRGlyph[row]);
  if (character == 'H') return pgm_read_byte(&upperHGlyph[row]);
  if (character == 'r') return pgm_read_byte(&rGlyph[row]);
  if (character == '%') return pgm_read_byte(&percentGlyph[row]);
  if (character == '-') return pgm_read_byte(&minusGlyph[row]);
  return 0;
}

/**
 * @brief Return the pixel width used for a readout glyph.
 * @param character Glyph whose width is requested.
 * @return Five pixels for `%`, one for punctuation, two for space, otherwise
 *         three pixels.
 * @throws None.
 */
uint8_t glyphWidth(char character) {
  if (character == '%') return 5;
  if (character == '.' || character == ':') return 1;
  if (character == ' ') return 2;
  return 3;
}

/**
 * @brief Determine horizontal spacing between adjacent readout glyphs.
 * @param current Character at the current glyph position.
 * @param next Following character in the readout.
 * @return Number of blank columns inserted between the glyphs.
 * @throws None.
 */
uint8_t glyphGap(char current, char next) {
  if (current == '.' || next == '.') return 1;
  if (current == ':' || next == ':' || current == ' ') return 0;
  if (next == ' ') return 0;
  return 1;
}

/**
 * @brief Center and draw a compact environmental text readout.
 * @param text Null-terminated text composed of supported compact glyphs.
 * @return Nothing.
 * @throws None.
 */
void drawReadout(const char *text) {
  uint16_t totalWidth = 0;
  for (uint8_t i = 0; text[i] != '\0'; ++i) {
    totalWidth += glyphWidth(text[i]);
    if (totalWidth > DISPLAY_WIDTH) return;
    if (text[i + 1] != '\0') totalWidth += glyphGap(text[i], text[i + 1]);
    if (totalWidth > DISPLAY_WIDTH) return;
  }

  // Width is now bounded by DISPLAY_WIDTH, so centering cannot underflow.
  uint8_t x = (DISPLAY_WIDTH - totalWidth) / 2;
  display.update(MD_MAX72XX::OFF);
  display.clear();
  for (uint8_t i = 0; text[i] != '\0'; ++i) {
    const char character = text[i];
    if (character == '.') {
      display.setPoint(6, displayColumn(x), true);
    } else if (character == ':') {
      display.setPoint(2, displayColumn(x), true);
      display.setPoint(4, displayColumn(x), true);
    } else {
      const uint8_t width = glyphWidth(character);
      for (uint8_t row = 0; row < 7; ++row) {
        const uint8_t pixels = glyphRows(character, row);
        for (uint8_t column = 0; column < width; ++column) {
          const bool pixelOn = character != ' ' &&
                               (pixels & (1 << (width - 1 - column)));
          display.setPoint(row, displayColumn(x + column), pixelOn);
        }
      }
    }
    x += glyphWidth(character);
    if (text[i + 1] != '\0') {
      x += glyphGap(character, text[i + 1]);
    }
  }
  display.update();
}

/**
 * @brief Render the latest SHTC3 temperature in Celsius.
 * @details Values are rounded to tenths before converting to text. A missing
 *          sensor reading is represented by two dashes.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void drawTemperature() {
  if (!sensorReadingValid || !isfinite(roomTemperatureC)) {
    drawReadout("--");
    return;
  }

  const int16_t temperatureTenths = (int16_t)lroundf(roomTemperatureC * 10.0f);
  const int16_t absoluteTenths = abs(temperatureTenths);
  char value[16];
  snprintf(value, sizeof(value), "T : %s%d.%d C",
           temperatureTenths < 0 ? "-" : "",
           absoluteTenths / 10, absoluteTenths % 10);
  drawReadout(value);
}

/**
 * @brief Render the latest SHTC3 relative humidity.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void drawHumidity() {
  if (!sensorReadingValid || !isfinite(roomHumidityPercent)) {
    drawReadout("--");
    return;
  }

  char value[16];
  snprintf(value, sizeof(value), "%d %% RH", (int)lroundf(roomHumidityPercent));
  drawReadout(value);
}

/**
 * @brief Draw the page selected by `displayPage` using the latest state.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void showCurrentPage() {
  if (displayPage == CLOCK_PAGE) {
    if (haveShownTime) {
      const DateTime shownTime(2000, 1, 1, shownHour, shownMinute, shownSecond);
      drawDateTime(shownTime, colonsVisible);
    }
  } else if (displayPage == TEMPERATURE_PAGE) {
    drawTemperature();
  } else {
    drawHumidity();
  }
}

/**
 * @brief Emit one compact serial diagnostics block and play the debug chime.
 * @details Called on debug entry and after a PPS/RTC-driven display update.
 *          C/N0 is a mean of valid per-satellite GSV signal values; HDOP is a
 *          geometry indicator, not a time-accuracy measurement.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void printDebugTelemetry() {
  // A brief C5 note is played only when debug telemetry is sent.
  tone(DEBUG_BUZZER_PIN, 523, 20);

  noInterrupts();
  const bool ppsSeen = haveSeenGpsPps;
  const uint32_t ppsMillis = lastGpsPpsMillis;
  interrupts();
  const uint32_t now = millis();
  const uint32_t ppsAge = ppsSeen ? now - ppsMillis : UINT32_MAX;

  Serial.println();
  Serial.println(F("--- CLOCK DEBUG ---"));
  Serial.print(F("Local "));
  if (haveShownTime) {
    const uint32_t elapsedSeconds = (now - lastClockTickMillis) / 1000;
    const DateTime currentLocalTime(2000, 1, 1, shownHour, shownMinute, shownSecond);
    const DateTime liveLocalTime = currentLocalTime + TimeSpan(0, 0, 0, elapsedSeconds);
    if (liveLocalTime.hour() < 10) Serial.print('0');
    Serial.print(liveLocalTime.hour());
    Serial.print(':');
    if (liveLocalTime.minute() < 10) Serial.print('0');
    Serial.print(liveLocalTime.minute());
    Serial.print(':');
    if (liveLocalTime.second() < 10) Serial.print('0');
    Serial.print(liveLocalTime.second());
  } else {
    Serial.print(F("unavailable"));
  }
  Serial.print(F("  Src "));
  if (usingGps) Serial.println(F("GPS"));
  else if (rtcTimeValid) Serial.println(F("RTC"));
  else Serial.println(F("none"));

  Serial.print(F("UTC "));
  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.print(gps.date.year());
    Serial.print('-');
    if (gps.date.month() < 10) Serial.print('0');
    Serial.print(gps.date.month());
    Serial.print('-');
    if (gps.date.day() < 10) Serial.print('0');
    Serial.print(gps.date.day());
    Serial.print(' ');
    if (gps.time.hour() < 10) Serial.print('0');
    Serial.print(gps.time.hour());
    Serial.print(':');
    if (gps.time.minute() < 10) Serial.print('0');
    Serial.print(gps.time.minute());
    Serial.print(':');
    if (gps.time.second() < 10) Serial.print('0');
    Serial.print(gps.time.second());
    Serial.print(F(" age "));
    Serial.print(gps.time.age());
    Serial.println(F("ms"));
  } else {
    Serial.println(F("unavailable"));
  }

  Serial.print(F("GPS fix "));
  Serial.print(gpsHasFix ? F("yes") : F("no"));
  Serial.print(F("  sats "));
  if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
  else Serial.print('-');
  Serial.print(F("  loc age "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.age());
    Serial.print(F("ms"));
  } else {
    Serial.print(F("n/a"));
  }
  Serial.print(F("  HDOP "));
  if (gps.hdop.isValid()) {
    const float hdopValue = gps.hdop.hdop();
    Serial.print(hdopValue, 1);
    Serial.print(' ');
    if (hdopValue < 1.0f) Serial.print(F("excellent"));
    else if (hdopValue < 2.0f) Serial.print(F("good"));
    else if (hdopValue < 5.0f) Serial.print(F("moderate"));
    else Serial.print(F("weak"));
  } else {
    Serial.print(F("n/a"));
  }
  Serial.print(F("  C/N0 "));
  if (cn0Valid && now - lastCn0UpdateMillis <= 5000) {
    Serial.print(averageCn0);
    Serial.print(' ');
    if (averageCn0 < 25) Serial.print(F("poor"));
    else if (averageCn0 < 30) Serial.print(F("weak"));
    else if (averageCn0 < 40) Serial.print(F("usable"));
    else Serial.print(F("good"));
  } else {
    Serial.print(F("n/a"));
  }
  Serial.println();

  Serial.print(F("Pos "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(',');
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("unavailable"));
  }
  Serial.print(F("  Alt "));
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters(), 1);
    Serial.print('m');
  } else {
    Serial.print(F("n/a"));
  }
  Serial.print(F("  Spd "));
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph(), 1);
    Serial.println(F("km/h"));
  } else {
    Serial.println(F("n/a"));
  }

  Serial.print(F("PPS "));
  if (ppsAge <= 5000) {
    Serial.print(ppsAge);
    Serial.print(F("ms"));
  } else {
    Serial.print(F("missing"));
  }
  Serial.print(F("  RTC "));
  if (!rtcAvailable) Serial.print(F("unavailable"));
  else Serial.print(rtcTimeValid ? F("valid ") : F("invalid "));
  if (rtcAvailable && rtcTimeValid) {
    const DateTime rtcTime = rtc.now();
    if (rtcTime.hour() < 10) Serial.print('0');
    Serial.print(rtcTime.hour());
    Serial.print(':');
    if (rtcTime.minute() < 10) Serial.print('0');
    Serial.print(rtcTime.minute());
    Serial.print(':');
    if (rtcTime.second() < 10) Serial.print('0');
    Serial.print(rtcTime.second());
  }
  Serial.println();

  Serial.print(F("SHTC3 T "));
  if (sensorReadingValid) {
    Serial.print(roomTemperatureC, 1);
    Serial.print(F("C  RH "));
    Serial.print(roomHumidityPercent, 1);
    Serial.print(F("% age "));
    Serial.print(now - lastSensorReadMillis);
    Serial.println(F("ms"));
  } else {
    Serial.println(F("unavailable"));
  }
  Serial.println(F("-------------------"));
}

/**
 * @brief Toggle serial debug mode and report the mode change.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void handleDebugButtonPress() {
  debugMode = !debugMode;
  if (debugMode) {
    Serial.println();
    Serial.println(F("Debug mode entered."));
    Serial.println(F("C/N0 guide: <25 poor, 25-29 weak, 30-39 usable, >=40 good (dB-Hz)"));
    Serial.println(F("HDOP guide: <1 excellent, 1-<2 good, 2-<5 moderate, >=5 weak geometry"));
    printDebugTelemetry();
  } else {
    noTone(DEBUG_BUZZER_PIN);
    Serial.println();
    Serial.println(F("Debug mode exited."));
  }
}

/**
 * @brief Debounce the active-low onboard button and detect press edges.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void serviceDebugButton() {
  const uint32_t now = millis();
  const bool reading = digitalRead(USER_BUTTON_PIN);
  if (reading != buttonLastReading) {
    buttonLastReading = reading;
    buttonLastChangeMillis = now;
  }

  if (now - buttonLastChangeMillis >= BUTTON_DEBOUNCE_MS &&
      reading != buttonStableState) {
    buttonStableState = reading;
    if (!buttonStableState) handleDebugButtonPress();
  }
}

/**
 * @brief Parse a decimal field only when it fits within a supplied bound.
 * @param text Null-terminated field containing decimal digits.
 * @param maximum Largest accepted value, inclusive.
 * @param value Output reference assigned only when parsing succeeds.
 * @return `true` for a nonempty, all-digit value no greater than `maximum`.
 * @throws None.
 */
bool parseBoundedUnsigned(const char *text, uint16_t maximum,
                          uint16_t &value) {
  if (text == nullptr || *text == '\0') return false;

  uint16_t parsed = 0;
  for (const char *cursor = text; *cursor != '\0'; ++cursor) {
    if (*cursor < '0' || *cursor > '9') return false;
    const uint8_t digit = (uint8_t)(*cursor - '0');
    if (digit > maximum || parsed > (maximum - digit) / 10) return false;
    parsed = parsed * 10 + digit;
  }

  value = parsed;
  return true;
}

/**
 * @brief Convert one hexadecimal checksum character to its numeric value.
 * @param character ASCII hexadecimal digit, upper- or lowercase.
 * @return Value from 0 through 15, or -1 when the character is not hexadecimal.
 * @throws None.
 */
int8_t hexDigitValue(char character) {
  if (character >= '0' && character <= '9') return character - '0';
  if (character >= 'A' && character <= 'F') return character - 'A' + 10;
  if (character >= 'a' && character <= 'f') return character - 'a' + 10;
  return -1;
}

/**
 * @brief Validate a completed NMEA GSV sentence and update mean C/N0 state.
 * @details Validates the XOR checksum before interpreting signal-to-noise
 *          fields. GSV messages are accumulated until the last message in a
 *          sequence. Sequence metadata is capped at 16 messages (enough for
 *          64 satellites) and decimal fields are parsed with overflow checks.
 *          Malformed or checksum-failing sentences are discarded.
 * @note No parameters. Reads and modifies the shared `gsvSentence` buffer.
 * @return Nothing.
 * @throws None.
 */
void parseGsvSentence() {
  char *checksumSeparator = strchr(gsvSentence, '*');
  if (checksumSeparator == nullptr || strlen(checksumSeparator + 1) < 2) return;

  uint8_t checksum = 0;
  for (char *p = gsvSentence + 1; p < checksumSeparator; ++p) checksum ^= *p;
  const int8_t checksumHigh = hexDigitValue(checksumSeparator[1]);
  const int8_t checksumLow = hexDigitValue(checksumSeparator[2]);
  if (checksumHigh < 0 || checksumLow < 0 ||
      (checksumSeparator[3] != '\0' && checksumSeparator[3] != '\r')) {
    return;
  }
  const uint8_t expectedChecksum = (checksumHigh << 4) | checksumLow;
  if (checksum != expectedChecksum || strncmp(gsvSentence + 3, "GSV", 3) != 0) return;
  *checksumSeparator = '\0';

  uint8_t totalMessages = 0;
  uint8_t messageNumber = 0;
  uint8_t sentenceSignalCount = 0;
  uint16_t sentenceSignalSum = 0;
  uint8_t fieldNumber = 0;
  uint16_t parsedField = 0;
  char *fieldStart = gsvSentence;
  for (char *cursor = gsvSentence;; ++cursor) {
    if (*cursor == ',' || *cursor == '\0') {
      const char delimiter = *cursor;
      *cursor = '\0';
      if (fieldNumber == 1 && parseBoundedUnsigned(fieldStart, 16, parsedField)) {
        totalMessages = (uint8_t)parsedField;
      } else if (fieldNumber == 2 && parseBoundedUnsigned(fieldStart, 16, parsedField)) {
        messageNumber = (uint8_t)parsedField;
      } else if (fieldNumber >= 7 && (fieldNumber - 4) % 4 == 3 &&
                 sentenceSignalCount < 4 &&
                 parseBoundedUnsigned(fieldStart, 99, parsedField)) {
        // GSV has four fields per satellite (PRN/elevation/azimuth/C/N0);
        // signal values therefore occur at field indexes 7, 11, 15, ... .
        if (parsedField > 0) {
          sentenceSignalSum += parsedField;
          ++sentenceSignalCount;
        }
      }
      ++fieldNumber;
      if (delimiter == '\0') break;
      fieldStart = cursor + 1;
    }
  }

  // Reject malformed sequence metadata so counters cannot wrap on bad input.
  if (totalMessages == 0 || messageNumber == 0 || messageNumber > totalMessages) return;

  if (messageNumber == 1) {
    gsvSignalSum = 0;
    gsvSignalCount = 0;
  }
  gsvSignalSum += sentenceSignalSum;
  gsvSignalCount += sentenceSignalCount;
  if (totalMessages != 0 && messageNumber == totalMessages) {
    cn0Valid = gsvSignalCount != 0;
    if (cn0Valid) averageCn0 = gsvSignalSum / gsvSignalCount;
    lastCn0UpdateMillis = millis();
  }
}

/**
 * @brief Feed one received byte to TinyGPSPlus and the GSV signal parser.
 * @param character Byte received from the GPS receiver's NMEA stream.
 * @return Nothing.
 * @throws None.
 */
void processGpsCharacter(char character) {
  gps.encode(character);
  if (character == '$') {
    // A new sentence start resynchronizes after truncated or oversized input.
    gsvSentence[0] = '$';
    gsvSentenceLength = 1;
  } else if (gsvSentenceLength != 0) {
    if (character == '\n') {
      gsvSentence[gsvSentenceLength] = '\0';
      parseGsvSentence();
      gsvSentenceLength = 0;
    } else if (gsvSentenceLength < sizeof(gsvSentence) - 1) {
      gsvSentence[gsvSentenceLength++] = character;
    } else {
      gsvSentenceLength = 0;
    }
  }
}

/**
 * @brief Process a bounded batch of buffered GPS input bytes.
 * @details Limiting each batch lets the main loop return frequently to clock
 *          edge handling while still draining NMEA bursts faster than they
 *          arrive at 9600 baud.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void serviceGpsSerial() {
  // Bound parsing work so PPS/RTC service and display work keep getting time.
  uint8_t processed = 0;
  while (gpsSerial.available() && processed < GPS_RX_BATCH_SIZE) {
    processGpsCharacter(gpsSerial.read());
    ++processed;
  }
}

/**
 * @brief Select clock, temperature, or humidity page from the local-time cycle.
 * @details A 19-second cycle contains 15 seconds of clock, 2 seconds of
 *          temperature, and 2 seconds of humidity.
 * @param time Croatia-local time used to calculate the cycle position.
 * @return `true` if the selected page changed; otherwise `false`.
 * @throws None.
 */
bool selectDisplayPage(const DateTime &time) {
  const uint8_t position = time.unixtime() % DISPLAY_CYCLE_SECONDS;
  DisplayPage newPage = CLOCK_PAGE;
  if (position >= CLOCK_PAGE_SECONDS + TEMPERATURE_PAGE_SECONDS) {
    newPage = HUMIDITY_PAGE;
  } else if (position >= CLOCK_PAGE_SECONDS) {
    newPage = TEMPERATURE_PAGE;
  }
  const bool changed = newPage != displayPage;
  displayPage = newPage;
  return changed;
}

/**
 * @brief Read the SHTC3 on its nonblocking 5-second schedule.
 * @details Updates the current environmental values and redraws an active
 *          environmental page. The call occurs after clock-edge processing.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void serviceSensor() {
  if (!shtc3Available || millis() - lastSensorReadMillis < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMillis = millis();

  sensors_event_t humidityEvent;
  sensors_event_t temperatureEvent;
  sensorReadingValid = shtc3.getEvent(&humidityEvent, &temperatureEvent);
  sensorReadingValid = sensorReadingValid &&
                       isfinite(temperatureEvent.temperature) &&
                       isfinite(humidityEvent.relative_humidity);
  if (sensorReadingValid) {
    roomTemperatureC = temperatureEvent.temperature;
    roomHumidityPercent = humidityEvent.relative_humidity;
  }
  if (displayPage == TEMPERATURE_PAGE || displayPage == HUMIDITY_PAGE) {
    showCurrentPage();
  }
}

/**
 * @brief Return the number of days in a Gregorian calendar month.
 * @param year Four-digit Gregorian year used for leap-year rules.
 * @param month Month number in the range 1 through 12.
 * @return Number of days in the requested month; zero if the month is invalid.
 * @throws None.
 */
uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t days[] = {31, 28, 31, 30, 31, 30,
                                 31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 0;
  if (month == 2 && (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0))) {
    return 29;
  }
  return days[month - 1];
}

/**
 * @brief Find the calendar day of the month's final Sunday.
 * @param year Gregorian year.
 * @param month Month number in the range 1 through 12.
 * @return Day of month on which the final Sunday falls; zero if month invalid.
 * @throws None.
 */
uint8_t lastSundayOfMonth(uint16_t year, uint8_t month) {
  if (month < 1 || month > 12) return 0;
  const uint8_t lastDay = daysInMonth(year, month);
  const DateTime finalDate(year, month, lastDay, 0, 0, 0);
  return lastDay - finalDate.dayOfTheWeek();  // RTClib: Sunday == 0
}

/**
 * @brief Determine whether a UTC instant falls in Croatia's CEST period.
 * @details Croatia uses the EU rule: DST begins at 01:00 UTC on the final
 *          Sunday in March and ends at 01:00 UTC on the final Sunday in
 *          October. UTC is tested directly to avoid ambiguous local times.
 * @param utc UTC timestamp to classify.
 * @return `true` during CEST; otherwise `false` during CET.
 * @throws None.
 */
bool isCroatiaSummerTime(const DateTime &utc) {
  const uint8_t month = utc.month();
  if (month < 3 || month > 10) return false;
  if (month > 3 && month < 10) return true;

  if (month == 3) {
    const uint8_t changeDay = lastSundayOfMonth(utc.year(), 3);
    return utc.day() > changeDay || (utc.day() == changeDay && utc.hour() >= 1);
  }

  const uint8_t changeDay = lastSundayOfMonth(utc.year(), 10);
  return utc.day() < changeDay || (utc.day() == changeDay && utc.hour() < 1);
}

/**
 * @brief Convert UTC to Croatia local time using CET or CEST as appropriate.
 * @param utc UTC timestamp from GPS or the DS3231.
 * @return Local DateTime with a +1 or +2 hour offset.
 * @throws None.
 */
DateTime croatiaLocalTime(const DateTime &utc) {
  const int8_t utcOffsetHours = isCroatiaSummerTime(utc) ? 2 : 1;
  return utc + TimeSpan(0, utcOffsetHours, 0, 0);
}

/**
 * @brief Update displayed clock state and render the page for a clock tick.
 * @param time UTC time supplied by GPS or DS3231.
 * @param tickMillis Millisecond timestamp captured at the source clock edge.
 * @return Nothing.
 * @throws None.
 */
void showClockTime(const DateTime &time, uint32_t tickMillis) {
  // GPS and DS3231 stay in UTC; apply Croatia's CET/CEST offset for display.
  const DateTime localTime = croatiaLocalTime(time);
  shownHour = localTime.hour();
  shownMinute = localTime.minute();
  shownSecond = localTime.second();
  lastClockTickMillis = tickMillis;
  haveShownTime = true;
  const bool pageChanged = selectDisplayPage(localTime);
  colonsVisible = true;
  if (pageChanged) {
    showCurrentPage();
  } else if (displayPage == CLOCK_PAGE) {
    drawDateTime(localTime, true);
  }
}

/**
 * @brief Consume pending PPS/RTC edges and refresh the displayed time.
 * @details Copies ISR-shared values atomically. A fresh PPS plus fresh NMEA
 *          date/time selects GPS; NEO-6M's NMEA epoch is advanced one second
 *          to label the PPS edge. If GPS timing data expires, a valid RTC tick
 *          becomes the fallback. Display I/O happens before RTC synchronization
 *          and debug serial output to minimize edge-to-display latency.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void serviceClock() {
  noInterrupts();
  const bool gpsPpsTick = gpsPpsPending;
  const bool rtcTick = rtcTickPending;
  const bool seenGpsPps = haveSeenGpsPps;
  const uint32_t lastPps = lastGpsPpsMillis;
  const uint32_t lastRtcTick = lastRtcTickMillis;
  gpsPpsPending = false;
  rtcTickPending = false;
  interrupts();

  if (gpsPpsTick) {
    if ((uint32_t)(millis() - lastPps) < GPS_PPS_HIGH_MS) {
      // The module indicator uses the inverse of this PPS pulse phase. Keep
      // the board LED dark during PPS-high, then illuminate it for the rest
      // of the second so both indicators visibly blink together.
      setBoardLed(false);
      ppsLedOffIntervalActive = true;
      ppsLedOffStartMillis = lastPps;
    }
  }

  const bool ppsFresh = seenGpsPps && (millis() - lastPps <= 1500);
  const bool nmeaTimeFresh = gps.time.isValid() && gps.time.age() <= 1500;
  const bool nmeaDateFresh = gps.date.isValid() && gps.date.age() <= 2000;

  if (gpsPpsTick && ppsFresh && nmeaTimeFresh && nmeaDateFresh) {
    // NEO-6M emits the NMEA epoch after its PPS edge. Advance its timestamp
    // one second to label the PPS edge currently being handled.
    const DateTime nmeaTime(
        gps.date.year(), gps.date.month(), gps.date.day(),
        gps.time.hour(), gps.time.minute(), gps.time.second());
    const DateTime gpsTimeAtPps = nmeaTime + TimeSpan(1);

    const bool justAcquiredGps = !usingGps;
    usingGps = true;

    // Display first to minimize PPS-to-display latency. Synchronize the backup
    // RTC immediately afterward on GPS acquisition, then once per hour.
    const uint32_t gpsEpoch = gpsTimeAtPps.unixtime();
    showClockTime(gpsTimeAtPps, lastPps);
    if (rtcAvailable &&
        (justAcquiredGps || gpsEpoch - lastRtcSyncEpoch >= 3600)) {
      rtc.adjust(gpsTimeAtPps);
      rtcTimeValid = true;
      lastRtcSyncEpoch = gpsEpoch;
    }
    if (debugMode) printDebugTelemetry();
  } else {
    if (!ppsFresh || !nmeaTimeFresh || !nmeaDateFresh) {
      usingGps = false;
    }

    if (rtcTick && !usingGps && rtcAvailable && rtcTimeValid) {
      showClockTime(rtc.now(), lastRtcTick);
      if (debugMode) printDebugTelemetry();
    }
  }
}

/**
 * @brief Phase-lock the display colons to the most recently handled clock tick.
 * @details `lastClockTickMillis` is captured at the PPS/RTC edge; the colons
 *          alternate every 500 ms for one complete cycle per second.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void serviceColonBlink() {
  if (!haveShownTime || displayPage != CLOCK_PAGE) return;

  // One complete on/off cycle per second, phase-locked to the latest clock tick.
  const bool shouldBeVisible = ((millis() - lastClockTickMillis) % 1000) < 500;
  if (shouldBeVisible != colonsVisible) {
    colonsVisible = shouldBeVisible;
    const DateTime shownTime(2000, 1, 1, shownHour, shownMinute, shownSecond);
    drawDateTime(shownTime, colonsVisible);
  }
}

/**
 * @brief End the short LED-off interval generated by a fresh PPS edge.
 * @note No parameters.
 * @return Nothing.
 * @throws None.
 */
void servicePpsLed() {
  if (ppsLedOffIntervalActive &&
      (uint32_t)(millis() - ppsLedOffStartMillis) >= GPS_PPS_HIGH_MS) {
    setBoardLed(true);
    ppsLedOffIntervalActive = false;
  }
}

/**
 * @brief Initialize displays, sensors, serial links, RTC, and interrupts.
 * @note No parameters. Called once by the Arduino runtime after reset.
 * @return Nothing.
 * @throws None. Device availability is recorded in state for runtime fallback.
 */
void setup() {
  display.begin();
  display.control(MD_MAX72XX::INTENSITY, MAX7219_INTENSITY);
  display.clear();
  boardLed.begin();
  boardLed.setBrightness(255);
  setBoardLed(true);

  Serial.begin(115200);
  pinMode(DEBUG_BUZZER_PIN, OUTPUT);
  shtc3Available = shtc3.begin();

  gpsSerial.begin(9600);
  // The GPS receiver only sends data to D6. Free D7 for the onboard button.
  pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
  buttonLastReading = digitalRead(USER_BUTTON_PIN);
  buttonStableState = buttonLastReading;
  rtcAvailable = rtc.begin();
  if (rtcAvailable) {
    rtcTimeValid = !rtc.lostPower();
    rtc.disable32K();
    rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
    pinMode(RTC_SQW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), onRtcSquareWave, FALLING);
  }

  pinMode(GPS_PPS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), onGpsPps, RISING);
}

/**
 * @brief Run one cooperative service iteration with clock edges first.
 * @details Service order is intentional: consume pending timing edges, handle
 *          the button and indicators, parse a bounded GPS batch, recheck for
 *          edges that arrived during parsing, then perform sensor and visual
 *          maintenance work.
 * @note No parameters. Called repeatedly by the Arduino runtime.
 * @return Nothing.
 * @throws None.
 */
void loop() {
  // Give interrupt-latched clock edges first priority in the main loop.
  serviceClock();
  serviceDebugButton();
  updateGpsStatus();
  serviceGpsSerial();
  updateGpsStatus();
  // Catch an edge that arrived while the bounded GPS batch was being parsed.
  serviceClock();
  serviceSensor();
  serviceColonBlink();
  servicePpsLed();
}
