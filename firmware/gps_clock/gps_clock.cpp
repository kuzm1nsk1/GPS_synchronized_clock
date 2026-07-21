/*
 * GPS Synchronized Clock — main firmware
 * ======================================
 * Disciplines an 8x32 MAX7219 LED-matrix clock to GPS time, falling back to a
 * DS3231 RTC when GPS is unavailable. A rotary encoder drives a small menu
 * system (clock / alarm / timer / stopwatch / settings). An SHTC3 sensor adds
 * a temperature readout, and a buzzer sounds the alarm.
 *
 * Time source arbitration (see GPS_ISR / RTC_ISR):
 *   - The GPS module's PPS output (gpsINTR, rising edge) marks the true second.
 *     While PPS pulses arrive, usingRTC = false and time comes from the GPS NMEA
 *     stream.
 *   - The DS3231's 1 Hz square wave (rtcINTR, falling edge) is the fallback. If
 *     no PPS has been seen for > ~1 s, usingRTC flips back to true and the RTC
 *     drives the display. The RTC is periodically re-synced from GPS.
 *
 * -------------------------------------------------------------------------
 * PlatformIO note: this file is a .cpp (not a .ino). A .ino is auto-prototyped
 * by the Arduino build; a .cpp is not, and this sketch calls many functions
 * before they are defined. The forward declarations below provide those
 * prototypes so it compiles unchanged under PlatformIO. The logic is identical
 * to the original sketch. To open in the Arduino IDE, copy this file to
 * gps_clock.ino (the #include <Arduino.h> line is harmless there).
 * -------------------------------------------------------------------------
 */

#include <Arduino.h>              // Core types/functions (String, millis, tone, pinMode, ...)
#include <Wire.h>                 // I2C (SHTC3, DS3231, Soldered easyC)
#include <EEPROM.h>              // Persist settings across power loss
#include <RTClib.h>              // DS3231 real-time clock
#include <SoftwareSerial.h>      // Serial link to the NEO-6M GPS module
#include <TinyGPSPlus.h>         // NMEA parser
#include <Rotary-encoder-easyC-SOLDERED.h>  // Soldered rotary encoder (easyC/I2C)
#include <SHTC3-SOLDERED.h>                  // Soldered SHTC3 temperature sensor
#include "MAX7219.h"             // Custom 8x32 matrix driver (bundled in this folder)

// ---- Pin map ----
#define rtcINTR 2                 // DS3231 1 Hz square-wave input (INT0)
#define gpsINTR 3                 // GPS PPS input (INT1)
#define txPin 7                   // SoftwareSerial TX -> GPS RX
#define rxPin 6                   // SoftwareSerial RX <- GPS TX
#define buzzerPin 9               // Alarm buzzer
#define toneFrequency 500         // Alarm tone (Hz)

// ---- Devices ----
MAX7219 max7219;                  // Default pins: DIN 11, CLK 13, CS 10, 4 modules
RTC_DS3231 rtc;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(rxPin, txPin);
Rotary rotary;
SHTC3 shtcSensor;

// ---- Current time being displayed/edited ----
// Laid out as three consecutive int8_t so timePtr can step field-by-field
// (hour -> minute -> second) when editing alarm/timer values.
struct {
  int8_t hour;
  int8_t minute;
  int8_t second;
} time;

int8_t* timePtr = (int8_t*)&time; // Points at the field currently being edited

// ---- Persisted display settings ----
// Field order matches EEPROM addresses 1..4 (see setup / longPress), so the
// config-menu enum index doubles as the EEPROM address for each setting.
struct {
  bool FONT;                      // false = HH:MM:SS font, true = HH:MM font
  int8_t BRT;                     // Brightness 1..15
  bool DST;                       // Daylight saving time offset
  bool ALARM;                     // Alarm enabled
} matrix;

uint8_t* matrixPtr = (uint8_t*)&matrix; // Byte view, for EEPROM save in longPress

// ---- Top-level menu state machine ----
enum menu {
  menuBlank,
  clock,
  alarm,
  timer,
  stopwatch,
  config
};
menu menuActive = clock;          // Which screen is active
menu menuPos = clock;             // Highlighted item while browsing from the clock

// ---- Settings sub-menu state machine ----
enum configMenu {
  configMenuBlank,
  font,
  brightness,
  daylightSavingTime,
  alarmOnOff
};
configMenu configMenuActive = configMenuBlank;
configMenu configMenuPos = font;

// ---- Runtime flags / timers ----
uint32_t prevTime = 0, startTime = 0;
bool alarmActive = false, timerActive = false, stopwatchActive = false, buzzerState = false;
volatile bool interruptFlag = false, usingRTC = true;  // Set from ISRs


// ---- Forward declarations ----
// Required for .cpp compilation (a .ino would generate these automatically).
void clockwise();
void counterClockwise();
void click();
void longPress();
void doubleClick();
void turnOnAlarm();
void turnOffAlarm();
void updateTimer();
void updateStopwatch();
void toggleBuzzer();
void updateTime();
void displayMenu();
String time2String(uint8_t hour, uint8_t minute, uint8_t second);
String temp2String(float t);
void RTC_ISR(void);
void GPS_ISR(void);
void checkTime();


void setup() {

  max7219.begin();
  gpsSerial.begin(9600);

  shtcSensor.begin();
  rotary.begin();
  rotary.resetCounter();

  // DS3231 setup: disable the 32 kHz output and any stale alarms, then emit a
  // 1 Hz square wave on the INT/SQW pin (our RTC "tick" source).
  rtc.begin();
  rtc.disable32K();
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

  pinMode(rtcINTR, INPUT_PULLUP);
  pinMode(gpsINTR, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  // RTC tick on falling edge, GPS PPS on rising edge.
  attachInterrupt(digitalPinToInterrupt(rtcINTR), RTC_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(gpsINTR), GPS_ISR, RISING);

  // Restore persisted settings (EEPROM addresses 1..4 = FONT, BRT, DST, ALARM).
  matrix.FONT = EEPROM.read(1);
  matrix.BRT = EEPROM.read(2);
  matrix.DST = EEPROM.read(3);
  matrix.ALARM = EEPROM.read(4);

  max7219.setBrightness(matrix.BRT);
  max7219.font = matrix.FONT;
}


void loop() {

  uint8_t state = rotary.getState();

  // Ignore encoder input during the first 100 ms after power-up (startup noise).
  if (millis() < 100) state = ROTARY_IDLE;

  // When GPS is the active source, keep feeding NMEA bytes to the parser.
  if (!usingRTC) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
  }

  // Fire the alarm when the RTC's Alarm 1 matches (and alarms are enabled).
  if (rtc.alarmFired(1) && matrix.ALARM) turnOnAlarm();
  if (alarmActive) toggleBuzzer();

  // interruptFlag is set once per second by whichever tick source is active.
  if (interruptFlag) {
    if (menuActive == clock) {
      updateTime();
      if (menuPos == clock) {
        // Show the time for most of the minute; flash temperature at :56.
        if (time.second <= 55) displayMenu();
        else if (time.second == 56) {
          shtcSensor.sample();
          max7219.display(temp2String(shtcSensor.readTempC()));
        }
      }
    } else {
      if (timerActive) updateTimer();
      if (stopwatchActive) updateStopwatch();
    }
    interruptFlag = false;
  }

  // Dispatch encoder events. While the alarm is sounding, any input silences it.
  if (state != ROTARY_IDLE) {
    if (alarmActive) turnOffAlarm();
    else {
      switch (state) {
        case ROTARY_CW: clockwise(); break;
        case ROTARY_CCW: counterClockwise(); break;
        case BTN_CLICK: click(); break;
        case BTN_LONG_PRESS: longPress(); break;
        case BTN_DOUBLE_CLICK: doubleClick(); break;
      }
    }

    state = ROTARY_IDLE;
  }
}


// Encoder turned clockwise: advance menu selection or increment the edited field.
void clockwise() {

  switch (menuActive) {
    case menuBlank:
    case stopwatch: break;
    case clock:
      menuPos = (menu)((int)menuPos + 1);
      if (menuPos > config) menuPos = clock;
      displayMenu();
      break;

    case alarm:
    case timer:
      if (!timerActive && !stopwatchActive) {
        (*timePtr)++;              // Increment the field timePtr points at
        checkTime();
        displayMenu();
      }
      break;

    case config:
      if (configMenuActive == configMenuBlank) {
        // Browsing the settings list.
        configMenuPos = (configMenu)((int)configMenuPos + 1);
        if (configMenuPos > alarmOnOff) configMenuPos = font;
        displayMenu();
      } else {
        // Editing a specific setting.
        switch (configMenuActive) {
          case configMenuBlank: break;

          case font:
            matrix.FONT = !matrix.FONT;
            max7219.font = matrix.FONT;
            displayMenu();
            break;

          case brightness:
            matrix.BRT++;
            if (matrix.BRT > 15) matrix.BRT = 1;   // Wrap 15 -> 1
            max7219.setBrightness(matrix.BRT);
            displayMenu();
            break;

          case daylightSavingTime:
            matrix.DST = !matrix.DST;
            displayMenu();
            break;

          case alarmOnOff:
            matrix.ALARM = !matrix.ALARM;
            displayMenu();
            break;
        }
      }
      break;
  }
}


// Encoder turned counter-clockwise: mirror of clockwise(), decrementing.
void counterClockwise() {

  switch (menuActive) {
    case menuBlank:
    case stopwatch: break;
    case clock:
      menuPos = (menu)((int)menuPos - 1);
      if (menuPos < clock) menuPos = config;
      displayMenu();
      break;

    case alarm:
    case timer:
      if (!timerActive && !stopwatchActive) {
        (*timePtr)--;
        checkTime();
        displayMenu();
      }
      break;

    case config:
      if (configMenuActive == configMenuBlank) {
        configMenuPos = (configMenu)((int)configMenuPos - 1);
        if (configMenuPos < font) configMenuPos = alarmOnOff;
        displayMenu();
      } else {
        switch (configMenuActive) {
          case configMenuBlank: break;
          case font:
            matrix.FONT = !matrix.FONT;
            max7219.font = matrix.FONT;
            displayMenu();
            break;

          case brightness:
            matrix.BRT--;
            if (matrix.BRT < 1) matrix.BRT += 15;   // Wrap 0 -> 15
            max7219.setBrightness(matrix.BRT);
            displayMenu();
            break;

          case daylightSavingTime:
            matrix.DST = !matrix.DST;
            displayMenu();
            break;
          case alarmOnOff:
            matrix.ALARM = !matrix.ALARM;
            displayMenu();
            break;
        }
      }
      break;
  }
}

// Short press: enter the highlighted menu, or in alarm/timer advance the edited
// field (hour -> minute -> second, wrapping via pointer arithmetic).
void click() {

  uint8_t timeSize = sizeof(time);

  switch (menuActive) {
    case menuBlank:
    case stopwatch: break;
    case clock:
      menuActive = menuPos;
      if (menuActive == timer || menuActive == stopwatch) {
        time.hour = 0;
        time.minute = 0;
        time.second = 0;
      }
      displayMenu();
      break;

    case alarm:
    case timer:
      if (!timerActive && !stopwatchActive) {
        timePtr++;                 // Move to the next time field
        // Wrap back to the first field once past the end of the struct.
        if (timePtr == (int8_t*)&time + timeSize) timePtr -= timeSize;
      }
      break;

    case config:
      if (configMenuActive == configMenuBlank) configMenuActive = configMenuPos;
      displayMenu();
      break;
  }
}


// Long press: confirm/commit the current screen.
void longPress() {

  switch (menuActive) {
    case menuBlank:
    case clock: break;
    case alarm:
      // Program DS3231 Alarm 1 to match on hours/minutes/seconds. The SQW is
      // briefly turned off while writing the alarm, then the 1 Hz tick resumes.
      rtc.writeSqwPinMode(DS3231_OFF);
      rtc.setAlarm1(DateTime(2024, 5, 2, time.hour, time.minute, time.second), DS3231_A1_Hour);
      rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
      menuActive = clock;
      menuPos = clock;
      timePtr = (int8_t*)&time;
      break;

    case timer:
      timerActive = true;          // Start counting down
      timePtr = (int8_t*)&time;
      menuActive = timer;
      menuPos = menuBlank;
      displayMenu();
      break;

    case stopwatch:
      stopwatchActive = true;      // Start counting up
      menuActive = stopwatch;
      menuPos = menuBlank;
      displayMenu();
      break;

    case config:
      if (configMenuActive != configMenuBlank) {
        // Persist the edited setting. The config enum index (1..4) is also the
        // EEPROM address and the byte offset into the matrix struct.
        EEPROM.update((uint8_t)configMenuActive, *(matrixPtr + (uint8_t)configMenuActive - 1));
        configMenuActive = configMenuBlank;
        configMenuPos = font;
        displayMenu();
      }
      break;
  }
}


// Double press: back out to the clock (or one level up within settings).
void doubleClick() {

  switch (menuActive) {
    case menuBlank: break;
    case clock:
    case alarm:
    case timer:
    case stopwatch:
      menuActive = clock;
      menuPos = clock;
      timerActive = false;
      stopwatchActive = false;
      timePtr = (int8_t*)&time;
      displayMenu();
      break;

    case config:
      if (configMenuActive == configMenuBlank) {
        menuActive = clock;
        menuPos = clock;
        displayMenu();
        break;
      } else {
        configMenuActive = configMenuBlank;
        displayMenu();
      }
      break;
  }
}


// Start the alarm: pause GPS serial, clear the RTC alarm latch, flag active.
void turnOnAlarm() {

  gpsSerial.end();
  rtc.clearAlarm(1);
  alarmActive = true;
  menuActive = menuBlank;
}


// Stop the alarm and return to the clock.
void turnOffAlarm() {

  gpsSerial.begin(9600);
  noTone(buzzerPin);
  max7219.allOff();
  alarmActive = false;
  timerActive = false;
  menuActive = clock;
  menuPos = clock;
}


// Countdown timer: decrement one second per tick; fire alarm at zero.
void updateTimer() {

  time.second--;
  checkTime();
  if (time.hour == 0 && time.minute == 0 && time.second == 0) {
    gpsSerial.end();
    alarmActive = true;
    timerActive = false;
  }
  interruptFlag = false;
  displayMenu();
}


// Stopwatch: increment one second per tick.
void updateStopwatch() {

  time.second++;
  checkTime();
  displayMenu();
  interruptFlag = false;
}


// Blink the buzzer/display roughly twice per second while the alarm is active.
void toggleBuzzer() {

  if (millis() - prevTime > 500) {
    prevTime = millis();
    buzzerState = !buzzerState;

    if (buzzerState) {
      tone(buzzerPin, toneFrequency);
      max7219.allOn();
    } else {
      noTone(buzzerPin);
      max7219.allOff();
    }
  }
}


// Refresh `time` from whichever source is active.
void updateTime() {

  if (usingRTC) {

    time.hour = rtc.now().hour();
    time.minute = rtc.now().minute();
    time.second = rtc.now().second();

  } else {

    if (gps.time.isValid()) {

      // GPS reports UTC; add 1 h base offset (+ DST) for local time. The +1 s
      // aligns the NMEA timestamp with the PPS edge that triggered this update.
      time.hour = gps.time.hour() + 1 + matrix.DST;
      time.minute = gps.time.minute();
      time.second = gps.time.second() + 1;
      checkTime();

      // Once a minute, push the accurate GPS time into the RTC as backup.
      if (time.minute == 30 && time.second == 30) {
        rtc.adjust(DateTime(gps.date.year(), gps.date.month(), gps.date.day(), time.hour, 30, 30));
      }
    }
  }
}


// Render the correct string for the current menu/config state onto the matrix.
void displayMenu() {

  if (menuActive != config) {

    if (menuPos != clock && menuActive == clock) {

      // Browsing menu labels from the clock screen.
      switch (menuPos) {
        case menuBlank:
        case clock: break;
        case alarm: matrix.FONT ? max7219.display("ALRM") : max7219.display("ALARM"); break;
        case timer: matrix.FONT ? max7219.display("TIMR") : max7219.display("TIMER"); break;
        case stopwatch: matrix.FONT ? max7219.display("STPW") : max7219.display("STPW"); break;
        case config: matrix.FONT ? max7219.display("CONF") : max7219.display("CONFIG"); break;
      }
    } else {

      // Showing the running time (clock/alarm/timer/stopwatch).
      switch (menuActive) {
        case menuBlank:
        case clock:
        case alarm:
        case timer:
        case stopwatch: max7219.display(time2String(time.hour, time.minute, time.second)); break;
        case config: max7219.display("FONT"); break;
      }
    }
  } else {

    if (configMenuPos != configMenuBlank && configMenuActive == configMenuBlank) {

      // Settings list (item names).
      switch (configMenuPos) {
        case configMenuBlank: break;
        case font: max7219.display("FONT"); break;
        case brightness: max7219.display("BRT"); break;
        case daylightSavingTime: max7219.display("DST"); break;
        case alarmOnOff: max7219.display("ALRM"); break;
      }
    } else if (configMenuPos != configMenuBlank && configMenuActive != configMenuBlank) {

      // Editing a setting (current value).
      switch (configMenuActive) {
        case configMenuBlank: break;
        case font: max7219.display(matrix.FONT ? "12 : 34" : "12 : 34 : 56"); break;
        case brightness: max7219.display(String(matrix.BRT)); break;
        case daylightSavingTime: max7219.display(matrix.DST ? "ON" : "OFF"); break;
        case alarmOnOff: max7219.display(matrix.ALARM ? "ON" : "OFF"); break;
      }
    }
  }
}


// Format HH:MM(:SS) for the matrix. The colon "blinks" by swapping the
// separator between " : " and spaces on alternate (even/odd) seconds.
String time2String(uint8_t hour, uint8_t minute, uint8_t second) {

  String _hour = (hour < 10 ? "   " : String(hour / 10)) + String(hour % 10);
  String _minute = String(minute / 10) + String(minute % 10);
  String _second = String(second / 10) + String(second % 10);

  String timeStr;
  if (time.second % 2 == 0) timeStr = matrix.FONT ? _hour + " : " + _minute : _hour + " : " + _minute + " : " + _second;
  else timeStr = matrix.FONT ? _hour + "    " + _minute : _hour + "    " + _minute + "    " + _second;

  return timeStr;
}


// Format the temperature as "NN.Nc".
String temp2String(float t) {

  String _t = (matrix.FONT ? "" : "     ") + String((int)t) + "." + String(((int)(t * 10) % 10)) + "c";

  return _t;
}


// DS3231 1 Hz tick ISR (fallback source). Only accept it as the active tick if
// no GPS PPS has arrived for just over a second, then mark RTC as the source.
void RTC_ISR(void) {

  if (millis() - startTime > 1001) {
    usingRTC = true;
    interruptFlag = 1;
  }
}


// GPS PPS ISR (preferred source). Every pulse marks the true second edge and
// makes GPS the active source.
void GPS_ISR(void) {

  startTime = millis();
  usingRTC = false;
  interruptFlag = 1;
}


// Normalize `time` after +/- edits or timer/stopwatch rollover, carrying/
// borrowing between seconds, minutes and hours and wrapping at 24 h.
void checkTime() {

  if (time.second > 59) {
    time.second -= 60;
    time.minute++;
  }

  if (time.minute > 59) {
    time.minute -= 60;
    time.hour++;
  }

  if (time.hour > 23) time.hour -= 24;

  if (time.second < 0) {
    time.second += 60;
    time.minute--;
  }

  if (time.minute < 0) {
    time.minute += 60;
    time.hour--;
  }

  if (time.hour < 0) time.hour += 24;
}
