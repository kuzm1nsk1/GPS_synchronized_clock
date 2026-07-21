#include "MAX7219.h"

MAX7219::MAX7219(uint8_t dinPin, uint8_t clkPin, uint8_t csPin, uint8_t devicesNum) {

  _dinPin = dinPin;
  _clkPin = clkPin;
  _csPin = csPin;
  _devicesNum = devicesNum;

  pinMode(_dinPin, OUTPUT);
  pinMode(_clkPin, OUTPUT);
  pinMode(_csPin, OUTPUT);
}

void MAX7219::begin() {

  SPI.begin();

  send(DISPLAY_TEST, 0);
  send(SCAN_LIMIT, 7);
  send(DECODE_MODE, 0);
  send(INTENSITY, 1);
  displayBitmap(_bitmap);
  send(SHUTDOWN, 1);
}


void MAX7219::allOn() {

  clear(_bitmap);
  displayBitmap(_bitmap);
  send(DISPLAY_TEST, 1);
}


void MAX7219::allOff() {

  send(DISPLAY_TEST, 0);
}


void MAX7219::send(uint8_t addr, uint8_t data) {

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  digitalWrite(_csPin, LOW);

  for (int i = 0; i < _devicesNum; i++) {
    SPI.transfer(addr);
    SPI.transfer(data);
  }
  digitalWrite(_csPin, HIGH);
  
  SPI.endTransaction();
}


void MAX7219::displayBitmap(uint32_t (&bitmap)[8]) {

  for (int i = 1; i < 9; i++) {
    digitalWrite(_csPin, LOW);

    for (int j = 0; j <= 3; j++) {
      SPI.transfer(i);
      SPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
      SPI.transfer((bitmap[i - 1] >> j * 8) & 0xFF);
      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    }
    digitalWrite(_csPin, HIGH);
  }
  SPI.endTransaction();
}


void MAX7219::clear(uint32_t (&bitmap)[8]) {
  
  for (int i = 0; i < 8; i++) {
    bitmap[i] = 0;
  }
}


void MAX7219::setBrightness(uint8_t brightness) {

  send(INTENSITY, brightness);
}


void MAX7219::display(String str) {

  uint8_t fontOffset;

  font ? fontOffset = 100 : fontOffset = 0;

  clear(_bitmap);

  int length = str.length();
  int pos = 0;

  for (int i = 0; i < length; i++) {
    uint16_t startIndex = 0;

    for (int j = 0; j < (str[i] + fontOffset); j++) {
      startIndex += pgm_read_byte(&MAX7219_FONT[startIndex]) + 1;
    }

    uint8_t charLength = pgm_read_byte(&MAX7219_FONT[startIndex]);

    for (int i = 0; i < charLength; ++i) {
      uint8_t data = pgm_read_byte(&MAX7219_FONT[startIndex + 1 + i]);

      for (int j = 0; j < 8; j++) {
        bitWrite(_bitmap[j], 31 - pos, bitRead(data, 7 - j));
      }
      pos++;
    }
    pos++;
  }
  displayBitmap(_bitmap);
}