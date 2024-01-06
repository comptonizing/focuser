#include "Storage.h"

namespace Storage {

void readEEPROM(uint16_t address, uint8_t *buff, uint16_t n) {
  for (int ii=0; ii<n; ii++) {
    buff[ii] = EEPROM.read(address+ii);
  }
}

void writeEEPROM(uint16_t address, uint8_t *buff, uint16_t n) {
  digitalWrite(13, HIGH);
  for (int ii=0; ii<n; ii++) {
    EEPROM.write(address+ii, buff[ii]);
  }
  digitalWrite(13, LOW);
}

uint16_t crcCalc(uint8_t *data, uint16_t n) {
  uint16_t crc = 0;
  for (uint16_t ii=0; ii<n; ii++) {
    crc = _crc16_update(crc, data[ii]);
  }
  return crc;
}

uint16_t crcCalc(const char *str) {
  uint16_t crc = 0;
  while ( *str != '\0' ) {
    crc = _crc16_update(crc, *str);
    str++;
  }
  return crc;
}

uint16_t crcCalc(const __FlashStringHelper *data, uint16_t n) {
  const char *ptr = reinterpret_cast<const char *>(data);
  uint16_t crc = 0;
  for (uint16_t ii=0; ii<n; ii++) {
    crc = _crc16_update(crc, pgm_read_byte(ptr+ii));
  }
  return crc;
}

}
