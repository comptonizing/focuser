#pragma once

#include <inttypes.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <util/crc16.h>

namespace Storage {

void readEEPROM(uint16_t address, uint8_t *buff, uint16_t n);
void writeEEPROM(uint16_t address, uint8_t *buff, uint16_t n);
uint16_t crcCalc(uint8_t *data, uint16_t n);
uint16_t crcCalc(const char *str);
uint16_t crcCalc(const __FlashStringHelper *data, uint16_t n);

}
