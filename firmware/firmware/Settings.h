/*
 *  This file is part of the Pollux Rotator software
 *
 *  Created by Philipp Weber
 *  Copyright (c) 2023 Philipp Weber
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */



#pragma once

#define MSG_PREFIX '#'
#define MSG_POSTFIX '$'

#define BUFFSIZE 384

#include <string.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <EEPROM.h>
#include <util/crc16.h>
#include <TMC2209.h>
#include <ArduinoJson.h>

#include "CommandBuffer.h"
#include "Motor.h"
#include "Storage.h"

class Settings {
  public:
    static Settings &i();
    bool runCommand(const char *cmd);
    void setup();
    void loop();
  private:
    Settings();
    ~Settings();
    Settings(const Settings&);
    Settings& operator=(const Settings&);
    bool loadFromEEPROM();
    void saveToEEPROM();
    void applySettings();

    bool runStatus(const char *cmd);
    bool runSetTarget(const char *cmd);
    bool runSetRunCurrent(const char *cmd);
    bool runSetHoldCurrent(const char *cmd);
    bool runSetMicroStepping(const char *cmd);
    bool runSetInverted(const char *cmd);
    bool runSetSpeed(const char *cmd);
    bool runSetAccel(const char *cmd);
    bool runSetStandStillMode(const char *cmd);
    bool runSetStealthChop(const char *cmd);
    bool runSetCoolStep(const char *cmd);
    bool runSync(const char *cmd);
    bool runStop(const char *cmd);
    bool runSetMaxSteps(const char *cmd);
    bool runSetBacklash(const char *cmd);
	bool runSetBacklashEnabled(const char *cmd);
    void runUnknownCommand();
    void saveAndAck();
    void sendMessage(char *msg);
    void sendMessage(const __FlashStringHelper *msg);
    void sendErrorMessage(char *msg);
    void sendErrorMessage(const __FlashStringHelper *msg);
    void sendStatus();

	step_t m_maxSteps = 10000;
	step_t m_backlash = 0;
	bool m_backlashEnabled = false;
    uint8_t m_runCurrent = 50;
    uint8_t m_holdCurrent = 10;
    uint8_t m_microStepping = 16;
    uint16_t m_speed = 500;
    uint16_t m_accel = 200;
    bool m_invert = false;
    bool m_stealthChop = false;
    bool m_coolStep = true;
    TMC2209::StandstillMode m_standStillMode = TMC2209::StandstillMode::NORMAL;
};
