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


#include "focuser.h"
#include <indifocuser.h>

static std::unique_ptr<FocuserPollux> rotatorPolluxDriver((new FocuserPollux()));

FocuserPollux::FocuserPollux() {
    setVersion(0, 1);
}

const char *FocuserPollux::getDefaultName() {
    return "Pollux Focuser";
}

bool FocuserPollux::loadConfig(bool silent, const char *property) {
    // Everything ignored here, config comes from the devices itself
    (void) silent;
    (void) property;
    return true;
}

bool FocuserPollux::saveConfigItems(FILE *fp) {
    (void) fp;
    return true;
}

bool FocuserPollux::initProperties() {
    INDI::Focuser::initProperties();
    SetCapability(
			 FOCUSER_CAN_ABS_MOVE |
			 FOCUSER_CAN_REL_MOVE |
			 FOCUSER_CAN_ABORT |
			 FOCUSER_CAN_REVERSE |
			 FOCUSER_CAN_SYNC |
			 FOCUSER_HAS_VARIABLE_SPEED |
			 FOCUSER_HAS_BACKLASH
            );
    addAuxControls();
    addDebugControl();
    addConfigurationControl();
    setDefaultPollingPeriod(500);
    addPollPeriodControl();

    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
    serialConnection->registerHandshake([&]() {
        return Handshake();
        });
    registerConnection(serialConnection);

    IUFillNumber(&MotorN[ACCEL], "ACCEL", "Acceleration", "%.0f", 1, 4000, 1000, 1);
    IUFillNumber(&MotorN[RUNCURRENT], "RUNCURRENT", "Run Current [%]", "%.0f", 1, 100, 1, 100);
    IUFillNumber(&MotorN[HOLDCURRENT], "HOLDCURRENT", "Hold Current [%]", "%.0f", 1, 100, 1, 100);
    IUFillNumberVector(&MotorNP, MotorN, 3, getDeviceName(), "MOTOR", "Motor Settings",
            MAIN_CONTROL_TAB, IP_RW, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&MicroSteppingS[MS1], "MS1", "1", ISS_ON);
    IUFillSwitch(&MicroSteppingS[MS2], "MS2", "2", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS4], "MS4", "4", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS8], "MS8", "8", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS16], "MS16", "16", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS32], "MS32", "32", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS64], "MS64", "64", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS128], "MS128", "128", ISS_OFF);
    IUFillSwitch(&MicroSteppingS[MS256], "MS256", "256", ISS_OFF);
    IUFillSwitchVector(&MicroSteppingSP, MicroSteppingS, 9, getDeviceName(),
            "MICROSTEPPING", "Microstepping", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&StandStillModeS[NORMAL], "NORMAL", "Normal", ISS_ON);
    IUFillSwitch(&StandStillModeS[FREEWHEELING], "FREEWHEELING", "Freewheeling", ISS_OFF);
    IUFillSwitch(&StandStillModeS[BREAKING], "BREAKING", "Breaking", ISS_OFF);
    IUFillSwitch(&StandStillModeS[STRONG_BREAKING], "STRONG_BREAKING", "Strong Breaking", ISS_OFF);
    IUFillSwitchVector(&StandStillModeSP, StandStillModeS, 4, getDeviceName(),
            "STANDSTILL", "Standstill Mode", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&StealthChopS[INDI_ENABLED], "ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&StealthChopS[INDI_DISABLED], "DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&StealthChopSP, StealthChopS, 2, getDeviceName(),
            "STEALTHCHOP", "Stealth Chop", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

    IUFillSwitch(&CoolStepS[INDI_ENABLED], "ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&CoolStepS[INDI_DISABLED], "DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&CoolStepSP, CoolStepS, 2, getDeviceName(),
            "COOLSTEP", "Cool Step", MAIN_CONTROL_TAB, IP_RW,
            ISR_1OFMANY, TIMEOUT, IPS_IDLE);

	FocusSpeedN[0].min = 1;
	FocusSpeedN[0].max = 10000;

	FocusAbsPosN[0].min = 0;
	FocusAbsPosN[0].max = 1000000;

	FocusMaxPosN[0].min = 1;
	FocusMaxPosN[0].max = 1000000;

	FocusBacklashN[0].min = 0;
	FocusBacklashN[0].max = 10000;

    return true;
}

bool FocuserPollux::updateProperties() {
    INDI::Focuser::updateProperties();
    if ( isConnected() ) {
        defineProperty(&MotorNP);
        defineProperty(&MicroSteppingSP);
        defineProperty(&StandStillModeSP);
        defineProperty(&StealthChopSP);
        defineProperty(&CoolStepSP);
    } else {
        deleteProperty(MotorNP.name);
        deleteProperty(MicroSteppingSP.name);
        deleteProperty(StandStillModeSP.name);
        deleteProperty(StealthChopSP.name);
        deleteProperty(CoolStepSP.name);
    }
    return true;
}

bool FocuserPollux::Handshake() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
  json data;
  try {
    data = json::parse(rsp);
  } catch (...) {
    LOG_ERROR("JSON parse error");
    return false;
  }
  if ( data.contains("Error") ) {
    LOGF_ERROR("Device error: %s", data["Error"].template get<std::string>().c_str());
    return false;
  }
  if ( ! data.contains("id") ) {
	  LOG_ERROR("Did not get ID field! (wrong device?)");
  }
  int id = data["id"].template get<int>();
  if ( id != 3 ) {
	  LOGF_ERROR("Got ID field %d which is not %d! (wrong device?)", id);
	  return false;
  }
  updateFromResponse(rsp);
  return true;
}

void FocuserPollux::TimerHit() {
  if ( isConnected() ) {
    update();
  }
  SetTimer(getCurrentPollingPeriod());
}

bool FocuserPollux::ReverseFocuser(bool enabled) {
    bool commandStatus;
    char rsp[RSPBUFF];
    if ( enabled ) {
        commandStatus = sendCommand("set invert on", rsp);
    } else {
        commandStatus = sendCommand("set invert off", rsp);
    }
    if ( ! commandStatus || ! updateFromResponse(rsp) ) {
        return false;
    }
    return true;
}

bool FocuserPollux::SyncFocuser(uint32_t ticks) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    snprintf(cmd, CMDBUFF, "sync %u", ticks);
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        return false;
    }
    return true;
}

bool FocuserPollux::AbortFocuser() {
    LOG_INFO("Aborting rotation");
    char cmd[] = "stop";
    char rsp[RSPBUFF];
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        return false;
    }
    return true;
}

bool FocuserPollux::SetFocuserSpeed(int speed) {
	char cmd[CMDBUFF];
	char rsp[RSPBUFF];
	snprintf(cmd, CMDBUFF, "set speed %u", speed);
	if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
		return false;
	}
	return true;
}

IPState FocuserPollux::MoveAbsFocuser(uint32_t targetTicks) {
	char cmd[CMDBUFF];
	char rsp[RSPBUFF];
	snprintf(cmd, CMDBUFF, "set target %u", targetTicks);
	if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
		return IPS_ALERT;
	}
	return IPS_BUSY;
}

IPState FocuserPollux::MoveRelFocuser(FocusDirection dir, uint32_t ticks) {
	char cmd[CMDBUFF];
	char rsp[RSPBUFF];
	int currentPosition = FocusAbsPosN[0].value;
	LOGF_INFO("Initial: %d", currentPosition);
	LOGF_INFO("Ticks: %d", ticks);
	int targetPosition;
	if ( dir == FOCUS_INWARD ) {
		targetPosition = currentPosition - ticks;
	} else {
		targetPosition = currentPosition + ticks;
	}
	LOGF_INFO("Target: %d", targetPosition);
	if ( targetPosition < 0 ) {
		targetPosition = 0;
		LOG_WARN("Target position truncated to 0");
	}
	if ( targetPosition > FocusMaxPosN[0].value ) {
		targetPosition = FocusMaxPosN[0].value;
		LOGF_WARN("Target position truncated to %d", FocusMaxPosN[0].value);
	}
	snprintf(cmd, CMDBUFF, "set target %d", targetPosition);
	if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
		return IPS_ALERT;
	}
	return IPS_BUSY;
}

bool FocuserPollux::SetFocuserMaxPosition(uint32_t ticks) {
	char cmd[CMDBUFF];
	char rsp[RSPBUFF];
	snprintf(cmd, CMDBUFF, "max %u", ticks);
	if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
		SyncPresets(ticks);
		FocusSyncN[0].max = ticks;
		FocusSyncN[0].min = 0;
		IUUpdateMinMax(&FocusSyncNP);
		return true;
	}
	return false;
}

bool FocuserPollux::SetFocuserBacklash(int32_t ticks) {
	char cmd[CMDBUFF];
	char rsp[RSPBUFF];
	if ( ticks < 0 ) {
		LOG_ERROR("Negative backlash is not supported!");
		return false;
	}
	snprintf(cmd, CMDBUFF, "set bl %d", ticks);
	if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
		return true;
	}
	return false;;
}

bool FocuserPollux::SetFocuserBacklashEnabled(bool enabled) {
	char cmd[CMDBUFF];
	char rsp[RSPBUFF];
	if ( enabled ) {
		strncpy(cmd, "set backlash on", CMDBUFF);
	} else {
		strncpy(cmd, "set backlash off", CMDBUFF);
	}
	if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
		return true;
	}
	return false;
}

uint16_t FocuserPollux::crc16_update(uint16_t crc, uint8_t a) {
  // Code from crc16.h from the Arduino core tools
  crc ^= a;
  for (int ii=0; ii<8; ii++) {
    if ( crc & 1 ) {
      crc = (crc >> 1) ^ 0xA001;
    } else {
      crc = (crc >> 1);
    }
  }
  return crc;
}

uint16_t FocuserPollux::crcCalc(const void *data, size_t n) {
  const uint8_t *ptr = static_cast<const uint8_t *>(data);
  uint16_t crc = 0;
  for (size_t ii=0; ii<n; ii++) {
    crc = crc16_update(crc, ptr[ii]);
  }
  return crc;
}

uint16_t FocuserPollux::crcCalc(const char *str) {
  return crcCalc(static_cast<const void *>(str), strlen(str));
}

void FocuserPollux::cmdCrc(const char *cmd, char *out) {
  uint16_t crc = crcCalc(cmd);
  int len = strlen(cmd);
  memcpy(out+1, cmd, len);
  out[0] = '#';
  out[len+1] = '\0';
  out[len+2] = ((char *) &crc)[0];
  if ( out[len+2] == '$' ) {
    out[len+2] = '1';
  }
  out[len+3] = ((char *) &crc)[1];
  if ( out[len+3] == '$' ) {
    out[len+3] = '1';
  }
  out[len+4] = '$';
  out[len+5] = '\0';
}

bool FocuserPollux::checkCrc(const char *rsp) {
  uint16_t crcGotten;
  size_t len = strlen(rsp);
  ((char *) &crcGotten)[0] = rsp[len+1];
  ((char *) &crcGotten)[1] = rsp[len+2];
  uint16_t crcCalculated = crcCalc((const void *) rsp, len);
  char *crcChar = (char *) &crcCalculated;
  if ( crcChar[0] == '$' ) {
    crcChar[0] = '1';
  }
  if ( crcChar[1] == '$' ) {
    crcChar[1] = '1';
  }
  if ( crcGotten == crcCalculated ) {
    return true;
  } else {
    LOG_ERROR("Checksum error");
    LOGF_ERROR("Message: %s", rsp);
    LOGF_ERROR("Checksum: %d (0x%02x 0x%02x), expected %d (0x%02x 0x%02x)",
    crcCalculated, ((unsigned char *) &crcCalculated)[0], ((unsigned char *) &crcCalculated)[1],
    crcGotten, ((unsigned char *) &crcGotten)[0], ((unsigned char *) &crcGotten)[1]);
    return false;
  }
}

bool FocuserPollux::sendCommand(const char *cmd, char *rsp) {
  LOGF_INFO("Sending command: %s", cmd);
  int nbytes_written = 0, nbytes_read = 0, rc = -1;
  int PortFD = serialConnection->getPortFD();
  LOGF_DEBUG("PortFD: %d", PortFD);
  char buff[CMDBUFF];
  char err[ERRBUFF];
  char rspBuff[RSPBUFF];
  cmdCrc(cmd, buff);

  tcflush(PortFD, TCIOFLUSH);
  if ( (rc = tty_write(PortFD, buff, strlen(buff)+4, &nbytes_written)) != TTY_OK ) {
    tty_error_msg(rc, err, ERRBUFF);
    LOGF_ERROR("Error writing command %s: %s", cmd, err);
    return false;
  }

  LOG_DEBUG("RCV");

  // Somtimes there's garbage on the line so read until the next #
  rspBuff[0] = '\0';
  while ( rspBuff[0] != '#' ) {
    if ( (rc = tty_read(PortFD, rspBuff, 1, TIMEOUT, &nbytes_read) ) != TTY_OK ) {
      tty_error_msg(rc, err, ERRBUFF);
      LOGF_ERROR("Error reading response: %s", err);
      return false;
    }
  }

  if ( (rc = tty_read_section(PortFD, rspBuff+1, '$', TIMEOUT, &nbytes_read)) != TTY_OK ) {
    tty_error_msg(rc, err, ERRBUFF);
    LOGF_ERROR("Error reading response: %s", err);
    return false;
  }

  memcpy(rsp, rspBuff+1, strlen(rspBuff)+2);
  if ( ! checkCrc(rsp) ) {
    return false;
  }
  LOGF_INFO("RSP: %s", rsp);
  return true;
}

void FocuserPollux::setReverse(const json& data) {
    try {
        bool inverted = data["I"].template get<bool>();
        FocusReverseS[INDI_ENABLED].s = inverted ? ISS_ON : ISS_OFF;
        FocusReverseS[INDI_DISABLED].s = inverted ? ISS_OFF : ISS_ON;
        FocusReverseSP.s = IPS_OK;
        IDSetSwitch(&FocusReverseSP, nullptr);
    } catch (...) {
        FocusReverseSP.s = IPS_ALERT;
        IDSetSwitch(&FocusReverseSP, nullptr);
        throw;
    }
}

void FocuserPollux::setStealthChop(const json& data) {
    try {
        bool enabled = data["CP"].template get<bool>();
        StealthChopS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
        StealthChopS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
        StealthChopSP.s = IPS_OK;
        IDSetSwitch(&StealthChopSP, nullptr);
    } catch (...) {
        StealthChopSP.s = IPS_ALERT;
        IDSetSwitch(&StealthChopSP, nullptr);
        throw;
    }
}

void FocuserPollux::setCoolStep(const json& data) {
    try {
        bool enabled = data["CS"].template get<bool>();
        CoolStepS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
        CoolStepS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
        CoolStepSP.s = IPS_OK;
        IDSetSwitch(&CoolStepSP, nullptr);
    } catch (...) {
        CoolStepSP.s = IPS_ALERT;
        IDSetSwitch(&CoolStepSP, nullptr);
        throw;
    }
}

void FocuserPollux::setMotor(const json& data) {
    try {
        MotorN[ACCEL].value = data["AC"].template get<uint64_t>();
        MotorN[RUNCURRENT].value = data["RC"].template get<uint64_t>();
        MotorN[HOLDCURRENT].value = data["HC"].template get<uint64_t>();
    	MotorNP.s = IPS_OK;
    	IDSetNumber(&MotorNP, nullptr);
    } catch (...) {
        MotorNP.s = IPS_ALERT;
        IDSetNumber(&MotorNP, nullptr);
        throw;
    }
}

void FocuserPollux::setCurrentSteps(const json& data) {
	try {
		int currentPosition	= data["S"].template get<uint64_t>();
		int targetPosition = data["T"].template get<uint64_t>();
		bool moving = data["MO"].template get<bool>();
		FocusAbsPosN[0].value = currentPosition;

		if ( currentPosition == targetPosition ) {
			FocusAbsPosNP.s = IPS_OK;
			FocusRelPosNP.s = IPS_OK;
		} else {
			if ( ! moving ) {
				FocusAbsPosNP.s = IPS_ALERT;
				FocusRelPosNP.s = IPS_ALERT;
			}
		}

		IDSetNumber(&FocusAbsPosNP, nullptr);
		IDSetNumber(&FocusRelPosNP, nullptr);
	} catch (...) {
		FocusAbsPosNP.s = IPS_ALERT;
		IDSetNumber(&FocusAbsPosNP, nullptr);
		throw;
	}
}

void FocuserPollux::setBacklash(const json& data) {
	try {
		FocusBacklashN[0].value = data["B"].template get<uint64_t>();
		FocusBacklashNP.s = IPS_OK;
		IDSetNumber(&FocusBacklashNP, nullptr);
	} catch (...) {
		FocusBacklashNP.s = IPS_ALERT;
		IDSetNumber(&FocusBacklashNP, nullptr);
		throw;
	}
}

void FocuserPollux::setBacklashEnabled(const json& data) {
	try {
		bool enabled = data["BE"].template get<bool>();
		FocusBacklashS[INDI_ENABLED].s = enabled ? ISS_ON : ISS_OFF;
		FocusBacklashS[INDI_DISABLED].s = enabled ? ISS_OFF : ISS_ON;
		FocusBacklashSP.s = IPS_OK;
		IDSetSwitch(&FocusBacklashSP, nullptr);
	} catch (...) {
		FocusBacklashSP.s = IPS_ALERT;
		IDSetSwitch(&FocusBacklashSP, nullptr);
		throw;
	}
}

void FocuserPollux::setMaxSteps(const json& data) {
	try {
		FocusMaxPosN[0].value = data["MS"].template get<uint64_t>();
		FocusMaxPosNP.s = IPS_OK;
		IDSetNumber(&FocusMaxPosNP, nullptr);
		SyncPresets(FocusMaxPosN[0].value);
		FocusSyncN[0].max = FocusMaxPosN[0].value;
		FocusSyncN[0].min = 0;
		IUUpdateMinMax(&FocusSyncNP);
	} catch (...) {
		FocusMaxPosNP.s = IPS_ALERT;
		IDSetNumber(&FocusMaxPosNP, nullptr);
		throw;
	}
}

void FocuserPollux::setSpeed(const json& data) {
	try {
		FocusSpeedN[0].value = data["SP"].template get<uint64_t>();
		FocusSpeedNP.s = IPS_OK;
		IDSetNumber(&FocusSpeedNP, nullptr);
	} catch (...) {
		FocusSpeedNP.s = IPS_ALERT;
		IDSetNumber(&FocusSpeedNP, nullptr);
		throw;
	}
}

void FocuserPollux::setMicrostepping(const json& data) {
    try {
        uint8_t ms = data["uS"].template get<uint64_t>();
        for (uint8_t ii=0; ii<9; ii++) {
            if ( ms & ( 0x1 << ii) ) {
                MicroSteppingS[ii].s = ISS_ON;
            } else {
                MicroSteppingS[ii].s = ISS_OFF;
            }
        }
    	MicroSteppingSP.s = IPS_OK;
    	IDSetSwitch(&MicroSteppingSP, nullptr);
    } catch (...) {
      MicroSteppingSP.s = IPS_ALERT;
      IDSetSwitch(&MicroSteppingSP, nullptr);
      throw;
    }
}

void FocuserPollux::setStandstillMode(const json& data) {
    try {
        uint8_t mode = data["SSM"].template get<uint64_t>();
        for (uint8_t ii=0; ii<4; ii++) {
            StandStillModeS[ii].s = mode == ii ? ISS_ON : ISS_OFF;
        }
    	StandStillModeSP.s = IPS_OK;
    	IDSetSwitch(&StandStillModeSP, nullptr);
    } catch (...) {
        StandStillModeSP.s = IPS_ALERT;
        IDSetSwitch(&StandStillModeSP, nullptr);
        throw;
    }
}

bool FocuserPollux::updateFromResponse(const char *rsp) {
  json data;
  try {
    data = json::parse(rsp);
  } catch (...) {
    LOGF_ERROR("Error parsing JSON: %s", rsp);
    return false;
  }
  if ( data.contains("Error") ) {
    LOGF_ERROR("Device error: %s", data["Error"].template get<std::string>().c_str());
    return false;
  }
  try {
	  setCurrentSteps(data);
	  setBacklash(data);
	  setBacklashEnabled(data);
	  setMaxSteps(data);
      setReverse(data);
	  setSpeed(data);
      setStealthChop(data);
      setCoolStep(data);
      setMotor(data);
      setMicrostepping(data);
      setStandstillMode(data);
  } catch (...) {
    LOG_ERROR("Could not decode values from device");
    return false;
  }
  return true;
}

bool FocuserPollux::update() {
  char cmd[] = "status";
  char rsp[RSPBUFF];
  if ( ! sendCommand(cmd, rsp) ) {
    return false;
  }
  return updateFromResponse(rsp);
}

bool FocuserPollux::processMotorNP(double *values, char *names[], int n) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    for (int ii=0; ii<n; ii++) {
        for (int jj=0; jj<n; jj++) {
            if ( strcmp(MotorN[ii].name, names[jj]) == 0 ) {
                if ( MotorN[ii].value != values[jj] ) {
                    if ( strcmp(names[jj], "ACCEL") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set accel %d", static_cast<int>(values[jj]));
                    }
                    if ( strcmp(names[jj], "RUNCURRENT") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set rc %d", static_cast<int>(values[jj]));
                    }
                    if ( strcmp(names[jj], "HOLDCURRENT") == 0 ) {
                        snprintf(cmd, CMDBUFF, "set hc %d", static_cast<int>(values[jj]));
                    }
                    if ( ! sendCommand(cmd, rsp) ) {
                        MotorNP.s = IPS_ALERT;
                        IDSetNumber(&MotorNP, nullptr);
                        return false;
                    }
                }
            }
        }
    }
    if ( ! update() ) {
        MotorNP.s = IPS_ALERT;
        IDSetNumber(&MotorNP, nullptr);
        return false;
    }
    MotorNP.s = IPS_OK;
    IDSetNumber(&MotorNP, nullptr);
    return true;
}

bool FocuserPollux::processMicroSteppingSP(ISState *states, char *names[], int n) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    int old = IUFindOnSwitchIndex(&MicroSteppingSP);
    IUUpdateSwitch(&MicroSteppingSP, states, names, n);
    int active = IUFindOnSwitchIndex(&MicroSteppingSP);
    snprintf(cmd, CMDBUFF, "set micro steps %d", (0x1 <<  active));
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        MicroSteppingS[active].s = ISS_OFF;
        MicroSteppingS[old].s = ISS_ON;
        MicroSteppingSP.s = IPS_ALERT;
        IDSetSwitch(&MicroSteppingSP, nullptr);
        return false;
    }
    MicroSteppingSP.s = IPS_OK;
    IDSetSwitch(&MicroSteppingSP, nullptr);
    return true;
}

bool FocuserPollux::processStandStillModeSP(ISState *states, char *names[], int n) {
    char cmd[CMDBUFF];
    char rsp[RSPBUFF];
    int old = IUFindOnSwitchIndex(&StandStillModeSP);
    IUUpdateSwitch(&StandStillModeSP, states, names, n);
    int active = IUFindOnSwitchIndex(&StandStillModeSP);
    snprintf(cmd, CMDBUFF, "set standstill %d", active);
    if ( ! sendCommand(cmd, rsp) || ! updateFromResponse(rsp) ) {
        StandStillModeS[active].s = ISS_OFF;
        StandStillModeS[old].s = ISS_ON;
        StandStillModeSP.s = IPS_ALERT;
        IDSetSwitch(&StandStillModeSP, nullptr);
        return false;
    }
    StandStillModeSP.s = IPS_OK;
    IDSetSwitch(&StandStillModeSP, nullptr);
    return true;
}

bool FocuserPollux::processStealthChopSP(ISState *states, char *names[], int n) {
    char rsp[RSPBUFF];
    bool commandStatus;
    int old = IUFindOnSwitchIndex(&StealthChopSP);
    IUUpdateSwitch(&StealthChopSP, states, names, n);
    int active = IUFindOnSwitchIndex(&StealthChopSP);
    if ( active == INDI_ENABLED ) {
        commandStatus = sendCommand("set stealthchop on", rsp);
    } else {
        commandStatus = sendCommand("set stealthchop off", rsp);
    }
    if ( ! commandStatus || ! updateFromResponse(rsp) ) {
        StealthChopS[active].s = ISS_OFF;
        StealthChopS[old].s = ISS_ON;
        StealthChopSP.s = IPS_ALERT;
        IDSetSwitch(&StealthChopSP, nullptr);
        return false;
    }
    StealthChopSP.s = IPS_OK;
    IDSetSwitch(&StealthChopSP, nullptr);
    return true;
}

bool FocuserPollux::processCoolStepSP(ISState *states, char *names[], int n) {
    char rsp[RSPBUFF];
    bool commandStatus;
    int old = IUFindOnSwitchIndex(&CoolStepSP);
    IUUpdateSwitch(&CoolStepSP, states, names, n);
    int active = IUFindOnSwitchIndex(&CoolStepSP);
    if ( active == INDI_ENABLED ) {
        commandStatus = sendCommand("set coolstep on", rsp);
    } else {
        commandStatus = sendCommand("set coolstep off", rsp);
    }
    if ( ! commandStatus || ! updateFromResponse(rsp) ) {
        CoolStepS[active].s = ISS_OFF;
        CoolStepS[old].s = ISS_ON;
        CoolStepSP.s = IPS_ALERT;
        IDSetSwitch(&CoolStepSP, nullptr);
        return false;
    }
    CoolStepSP.s = IPS_OK;
    IDSetSwitch(&CoolStepSP, nullptr);
    return true;
}

bool FocuserPollux::ISNewNumber(const char *dev, const char *name, double *values, char *names[], int n) {
    if (dev && !strcmp(dev, getDeviceName())) {
        if ( strcmp(name, MotorNP.name) == 0 ) {
            return processMotorNP(values, names, n);
        }
        return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
    }
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool FocuserPollux::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) {
    if (dev && !strcmp(dev, getDeviceName())) {
        if ( strcmp(name, MicroSteppingSP.name) == 0 ) {
            return processMicroSteppingSP(states, names, n);
        }
        if ( strcmp(name, StandStillModeSP.name) == 0 ) {
            return processStandStillModeSP(states, names, n);
        }
        if ( strcmp(name, StealthChopSP.name) == 0 ) {
            return processStealthChopSP(states, names, n);
        }
        if ( strcmp(name, CoolStepSP.name) == 0 ) {
            return processCoolStepSP(states, names, n);
        }
        return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
    }
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}
