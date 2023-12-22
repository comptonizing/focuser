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

#include <string.h>
#include <termios.h>
#include <memory>

#include <indibase.h>
#include <indifocuser.h>
#include <connectionplugins/connectionserial.h>

#include "json.hpp"

#define CMDBUFF 1024
#define ERRBUFF 256
#define RSPBUFF 1024

#define TIMEOUT 3

using json = nlohmann::json;

namespace Connection {
  class Serial;
}

class FocuserPollux : public INDI::Focuser {
    public:
        FocuserPollux();
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n) override;
        virtual bool ISNewNumber(const char *dev, const char *name, double *values, char *names[], int n) override;
    protected:
        virtual const char *getDefaultName() override;
        virtual bool saveConfigItems(FILE *fp) override;
        virtual bool loadConfig(bool silent, const char *property) override;
        virtual void TimerHit() override;
		virtual bool ReverseFocuser(bool enabled) override;
		virtual bool SyncFocuser(uint32_t ticks) override;
		virtual bool AbortFocuser() override;
		virtual bool SetFocuserSpeed(int speed) override;
		virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
		virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
		virtual bool SetFocuserMaxPosition(uint32_t ticks) override;
		virtual bool SetFocuserBacklash(int32_t ticks) override;
		virtual bool SetFocuserBacklashEnabled(bool enabled) override;
     private:
        bool Handshake() override;
        bool sendCommand(const char *cmd, char *rsp);
        void cmdCrc(const char *cmd, char *out);
        bool checkCrc(const char *rsp);
        uint16_t crc16_update(uint16_t crc, uint8_t a);
        uint16_t crcCalc(const void *data, size_t n);
        uint16_t crcCalc(const char *str);
        bool updateFromResponse(const char *rsp);
        bool update();
        void setReverse(const json& data);
        void setStealthChop(const json& data);
        void setCoolStep(const json& data);
        void setMotor(const json& data);
		void setCurrentSteps(const json& data);
		void setBacklash(const json& data);
		void setBacklashEnabled(const json& data);
		void setMaxSteps(const json& data);
		void setSpeed(const json& data);
        void setMicrostepping(const json& data);
        void setStandstillMode(const json& data);
        bool processMotorNP(double *values, char *names[], int n);
        bool processMicroSteppingSP(ISState *states, char *names[], int n);
        bool processStandStillModeSP(ISState *states, char *names[], int n);
        bool processStealthChopSP(ISState *states, char *names[], int n);
        bool processCoolStepSP(ISState *sates, char *names[], int n);

        enum {
            ACCEL = 0,
            RUNCURRENT = 1,
            HOLDCURRENT = 2
        };
        INumber MotorN[3];
        INumberVectorProperty MotorNP;

        enum {
            MS1 = 0,
            MS2 = 1,
            MS4 = 2,
            MS8 = 3,
            MS16 = 4,
            MS32 = 5,
            MS64 = 6,
            MS128 = 7,
            MS256 = 8
        };
        ISwitch MicroSteppingS[9];
        ISwitchVectorProperty MicroSteppingSP;

        enum {
            NORMAL = 0,
            FREEWHEELING = 1,
            BREAKING = 2,
            STRONG_BREAKING = 3
        };
        ISwitch StandStillModeS[4];
        ISwitchVectorProperty StandStillModeSP;

        ISwitch StealthChopS[2];
        ISwitchVectorProperty StealthChopSP;

        ISwitch CoolStepS[2];
        ISwitchVectorProperty CoolStepSP;
};
