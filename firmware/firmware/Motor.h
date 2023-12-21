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

#include <math.h>
#include <AccelStepper.h>
#include <TMC2209.h>
#include <ArduinoJson.h>

#define PIN_DIR 6
#define PIN_STEP 7
#define PIN_TX 8
#define PIN_RX 9

typedef uint32_t step_t;

class Motor {
    public:
        static Motor &i();
        void update();
        step_t currentSteps();
        step_t targetSteps();
        void setRunCurrent(uint8_t runCurrent);
        void setHoldCurrent(uint8_t m_holdCurrent);
        uint8_t runCurrent();
        uint8_t holdCurrent();
        void setMicrostepping(uint8_t stepping);
        uint8_t microstepping();
        void setInverted(bool inverted);
        bool isInverted();
        TMC2209::StandstillMode standStillMode();
        void setStandStillMode(TMC2209::StandstillMode mode);
        uint16_t speed();
        void setSpeed(uint16_t speed);
        uint16_t accel();
        void setAccel(uint16_t accel);
        void state(char *buff, size_t buffSize);
        void syncSteps(step_t steps);
        bool stealthChop();
        void setStealthChop(bool enabled);
        bool coolStep();
        void setCoolStep(bool enabled);
        void stop();
        void setTargetSteps(step_t steps);
		void setMaxSteps(step_t steps);
		step_t maxSteps();
		void setBacklash(step_t steps);
		step_t backlash();
		bool backlashEnabled();
		void setBacklashEnabled(bool enabled);

    private:
        Motor();
        ~Motor();
        Motor(const Motor &);
        Motor& operator=(const Motor&);

        uint8_t m_runCurrent = 50;
        uint8_t m_holdCurrent = 10;
        bool m_stealthChop = false;
        bool m_coolStep = true;
        bool m_invert = false;
        uint16_t m_speed = 500;
        uint16_t m_accel = 200;
        TMC2209::StandstillMode m_standStillMode = TMC2209::StandstillMode::NORMAL;

        uint8_t m_microStepping = 16;
		step_t m_maxSteps = 10000;
		step_t m_backlash = 0;
		bool m_backlashEnabled = false;

        SoftwareSerial *m_serial = nullptr;
        AccelStepper *m_stepper = nullptr;
        TMC2209 m_driver;

};
