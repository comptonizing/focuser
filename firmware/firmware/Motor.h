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
#include "AccelStepper.h"
#include <TMC2209.h>
#include <ArduinoJson.h>

#include "Storage.h"

#define PIN_DIR 6
#define PIN_STEP 7
#define PIN_TX 8
#define PIN_RX 9

static uint16_t __motion_magic = 0b0110100011001011; // by Eva <3

typedef int32_t step_t;

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
		void setMotionStorage(uint16_t location);
		bool loadMotionStatus();
		void enable();
		void disable();
		bool isEnabled();

		typedef enum : uint8_t {
			MOTION_INWARD = 0,
			MOTION_OUTWARD = 1,
			MOTION_UNKNOWN = 2
		} direction_t;

		typedef struct {
			step_t position;
			step_t reversed;
			direction_t direction;
		} motion_t;

    private:
        Motor();
        ~Motor();
        Motor(const Motor &);
        Motor& operator=(const Motor&);

		void saveMotionStatus(bool force = false);
		Motor::direction_t oppositeDirection(direction_t direction);

		bool m_stopped = false;
		step_t m_currentTarget = 0;

        uint8_t m_runCurrent = 50;
        uint8_t m_holdCurrent = 10;
        bool m_stealthChop = false;
        bool m_coolStep = true;
        bool m_invert = false;
        uint16_t m_speed = 500;
        uint16_t m_accel = 200;
        TMC2209::StandstillMode m_standStillMode = TMC2209::StandstillMode::NORMAL;
		bool m_enabled = false;

        uint8_t m_microStepping = 16;
		step_t m_maxSteps = 10000;
		step_t m_backlash = 0;
		bool m_backlashEnabled = false;

        SoftwareSerial *m_serial = nullptr;
        AccelStepper *m_stepper = nullptr;
        TMC2209 m_driver;

		uint16_t m_motionStorage = 0;
		uint32_t m_lastStored = 0;
		uint16_t m_storeInterval = 3000;

		motion_t m_lastMotion = { .position = 0, .reversed = 0, .direction = MOTION_UNKNOWN };
		motion_t m_lastSavedMotion = { .position = 0, .reversed = 0, .direction = MOTION_UNKNOWN };
		step_t m_backlashLeft = 0;
};
