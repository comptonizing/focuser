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



#include "Motor.h"
#include "Storage.h"

Motor::Motor() {
    m_serial = new SoftwareSerial(PIN_RX, PIN_TX);
    m_driver.setup(*m_serial);
    m_stepper = new AccelStepper(1, PIN_STEP, PIN_DIR);

    m_driver.setRunCurrent(m_runCurrent);
    m_driver.setHoldCurrent(m_holdCurrent);
    m_driver.setMicrostepsPerStep(m_microStepping);
    m_driver.setStandstillMode(m_standStillMode);
    m_driver.disableAnalogCurrentScaling();
    if ( m_coolStep ) {
        m_driver.enableCoolStep();
    } else {
        m_driver.disableCoolStep();
    }
    if ( m_stealthChop ) {
        m_driver.enableStealthChop();
    } else {
        m_driver.disableStealthChop();
    }
    m_driver.enable();
	m_enabled = true;

    m_stepper->setCurrentPosition(0);
    m_stepper->setSpeed(m_speed);
    m_stepper->setAcceleration(m_accel);
}

void Motor::update() {
	step_t positionOld = m_stepper->currentPosition();
  m_stepper->run(); // Does at MOST 1 step
	step_t positionNow = m_stepper->currentPosition();
	if ( m_lastMotion.position != positionNow ) {
		direction_t directionNow = positionNow > m_lastMotion.position ? MOTION_OUTWARD : MOTION_INWARD;
		if ( directionNow != m_lastMotion.direction ) {
			m_lastMotion.reversed = positionNow;
			m_lastMotion.direction = directionNow;
			m_backlashLeft = backlash() - m_backlashLeft;
		}
		m_lastMotion.position = positionNow;
	}

  // TODO no backlash correction for distances of 1 step relative to target.
  // That's because for some reason when reversing and only doing one step
  // no step is taken at all, stalling everything
	if ( positionOld != positionNow && abs(m_currentTarget - positionNow) > 0 && backlashEnabled() && m_backlashLeft > 0 ) {
		// Pretend the last step didn't actually happen
		m_stepper->setCurrentPositionForce(positionOld);
		m_lastMotion.position = positionOld;
		// Decrease the left backlash steps by 1
		m_backlashLeft--;
	}
	if ( ! m_stepper->isRunning() ) {
		if ( m_holdCurrent == 0 ) {
			disable();
		} else {
			enable();
		}
	}
	saveMotionStatus();
}

step_t Motor::currentSteps() {
	return m_stepper->currentPosition();
}

void Motor::setTargetSteps(step_t steps) {
	m_stopped = false;
	m_currentTarget = steps;
    m_stepper->moveTo(steps);
	if ( steps != m_stepper->currentPosition() ) {
		enable();
	}
}

step_t Motor::targetSteps() {
	return m_currentTarget;
}

void Motor::setRunCurrent(uint8_t runCurrent) {
    m_runCurrent = runCurrent;
    m_driver.setRunCurrent(m_runCurrent);
}

void Motor::setHoldCurrent(uint8_t holdCurrent) {
    m_holdCurrent = holdCurrent;
    m_driver.setHoldCurrent(m_holdCurrent);
}

uint8_t Motor::runCurrent() {
    return m_runCurrent;
}

uint8_t Motor::holdCurrent() {
    return m_holdCurrent;
}

void Motor::setMicrostepping(uint8_t stepping) {
	step_t oldSteps = m_stepper->currentPosition();
	step_t oldStepping = microstepping();
	step_t oldMax = maxSteps();
	step_t oldBacklash = backlash();

	float ratio = (float) stepping / (float) oldStepping;

	setMaxSteps( (step_t)  ( ratio * oldMax ));
	setBacklash( (step_t) ( ratio * oldBacklash ));
	m_stepper->setCurrentPosition( (step_t) ( ratio * oldSteps ));
	m_currentTarget = (step_t) (ratio * m_currentTarget);
	m_lastMotion.reversed = (step_t) (ratio * m_lastMotion.reversed);
	m_lastMotion.position = (step_t) (ratio * m_lastMotion.position);
	m_microStepping = stepping;
	m_driver.setMicrostepsPerStep(stepping);
	saveMotionStatus(true);
}

uint8_t Motor::microstepping() {
    return m_microStepping;
}

void Motor::setInverted(bool inverted) {
    if ( inverted ) {
        if ( ! m_invert ) {
            m_driver.enableInverseMotorDirection();
            m_stepper->setCurrentPosition(maxSteps() - m_stepper->currentPosition());
			m_currentTarget = maxSteps() - m_currentTarget;
			m_lastMotion.reversed = maxSteps() - m_lastMotion.reversed;
			m_lastMotion.direction = oppositeDirection(m_lastMotion.direction);
            m_invert = true;
        }
    } else {
        if ( m_invert ) {
            m_driver.disableInverseMotorDirection();
            m_stepper->setCurrentPosition(maxSteps() - m_stepper->currentPosition());
			m_currentTarget = maxSteps() - m_currentTarget;
			m_lastMotion.reversed = maxSteps() - m_lastMotion.reversed;
			m_lastMotion.direction = oppositeDirection(m_lastMotion.direction);
            m_invert = false;
        }
    }
	saveMotionStatus(true);
}

bool Motor::isInverted() {
    return m_invert;
}

TMC2209::StandstillMode Motor::standStillMode() {
    return m_standStillMode;
}

void Motor::setStandStillMode(TMC2209::StandstillMode mode) {
    m_standStillMode = mode;
    m_driver.setStandstillMode(m_standStillMode);
}

uint16_t Motor::speed() {
    return m_speed;
}

void Motor::setSpeed(uint16_t speed) {
    m_speed = speed;
    m_stepper->setMaxSpeed(m_speed);
}

uint16_t Motor::accel() {
    return m_accel;
}

void Motor::setAccel(uint16_t accel) {
    m_accel = accel;
    m_stepper->setAcceleration(m_accel);
}

Motor &Motor::i() {
    static Motor theInstance;
    return theInstance;
}

void Motor::state(char *buff, size_t buffSize) {
    // Make sure all parameters are current
    update();
    StaticJsonDocument<256> json;
    update();

    json[F("S")] = (step_t) currentSteps();
    update();
	json[F("B")] = (step_t) backlash();
	update();
	json[F("BE")] = backlashEnabled();
	update();
	json[F("MS")] = (step_t) maxSteps();
	update();
    json[F("RC")] = runCurrent();
    update();
    json[F("HC")] = holdCurrent();
    update();
    json[F("uS")] = microstepping();
    update();
    json[F("I")] = isInverted();
    update();
    json[F("SSM")] = standStillMode();
    update();
    json[F("SP")] = speed();
    update();
    json[F("AC")] = accel();
    update();
    json[F("MO")] = m_stepper->isRunning();
    update();
    json[F("CP")] = stealthChop();
    update();
    json[F("CS")] = coolStep();
	update();
	json[F("id")] = 3;
	update();
	json[F("T")] = m_stepper->targetPosition();

    update();

    serializeJson(json, buff, buffSize);

    update();
}

void Motor::syncSteps(step_t steps) {
	// TODO Set current m_lastMotion
    m_stepper->setCurrentPosition(steps);
	saveMotionStatus(true);
}

void Motor::stop() {
	m_stopped = true;
    m_stepper->stop();
}

bool Motor::stealthChop() {
    return m_stealthChop;
}

void Motor::setStealthChop(bool enabled) {
    m_stealthChop = enabled;
    if ( m_stealthChop ) {
        m_driver.enableStealthChop();
    } else {
        m_driver.disableStealthChop();
    }
}

bool Motor::coolStep() {
    return m_coolStep;
}

void Motor::setBacklash(step_t steps) {
	m_backlash = steps;
}

step_t Motor::backlash() {
	return m_backlash;
}

void Motor::setBacklashEnabled(bool enabled) {
	m_backlashEnabled = enabled;
}

bool Motor::backlashEnabled() {
	return m_backlashEnabled;
}

void Motor::setMaxSteps(step_t steps) {
  m_maxSteps = steps;
  if ( m_stepper->currentPosition() > steps ) {
	  m_stepper->setCurrentPosition(steps);
  }
}

step_t Motor::maxSteps() {
  return m_maxSteps;
}

void Motor::setCoolStep(bool enabled) {
    m_coolStep = enabled;
    if ( m_coolStep ) {
        m_driver.enableCoolStep();
    } else {
        m_driver.disableCoolStep();
    }
}

void Motor::setMotionStorage(uint16_t location) {
	m_motionStorage = location;
}

bool Motor::loadMotionStatus() {
	if ( m_motionStorage == 0 ) {
		return false;
	}
	uint16_t magic;
	uint16_t crc;
	motion_t motion;
	uint8_t data[sizeof(motion_t)];

	uint16_t addressMagic = m_motionStorage;
	Storage::readEEPROM(addressMagic, (uint8_t *) &magic, sizeof(magic));
	if ( magic != __motion_magic ) {
		return false;
	}
	uint16_t addressCRC = addressMagic + sizeof(magic);
	uint16_t addressData = addressCRC + sizeof(crc);

	Storage::readEEPROM(addressCRC, (uint8_t *) &crc, sizeof(crc));
	Storage::readEEPROM(addressData, (uint8_t *) &motion, sizeof(motion));

	if ( crc != Storage::crcCalc((uint8_t *) &motion, sizeof(motion)) ) {
		return false;
	}

	if ( motion.direction != MOTION_UNKNOWN &&
		 motion.direction != MOTION_INWARD &&
		 motion.direction != MOTION_OUTWARD ) {
		return false;
	}
	if ( motion.position < 0 || motion.position > m_maxSteps ) {
		return false;
	}
	if ( motion.reversed < 0 || motion.reversed > m_maxSteps ) {
		return false;
	}
	m_stepper->setCurrentPosition(motion.position);
	m_lastMotion = motion;
	m_lastSavedMotion = motion;
	return true;
}

void Motor::saveMotionStatus(bool force) {
	if ( m_stepper->isRunning() && ! force ) {
		return;
	}
	if ( m_motionStorage == 0 ) {
		return;
	}
	if ( (unsigned long)(millis() - m_lastStored) < m_storeInterval && ! force ) {
		return;
	}
	if ( m_lastMotion.position == m_lastSavedMotion.position &&
			m_lastMotion.reversed == m_lastSavedMotion.reversed &&
			m_lastMotion.direction == m_lastSavedMotion.direction &&
			! force ) {
		return;
	}

	uint16_t crc = Storage::crcCalc((uint8_t *) &m_lastMotion, sizeof(m_lastMotion));
	uint16_t addressMagic = m_motionStorage;
	uint16_t addressCRC = addressMagic + sizeof(__motion_magic);
	uint16_t addressData = addressCRC + sizeof(crc);
	Storage::writeEEPROM(addressMagic, (uint8_t *) &__motion_magic, sizeof(__motion_magic));
	Storage::writeEEPROM(addressCRC, (uint8_t *) &crc, sizeof(crc));
	Storage::writeEEPROM(addressData, (uint8_t *) &m_lastMotion, sizeof(m_lastMotion));

	m_lastSavedMotion = m_lastMotion;
	m_lastStored = millis();
}

Motor::direction_t Motor::oppositeDirection(direction_t direction) {
	switch (direction) {
		case MOTION_INWARD:
			return MOTION_OUTWARD;
		case MOTION_OUTWARD:
			return MOTION_INWARD;
		case MOTION_UNKNOWN:
			return MOTION_UNKNOWN;
		default:
			return MOTION_UNKNOWN;
	}
}

void Motor::enable() {
	if ( m_enabled ) {
		return;
	}
	m_driver.enable();
	m_enabled = true;
}

void Motor::disable() {
	if ( ! m_enabled ) {
		return;
	}
	m_driver.disable();
	m_enabled = false;
}

bool Motor::isEnabled() {
	return m_enabled;
}
