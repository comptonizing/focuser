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

Motor::Motor() {
    m_serial = new SoftwareSerial(PIN_RX, PIN_TX);
    m_driver.setup(*m_serial);
    m_stepper = new AccelStepper(1, PIN_STEP, PIN_DIR);

    //  m_driver.enableCoolStep();
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

    m_stepper->setCurrentPosition(0);
    m_stepper->setSpeed(m_speed);
    m_stepper->setAcceleration(m_accel);
}

void Motor::update() {
    m_stepper->run();
}

step_t Motor::currentSteps() {
    return m_stepper->currentPosition();
}

void Motor::setTargetSteps(step_t steps) {
    m_stepper->moveTo(steps);
}

step_t Motor::targetSteps() {
    return m_stepper->targetPosition();
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
	step_t oldSteps = currentSteps();
	step_t oldStepping = microstepping();
	step_t oldMax = maxSteps();
	step_t oldBacklasIn = backlashIn();
	step_t oldBacklashOut = backlashOut();

	float ratio = (float) stepping / (float) oldStepping;

	setMaxSteps( (step_t)  ( ratio * oldMax ));
	setBacklashIn( (step_t) ( ratio * oldBacklasIn ));
	setBacklashOut( (step_t) ( ratio * oldBacklashOut ));
	m_stepper->setCurrentPosition( (step_t) ( ratio * oldSteps ));
	m_microStepping = stepping;
	m_driver.setMicrostepsPerStep(stepping);
}

uint8_t Motor::microstepping() {
    return m_microStepping;
}

void Motor::setInverted(bool inverted) {
    if ( inverted ) {
        if ( ! m_invert ) {
            m_driver.enableInverseMotorDirection();
            m_stepper->setCurrentPosition(maxSteps() - m_stepper->currentPosition());
            step_t tmp = m_backlashIn;
            m_backlashIn = m_backlashOut;
            m_backlashOut = tmp;
            m_invert = true;
        }
    } else {
        if ( m_invert ) {
            m_driver.disableInverseMotorDirection();
            m_stepper->setCurrentPosition(maxSteps() - m_stepper->currentPosition());
            step_t tmp = m_backlashIn;
            m_backlashIn = m_backlashOut;
            m_backlashOut = tmp;
            m_invert = false;
        }
    }
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
	json[F("BI")] = (step_t) backlashIn();
	update();
	json[F("BO")] = (step_t) backlashOut();
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

    serializeJson(json, buff, buffSize);

    update();
}

void Motor::syncSteps(step_t steps) {
    m_stepper->setCurrentPosition(steps);
}

void Motor::stop() {
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

void Motor::setBacklashIn(step_t steps) {
	m_backlashIn = steps;
}

step_t Motor::backlashIn() {
	return m_backlashIn;
}

void Motor::setBacklashOut(step_t steps) {
	m_backlashOut = steps;
}

step_t Motor::backlashOut() {
	return m_backlashOut;
}

void Motor::setMaxSteps(step_t steps) {
  m_maxSteps = steps;
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
