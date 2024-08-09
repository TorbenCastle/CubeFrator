/*
 * stepper.cpp
 *
 *  Created on: Aug 9, 2024
 *      Author: Torben
 */

#include "stepper.h"

Stepper::Stepper()
    : currentPosition(0),
      targetPosition(0),
      currentDirection(0),
      lastDirection(0),
      minInterval(0),
      maxInterval(0),
      currentInterval(0),
      maxAcceleration(0),
      microsteps(16),
      stepsPerRevolution(3200),
      nextStepTime(0),
      lastStepTime(0),
      accelProfile(AccelProfile::NONE),
      mode(StepperMode::STOPPED),
      lastMode(StepperMode::STOPPED),
      recalculationFlag(true),
      rotateFlag(false),
      activeFlag(false),
      homedFlag(false),
      reverseFlag(false),
      syncTargetMoveFlag(false) {}

// Set Functions
void Stepper::setTargetPosition(int16_t position) {
    targetPosition = position;
}

void Stepper::setCurrentPosition(int16_t position) {
    currentPosition = position;
}

void Stepper::setDirection(uint8_t direction) {
    currentDirection = direction;
}

void Stepper::setMinInterval(uint32_t interval) {
    minInterval = interval;
}

void Stepper::setMaxInterval(uint32_t interval) {
    maxInterval = interval;
}

void Stepper::setMaxAcceleration(float acceleration) {
    maxAcceleration = acceleration;
}

void Stepper::setMicrosteps(uint8_t steps) {
    microsteps = steps;
}

void Stepper::setStepsPerRevolution(uint16_t steps) {
    stepsPerRevolution = steps;
}

void Stepper::setNextStepTime(uint32_t time) {
    nextStepTime = time;
}

void Stepper::setLastStepTime(uint32_t time) {
    lastStepTime = time;
}

void Stepper::setAccelProfile(AccelProfile profile) {
    accelProfile = profile;
}

void Stepper::setMode(StepperMode mode) {
    this->mode = mode;
}

// Get Functions
int16_t Stepper::getCurrentPosition() const {
    return currentPosition;
}

int16_t Stepper::getTargetPosition() const {
    return targetPosition;
}

uint8_t Stepper::getCurrentDirection() const {
    return currentDirection;
}

uint32_t Stepper::getMinInterval() const {
    return minInterval;
}

uint32_t Stepper::getMaxInterval() const {
    return maxInterval;
}

float Stepper::getMaxAcceleration() const {
    return maxAcceleration;
}

uint8_t Stepper::getMicrosteps() const {
    return microsteps;
}

uint16_t Stepper::getStepsPerRevolution() const {
    return stepsPerRevolution;
}

uint32_t Stepper::getNextStepTime() const {
    return nextStepTime;
}

uint32_t Stepper::getLastStepTime() const {
    return lastStepTime;
}

Stepper::AccelProfile Stepper::getAccelProfile() const {
    return accelProfile;
}

Stepper::StepperMode Stepper::getMode() const {
    return mode;
}

// Control Booleans
bool Stepper::recalculation() const {
    return recalculationFlag;
}

bool Stepper::isRotate() const {
    return rotateFlag;
}

bool Stepper::isActive() const {
    return activeFlag;
}

bool Stepper::isHomed() const {
    return homedFlag;
}

bool Stepper::isReverse() const {
    return reverseFlag;
}

bool Stepper::isSyncTargetMove() const {
    return syncTargetMoveFlag;
}
