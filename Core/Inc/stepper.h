/*
 * stepper.h
 *
 *  Created on: Aug 9, 2024
 *      Author: Torben
 */
#ifndef STEPPER_H
#define STEPPER_H


#include "main.h"

class Stepper {
public:
    // Motor Modes
    enum class StepperMode : uint8_t {
        STOPPED,
        ACCELERATE,
        CRUISING,
        DECELERATE,
        DEBUG
    };

    // Acceleration Profiles
    enum class AccelProfile : uint8_t {
        LINEAR,
        EXPONENTIAL,
        S_CURVE,
        STOP,
        NONE
    };

    // Constructor
    Stepper();

    // Set and Get Functions
    void setTargetPosition(int16_t position);
    void setCurrentPosition(int16_t position);
    void setDirection(uint8_t direction);
    void setMinInterval(uint32_t interval);
    void setMaxInterval(uint32_t interval);
    void setMaxAcceleration(float acceleration);
    void setMicrosteps(uint8_t microsteps);
    void setStepsPerRevolution(uint16_t steps);
    void setNextStepTime(uint32_t time);
    void setLastStepTime(uint32_t time);
    void setAccelProfile(AccelProfile profile);
    void setMode(StepperMode mode);

    int16_t getCurrentPosition() const;
    int16_t getTargetPosition() const;
    uint8_t getCurrentDirection() const;
    uint32_t getMinInterval() const;
    uint32_t getMaxInterval() const;
    float getMaxAcceleration() const;
    uint8_t getMicrosteps() const;
    uint16_t getStepsPerRevolution() const;
    uint32_t getNextStepTime() const;
    uint32_t getLastStepTime() const;
    AccelProfile getAccelProfile() const;
    StepperMode getMode() const;

    // Control Booleans
    bool recalculation() const;
    bool isRotate() const;
    bool isActive() const;
    bool isHomed() const;
    bool isReverse() const;
    bool isSyncTargetMove() const;

private:
    // Member variables
    int16_t currentPosition;
    int16_t targetPosition;
    uint8_t currentDirection;
    uint8_t lastDirection;

    uint32_t minInterval;
    uint32_t maxInterval;
    uint32_t currentInterval;

    float maxAcceleration;
    uint8_t microsteps;
    uint16_t stepsPerRevolution;

    uint32_t nextStepTime;
    uint32_t lastStepTime;

    AccelProfile accelProfile;
    StepperMode mode;
    StepperMode lastMode;

    bool recalculationFlag;
    bool rotateFlag;
    bool activeFlag;
    bool homedFlag;
    bool reverseFlag;
    bool syncTargetMoveFlag;
};

#endif // STEPPER_H
