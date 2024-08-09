/*
 * acceleration.c
 *
 *  Created on: Aug 9, 2024
 *      Author: Torben
 */


#include <stdint.h>
#include <math.h>

#define MAX_STEPS 2048
#define ACCEL_STEP_MAX 1024
#define MIN(a,b) ((a) < (b) ? (a) : (b))

typedef enum {
    ACCELERATING,
    CRUISING,
    DECELERATING,
    STOPPED
} MotorMode;

typedef struct {
    uint32_t step_array[MAX_STEPS];
    uint32_t accel_step_count;
    float max_acceleration;
} StepperMotor;

void linear_acceleration(StepperMotor* motor, int32_t current_dir, int32_t start_position,
                         int32_t current_target, int32_t current_position, float current_speed,
                         float target_speed, MotorMode moveing_mode, uint32_t accel_length,
                         float multiplier) {
    // Calculate distance to target and necessary acceleration steps
    int32_t distance_to_target = abs(current_target - current_position);
    uint32_t steps_to_accelerate = MIN(accel_length, ACCEL_STEP_MAX / 2);

    float step_increment = motor->max_acceleration * multiplier;

    for (uint32_t i = 0; i < steps_to_accelerate; i++) {
        motor->step_array[i] = (uint32_t)((1.0 / (current_speed + i * step_increment)) * 1000000); // microsecond timers
    }

    motor->accel_step_count = steps_to_accelerate;
}

void exponential_acceleration(StepperMotor* motor, int32_t current_dir, int32_t start_position,
                              int32_t current_target, int32_t current_position, float current_speed,
                              float target_speed, MotorMode moveing_mode, uint32_t accel_length,
                              float multiplier) {
    // Calculate distance to target and necessary acceleration steps
    int32_t distance_to_target = abs(current_target - current_position);
    uint32_t steps_to_accelerate = MIN(accel_length, ACCEL_STEP_MAX / 2);

    for (uint32_t i = 0; i < steps_to_accelerate; i++) {
        float step_factor = pow(2.0, (float)i / steps_to_accelerate) - 1.0;  // Exponential increase
        motor->step_array[i] = (uint32_t)((1.0 / (current_speed + step_factor * multiplier)) * 1000000); // microsecond timers
    }

    motor->accel_step_count = steps_to_accelerate;
}

void s_curve_acceleration(StepperMotor* motor, int32_t current_dir, int32_t start_position,
                          int32_t current_target, int32_t current_position, float current_speed,
                          float target_speed, MotorMode moveing_mode, uint32_t accel_length,
                          float multiplier) {
    // Calculate distance to target and necessary acceleration steps
    int32_t distance_to_target = abs(current_target - current_position);
    uint32_t steps_to_accelerate = MIN(accel_length, ACCEL_STEP_MAX / 2);

    for (uint32_t i = 0; i < steps_to_accelerate; i++) {
        float t = (float)i / steps_to_accelerate;
        float s_curve_value = t * t * (3 - 2 * t);  // S-curve function
        motor->step_array[i] = (uint32_t)((1.0 / (current_speed + s_curve_value * multiplier)) * 1000000); // microsecond timers
    }

    motor->accel_step_count = steps_to_accelerate;
}

void fast_stop(StepperMotor* motor) {
    motor->accel_step_count = 0;
    for (uint32_t i = 0; i < MAX_STEPS; i++) {
        motor->step_array[i] = 0; // Reset all timers to 0, stopping the motor
    }
}
