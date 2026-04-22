#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// SERVO CONFIG
// --------------------------------------------------------

// Teensy 4.0 PWM pins for TVC servos
#define SERVO_PITCH_PIN      3    // controls pitch axis (Y)
#define SERVO_YAW_PIN        4    // controls yaw axis (Z)

// Standard PWM pulse widths [microseconds]
// These are starting points — calibrate per-servo on the bench
#define SERVO_CENTER_US      1500  // neutral / center position
#define SERVO_MIN_US         1000  // full deflection one way
#define SERVO_MAX_US         2000  // full deflection other way

// Maximum TVC deflection angle [degrees]
// Physical limit of your gimbal — prevents linkage damage
#define SERVO_MAX_ANGLE_DEG  10.0f

// PWM update rate [Hz] — 50 Hz is standard for analog servos
// Digital servos can run at 200-300 Hz for faster response
#define SERVO_PWM_HZ         200

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise servo PWM outputs.
 * Centers both servos. Call once in setup().
 */
void servo_init();

/**
 * Command the pitch servo to a deflection angle.
 * Positive angle = pitch up.
 * Angle is clamped to ±SERVO_MAX_ANGLE_DEG.
 *
 * @param angle_deg  Desired deflection [degrees].
 */
void servo_set_pitch(float angle_deg);

/**
 * Command the yaw servo to a deflection angle.
 * Positive angle = yaw right.
 * Angle is clamped to ±SERVO_MAX_ANGLE_DEG.
 *
 * @param angle_deg  Desired deflection [degrees].
 */
void servo_set_yaw(float angle_deg);

/**
 * Return both servos to center position immediately.
 * Call on burnout, abort, or any fault condition.
 */
void servo_center();

/**
 * Disable servo PWM output entirely (de-energises servo).
 * Call after landing to prevent servo buzzing.
 */
void servo_disable();

/**
 * Returns current pitch servo pulse width [us] for logging.
 */
float servo_get_pitch_us();

/**
 * Returns current yaw servo pulse width [us] for logging.
 */
float servo_get_yaw_us();