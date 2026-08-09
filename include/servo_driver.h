#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board_pins.h"

// PIN_SERVO_X (5) and PIN_SERVO_Y (6) come from board_pins.h.
// Both pins are on separate FlexPWM submodules.

// --------------------------------------------------------
// SERVO CONFIG
// --------------------------------------------------------

// Standard PWM pulse widths [microseconds] — calibrate on bench (M3)
#define SERVO_CENTER_US      1500
#define SERVO_MIN_US         1000
#define SERVO_MAX_US         2000

// Maximum TVC deflection [degrees] — set from bench measurement (M3)
// Placeholder: spec assumed ±7°. Update after servo_range_finder run.
#define SERVO_MAX_ANGLE_DEG  7.0f

// PWM update rate
#define SERVO_PWM_HZ         200

// Direction invert flags — set after bench direction test (M4).
// 0 = natural direction, 1 = invert (negate command before sending).
// Confirm positive pitch command produces a RESTORING nozzle deflection.
#define SERVO_X_INVERT       0
#define SERVO_Y_INVERT       0

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void servo_init();
void servo_set_pitch(float angle_deg);
void servo_set_yaw(float angle_deg);
void servo_center();
void servo_disable();
float servo_get_pitch_us();
float servo_get_yaw_us();
