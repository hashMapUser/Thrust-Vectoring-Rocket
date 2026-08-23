#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board_pins.h"

// PIN_SERVO_X (5) and PIN_SERVO_Y (6) come from board_pins.h.
// Both pins are on separate FlexPWM submodules.

// --------------------------------------------------------
// SERVO CONFIG
// --------------------------------------------------------

// Per-axis PWM pulse widths [microseconds] — from bench range test (M3),
// 2026-08-23. Pitch and yaw are on separate linkages and are NOT symmetric
// around the same center — do not collapse these back into shared constants.
#define SERVO_PITCH_MIN_US      1100
#define SERVO_PITCH_MAX_US      1900
#define SERVO_PITCH_CENTER_US   1500

#define SERVO_YAW_MIN_US        1175
#define SERVO_YAW_MAX_US        1575
#define SERVO_YAW_CENTER_US     1375

// Maximum TVC deflection [degrees] — from bench measurement (M3), assuming
// 180° servos on both axes.
#define SERVO_PITCH_MAX_ANGLE_DEG  72.0f
#define SERVO_YAW_MAX_ANGLE_DEG    36.0f

// PWM update rate
#define SERVO_PWM_HZ         200

// Direction invert flags — confirmed via bench direction test (M4),
// 2026-08-23: positive pitch/yaw command produces a RESTORING nozzle
// deflection on both axes.
// 0 = natural direction, 1 = invert (negate command before sending).
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
