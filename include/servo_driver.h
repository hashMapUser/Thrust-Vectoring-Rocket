#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board_pins.h"

// PIN_SERVO_X (5) and PIN_SERVO_Y (6) come from board_pins.h.
// Both pins are on separate FlexPWM submodules.

// --------------------------------------------------------
// SERVO CONFIG
// --------------------------------------------------------

// Per-axis PWM pulse widths [microseconds]. CENTER_US is the verified
// neutral/trim position and is NOT to be moved to make the math convenient
// — it stays fixed at whatever the physical zero-deflection point is.
// MIN_US/MAX_US were re-measured hard stops as of 2026-09-13; they were
// asymmetric around CENTER_US, so the farther side on each axis has been
// pulled in to match the closer (more constraining) side, trading away
// some of that axis's extra travel to keep a single symmetric
// MAX_ANGLE_DEG safe in both directions. Pitch and yaw are on separate
// linkages with different spans — do not collapse these into shared
// constants.
#define SERVO_PITCH_MIN_US      975    // pulled in from 625 to match the 225us max-side stop
#define SERVO_PITCH_MAX_US      1425   // true stop — closer side, unchanged
#define SERVO_PITCH_CENTER_US   1200   // fixed — do not change

#define SERVO_YAW_MIN_US        700    // true stop — closer side, unchanged
#define SERVO_YAW_MAX_US        1650   // pulled in from 1675 to match the 475us min-side stop
#define SERVO_YAW_CENTER_US     1175   // fixed — do not change

// Maximum TVC deflection [degrees] — bench-confirmed with a protractor
// at the current MIN_US/MAX_US stops, 2026-09-13.
#define SERVO_PITCH_MAX_ANGLE_DEG  40.5f
#define SERVO_YAW_MAX_ANGLE_DEG    85.5f

// PWM update rate
#define SERVO_PWM_HZ         200

// Direction invert flags — re-confirmed via bench direction test (M4) on
// the current gimbal, 2026-09-13: positive pitch/yaw command still
// produces a RESTORING nozzle deflection on both axes after the rework.
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
