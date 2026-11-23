#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board_pins.h"

// PIN_SERVO_X (5) and PIN_SERVO_Y (6) come from board_pins.h.

// Which board output steers which axis. Bench test I, 2026-11-19: the
// servo on SERVO Y (pin 6) swings the nozzle along Z, so it's pitch; the
// one on SERVO X (pin 5) swings it along X, so it's yaw. The limits and
// centre for each axis below belong to the servo on that output — if this
// mapping ever changes, move those numbers with it. Don't fix a wrong-axis
// gimbal by swapping the servo plugs: the limits would then be on the
// wrong servo.
#define SERVO_PITCH_PIN   PIN_SERVO_Y
#define SERVO_YAW_PIN     PIN_SERVO_X

// --------------------------------------------------------
// SERVO CONFIG
// --------------------------------------------------------

// Per-axis PWM pulse widths [microseconds]. CENTER_US is the verified
// neutral/trim position and is NOT to be moved to make the math convenient
// — it stays fixed at whatever the physical zero-deflection point is.
// MIN_US/MAX_US: hard stops re-measured 2026-11-19 with the servo_cal
// limit finder (src/servo_limit_finder.cpp). Each limit sits 10 us inside
// its stop so the servo isn't stalled against it at full deflection. The
// stops are asymmetric around CENTER_US, so the farther side on each axis
// is pulled in to match the closer (more constraining) side, because the
// driver uses one span per axis for both directions (checked below). Pitch
// and yaw are on separate linkages with different spans — do not collapse
// these into shared constants.
#define SERVO_PITCH_MIN_US      665    // pin 6 servo: stop at 525; pulled in to match the max side (±510us)
#define SERVO_PITCH_MAX_US      1685   // pin 6 servo: stop at 1695 (closer side), 10us margin
#define SERVO_PITCH_CENTER_US   1175   // pin 6 servo: fixed — do not change

#define SERVO_YAW_MIN_US        985    // pin 5 servo: stop at 625; pulled in to match the max side (±215us)
#define SERVO_YAW_MAX_US        1415   // pin 5 servo: stop at 1425 (closer side), 10us margin
#define SERVO_YAW_CENTER_US     1200   // pin 5 servo: fixed — do not change

#if SERVO_PITCH_MAX_US - SERVO_PITCH_CENTER_US != SERVO_PITCH_CENTER_US - SERVO_PITCH_MIN_US || \
    SERVO_YAW_MAX_US   - SERVO_YAW_CENTER_US   != SERVO_YAW_CENTER_US   - SERVO_YAW_MIN_US
#error "SERVO_*_MIN_US / MAX_US must be symmetric about CENTER_US"
#endif

// Travel each side of centre [us].
#define SERVO_PITCH_SPAN_US  (SERVO_PITCH_MAX_US - SERVO_PITCH_CENTER_US)
#define SERVO_YAW_SPAN_US    (SERVO_YAW_MAX_US - SERVO_YAW_CENTER_US)

// Nozzle deflection at full travel [deg], measured 2026-11-20: pitch swings
// the nozzle ±7° along Z, yaw ±10° along X. Commands are in nozzle degrees
// (the units tvc_sim tunes the gains in), and these are also the PID output
// limits, so the controller can use all of the travel.
#define SERVO_PITCH_MAX_ANGLE_DEG  7.0f
#define SERVO_YAW_MAX_ANGLE_DEG    10.0f

// Pulse width per degree of nozzle deflection (the linkage, taken as linear).
#define SERVO_PITCH_US_PER_DEG  (SERVO_PITCH_SPAN_US / SERVO_PITCH_MAX_ANGLE_DEG)   // 72.9
#define SERVO_YAW_US_PER_DEG    (SERVO_YAW_SPAN_US / SERVO_YAW_MAX_ANGLE_DEG)       // 21.5

// Direction invert flags: 0 = natural direction, 1 = negate the command.
// NOT yet checked since the pitch/yaw outputs were swapped (2026-11-19):
// run bench test I, and if the nozzle swings TOWARD the lean on an axis,
// flip that axis's flag.
#define SERVO_PITCH_INVERT   0
#define SERVO_YAW_INVERT     0

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
