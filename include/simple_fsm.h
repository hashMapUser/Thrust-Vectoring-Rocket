#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pyro.h"

// ============================================================
// SIMPLE FSM — finned (no-TVC), recovery-only flight logic
// ============================================================
// Deliberately minimal, and separate from flight_sm.h — no arm/disarm
// step, no pad-rest latch, no continuity gate, no PID/TVC. It watches
// exactly two conditions:
//
//   launch  — g-force spike AND altitude has climbed SIMPLE_LAUNCH_ALT_M
//             from the pad, at the same instant
//   apogee  — altitude has dropped SIMPLE_RECOVERY_DROP_M from its peak
//
// That is the whole flight logic. Use this for a finned flight where the
// full flight_sm.h machinery (auto-arm, continuity, TVC gating) is more
// than the flight needs.
//
// SAFETY — READ BEFORE POWERING WITH A LIVE E-MATCH CONNECTED:
// The caller arms the pyro channel once at boot (pyro_arm()) — there is
// no separate arm step and no way to leave it disarmed on the pad. The
// only things preventing an accidental fire are:
//   1. Needing a genuine ~3 m altitude gain simultaneous with a real
//      g-force spike to ever leave IDLE (a bump or knock moves the
//      sensor by millimetres, not metres — it can spike accel, but it
//      cannot fake a barometric altitude gain).
// Do not power this up with a live e-match connected unless you are at
// the pad, ready to fly. Use 'X' over serial to safe the pyro outputs
// at any time.
//
// pyro.cpp also persists a "fired" flag per channel across resets, so a
// brownout mid-flight can't re-fire a spent channel — but that means the
// flag must be cleared for each *new* flight. finned_control_loop.cpp
// does this on the arming switch's off->on edge on the pad; see its
// setup()/loop() for the full arming-switch handling.

// --------------------------------------------------------
// TUNING — check these against your own motor/airframe before flying
// --------------------------------------------------------

// Launch detection
#define SIMPLE_LAUNCH_ACCEL_G     3.0f   // g-force (vertical) to call it a launch
#define SIMPLE_LAUNCH_ALT_M       3.0f   // required altitude gain from the pad [m]

// Apogee / recovery detection
#define SIMPLE_RECOVERY_DROP_M    5.0f   // drop from peak altitude that means "coming down" [m]

// Landing detection — stops logging (see main loop) and sounds the locator
#define SIMPLE_LANDED_ACCEL_LOW_G   0.75f
#define SIMPLE_LANDED_ACCEL_HIGH_G  1.35f
#define SIMPLE_LANDED_GYRO_DPS      10.0f
#define SIMPLE_LANDED_HOLD_MS       5000

// --------------------------------------------------------
// STATES
// --------------------------------------------------------

typedef enum {
    SIMPLE_STATE_IDLE     = 0,   // on the pad, waiting for launch
    SIMPLE_STATE_ASCENT   = 1,   // launched, climbing, watching for apogee
    SIMPLE_STATE_RECOVERY = 2,   // chute fired, descending
    SIMPLE_STATE_LANDED   = 3,   // settled — logging stopped, locator sounding
} SimpleState;

extern const char * const SIMPLE_STATE_NAMES[4];

typedef struct {
    SimpleState state;
    SimpleState prev_state;
    uint32_t    state_entry_ms;

    float       ground_altitude_m;   // launch baseline: set at init, re-set by the SW401 arming capture
    float       peak_altitude_m;     // highest altitude seen this flight

    bool        chute_fired;
} SimpleFSM;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise the state machine. Call once after the altitude estimator
 * has a ground-pressure baseline.
 * @param ground_altitude_m  Current altitude estimate — becomes the
 *                            baseline SIMPLE_LAUNCH_ALT_M is measured from.
 */
void simple_fsm_init(SimpleFSM *fsm, float ground_altitude_m);

/**
 * Re-set the launch baseline after the altitude estimator is re-referenced
 * (the SW401 arming capture). Also resets the peak — one left over from the
 * old reference would read as an instant "drop" once ASCENT starts. No-op
 * outside IDLE.
 * @param ground_altitude_m  Estimated altitude at the new reference.
 */
void simple_fsm_set_ground(SimpleFSM *fsm, float ground_altitude_m);

/**
 * Update the state machine and fire the recovery chute when appropriate.
 * Call every loop iteration. The caller must have already called
 * pyro_arm(pyro) once at boot — see the safety note above.
 */
void simple_fsm_update(SimpleFSM *fsm, PyroState *pyro,
                       float accel_up_g, float accel_mag_g, float gyro_rate_dps,
                       float altitude_m, uint32_t now_ms);

/** Returns true on the first call after a state transition (one-shot). */
bool simple_fsm_state_changed(SimpleFSM *fsm);
