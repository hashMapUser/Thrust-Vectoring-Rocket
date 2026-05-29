#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pyro.h"

// ============================================================
// VERTICAL AXIS CONVENTION
// ============================================================
// All vertical-axis inputs to this module — accel_up_g, velocity_ms —
// use "positive = up" semantics, independent of how the IMU is physically
// mounted in the airframe.
//
//   At rest on the pad:   accel_up_g ≈ +1.0 g
//   During boost:         accel_up_g ≈ +5…+8 g  (above LAUNCH threshold)
//   Coast / free fall:    accel_up_g ≈  0   g  (below BURNOUT threshold)
//   Descent on chute:     accel_up_g ≈ +1.0 g  (terminal velocity)
//
// The caller is responsible for mapping the physical IMU axis into
// this convention before calling fsm_update(). For example, on this
// vehicle the IMU's body-X axis points toward the tail (gravity acts
// as a positive vector along +X), so the caller does:
//
//     const float accel_up_g = -imu.accel_x_g();
//     fsm_update(&fsm, accel_up_g, vertical_velocity_ms, ...);
//
// vertical_velocity_ms is integrated by the caller from (accel_up_g - 1g)
// during POWERED and COAST states only. Barometer is not used this flight.
// ============================================================

// --------------------------------------------------------
// STATE TRANSITION THRESHOLDS
// --------------------------------------------------------

// IDLE → ARMED: auto-armed at startup. Serial 'D' disarms, 'X' aborts (emergency overrides).
// ARMED → POWERED: vertical accel exceeds this for LAUNCH_ACCEL_MS
#define LAUNCH_ACCEL_THRESHOLD_G   2.5f    // g — well above pad vibration
#define LAUNCH_ACCEL_MS            100     // must hold for this many ms

// POWERED → COAST: accel drops below this (motor burnout → free fall)
#define BURNOUT_ACCEL_THRESHOLD_G  0.5f    // g

// COAST → APOGEE: integrated vertical velocity crosses zero
// Guard prevents premature trigger immediately after burnout
#define APOGEE_TIMEOUT_MS          8000    // 8 s — max expected coast phase
#define COAST_APOGEE_MIN_MS        1000    // minimum coast time before apogee can trigger

// APOGEE → DESCENT: drogue fired on entering APOGEE
// DESCENT → MAIN: fixed timer from apogee — tune to expected drogue descent time
#define MAIN_DEPLOY_DELAY_MS       30000   // 30 s — adjust for flight profile

// MAIN → LANDED: accel magnitude near 1 g AND angular rate near zero
#define LANDED_ACCEL_LOW_G         0.75f   // g — lower bound of "at rest"
#define LANDED_ACCEL_HIGH_G        1.35f   // g — upper bound of "at rest"
#define LANDED_GYRO_THRESHOLD_DPS  10.0f   // deg/s — below this = not rotating
#define LANDED_TIME_MS             5000    // must hold for 5 s

// --------------------------------------------------------
// STATES
// --------------------------------------------------------

typedef enum {
    STATE_IDLE          = 0,  // powered, waiting for arm command
    STATE_ARMED         = 1,  // armed, TVC active, waiting for launch detect
    STATE_POWERED       = 2,  // motor burning, TVC active
    STATE_COAST         = 3,  // motor out, TVC off, coasting to apogee
    STATE_APOGEE        = 4,  // apogee detected, drogue fired
    STATE_DESCENT       = 5,  // descending under drogue
    STATE_MAIN          = 6,  // main chute deployed
    STATE_LANDED        = 7,  // on the ground
    STATE_ABORT         = 8,  // fault detected — safe all outputs
} FlightState;

// Human-readable state names for logging
static const char* const STATE_NAMES[] = {
    "IDLE", "ARMED", "POWERED", "COAST",
    "APOGEE", "DESCENT", "MAIN", "LANDED", "ABORT"
};

// --------------------------------------------------------
// STATE MACHINE CONTEXT
// --------------------------------------------------------

typedef struct {
    FlightState state;
    FlightState prev_state;

    uint32_t state_entry_ms;    // millis() when current state was entered
    uint32_t launch_detect_ms;  // millis() when launch accel first exceeded threshold

    float    prev_velocity_ms;  // previous vertical velocity — zero crossing = apogee

    bool     drogue_fired;
    bool     main_fired;
    bool     tvc_enabled;

    // Fault flags
    bool     imu_fault;
} FlightSM;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise state machine to IDLE.
 */
void fsm_init(FlightSM *fsm);

/**
 * Update state machine with latest sensor estimates.
 * Call every loop iteration.
 *
 * @param fsm             State machine context.
 * @param accel_up_g      Vertical specific force [g] — see VERTICAL AXIS CONVENTION.
 *                        ≈ +1 g at rest, >> LAUNCH_ACCEL_THRESHOLD_G during boost.
 * @param velocity_ms     Integrated vertical velocity [m/s], positive going up.
 *                        Caller integrates from IMU accel (baro not used this flight).
 * @param accel_mag_g     Magnitude of the 3-axis accel vector [g]. Used for landed detect.
 * @param gyro_rate_dps   Magnitude of the 3-axis gyro vector [deg/s]. Used for landed detect.
 * @param imu_valid       IMU read succeeded this cycle.
 */
void fsm_update(FlightSM *fsm,
                float accel_up_g,
                float velocity_ms,
                float accel_mag_g,
                float gyro_rate_dps,
                bool  imu_valid);

/**
 * Arm the flight computer. Only valid from STATE_IDLE.
 * Arms pyro channels unconditionally (continuity check removed for this flight).
 * @return true if arm accepted; false if not in IDLE.
 */
bool fsm_arm(FlightSM *fsm, PyroState *pyro);

/**
 * Disarm / return to IDLE. Valid from ARMED only.
 * Calls pyro_disarm() to safe pyro outputs.
 */
void fsm_disarm(FlightSM *fsm, PyroState *pyro);

/**
 * Manually trigger abort — safes all outputs, sets STATE_ABORT.
 */
void fsm_abort(FlightSM *fsm);

/**
 * Returns true on the first call after a state transition.
 * Use to trigger one-shot actions (fire pyro, enable TVC, etc.)
 */
bool fsm_state_changed(FlightSM *fsm);

/**
 * How long has the FSM been in the current state [ms].
 */
uint32_t fsm_time_in_state(const FlightSM *fsm);
