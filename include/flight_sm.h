#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// STATE TRANSITION THRESHOLDS
// --------------------------------------------------------

// IDLE → ARMED: triggered externally (button/serial command), not by sensor
// ARMED → POWERED_FLIGHT: vertical accel exceeds this for LAUNCH_ACCEL_MS
#define LAUNCH_ACCEL_THRESHOLD_G   2.5f    // g — well above pad vibration
#define LAUNCH_ACCEL_MS            100     // must hold for this many ms

// POWERED_FLIGHT → COAST: accel drops below this (motor burnout)
#define BURNOUT_ACCEL_THRESHOLD_G  0.5f    // g

// COAST → APOGEE: vertical velocity crosses zero (detected via alt estimator)
// Backup: altitude hasn't increased for APOGEE_TIMEOUT_MS
#define APOGEE_TIMEOUT_MS          8000    // 8 s — max expected coast phase

// APOGEE → DESCENT: triggered by apogee detection, fires drogue
// DESCENT → MAIN: altitude drops below this AGL
#define MAIN_DEPLOY_ALT_M          150.0f  // m AGL — adjust for your field

// MAIN → LANDED: vertical velocity near zero for LANDED_TIME_MS
#define LANDED_VEL_THRESHOLD_MS    0.5f    // m/s
#define LANDED_TIME_MS             3000    // must hold for 3 s

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

    float    max_altitude_m;    // peak altitude seen — used for apogee detection
    float    prev_velocity_ms;  // previous vertical velocity — zero crossing = apogee

    bool     drogue_fired;
    bool     main_fired;
    bool     tvc_enabled;

    // Fault flags
    bool     imu_fault;
    bool     baro_fault;
    bool     mag_fault;
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
 * @param fsm           State machine context.
 * @param accel_g       Vertical acceleration [g] (positive = up).
 * @param altitude_m    Estimated AGL altitude [m].
 * @param velocity_ms   Estimated vertical velocity [m/s].
 * @param imu_valid     IMU read succeeded this cycle.
 * @param baro_valid    Barometer read succeeded this cycle.
 */
void fsm_update(FlightSM *fsm,
                float accel_g,
                float altitude_m,
                float velocity_ms,
                bool  imu_valid,
                bool  baro_valid);

/**
 * Arm the flight computer. Only valid from STATE_IDLE.
 * @return true if arm was accepted.
 */
bool fsm_arm(FlightSM *fsm);

/**
 * Disarm / return to IDLE. Valid from ARMED only.
 */
void fsm_disarm(FlightSM *fsm);

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