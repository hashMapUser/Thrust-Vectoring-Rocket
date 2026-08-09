#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pyro.h"

// ============================================================
// VERTICAL AXIS CONVENTION
// ============================================================
// All vertical-axis inputs use "positive = up" semantics.
//   accel_up_g  ≈ +1.0 g at rest, >> threshold during boost.
//   velocity_ms : integrated vertical velocity [m/s], + = up.
// Caller maps physical IMU axis before calling fsm_update():
//   const float accel_up_g = -imu.accel_x_g();
// ============================================================

// --------------------------------------------------------
// STATE TRANSITION THRESHOLDS
// --------------------------------------------------------

// G2 — Harder launch detection (was 2.5 g / 100 ms)
#define LAUNCH_ACCEL_THRESHOLD_G   4.0f   // model motors deliver 5-15 g; 4 g has margin
#define LAUNCH_ACCEL_MS            200    // sustained hold time

// POWERED → COAST: motor burnout
#define BURNOUT_ACCEL_THRESHOLD_G  0.5f

// G1 — Pad-rest precondition (must latch before ARMED → POWERED is allowed)
#define PAD_REST_MS                2000   // ms of continuous pad-rest
#define PAD_REST_ACCEL_LOW_G       0.90f
#define PAD_REST_ACCEL_HIGH_G      1.10f
#define PAD_REST_GYRO_DPS          5.0f
#define PAD_REST_ACCEL_UP_G        0.85f  // vehicle must be vertical

// G3 — Flight-proof velocity latch
#define MIN_FLIGHT_VELOCITY_MS     25.0f  // m/s — unreachable on the ground

// G4 — Minimum flight time from POWERED entry
#define MIN_FLIGHT_TIME_MS         2500
#define COAST_APOGEE_MIN_MS        1500   // minimum coast time (was 1000)

// G5 — Altitude lockout for main deploy
#define PYRO_MAIN_MIN_ALT_M        50.0f  // m AGL (real altitude required)

// G7 — Timeout backstop (only fires if G3 + G4 are also satisfied)
#define APOGEE_TIMEOUT_MS          8000

// Landing detection
#define LANDED_ACCEL_LOW_G         0.75f
#define LANDED_ACCEL_HIGH_G        1.35f
#define LANDED_GYRO_THRESHOLD_DPS  10.0f
#define LANDED_TIME_MS             5000

// ARM_SENSE threshold: 1.3 V on 3.3 V 12-bit ADC = ~1614 counts
#define ARM_SENSE_THRESHOLD        1614

// --------------------------------------------------------
// STATES
// --------------------------------------------------------

typedef enum {
    STATE_IDLE          = 0,
    STATE_ARMED         = 1,
    STATE_POWERED       = 2,
    STATE_COAST         = 3,
    STATE_APOGEE        = 4,
    STATE_DESCENT       = 5,
    STATE_MAIN          = 6,
    STATE_LANDED        = 7,
    STATE_ABORT         = 8,
} FlightState;

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

    uint32_t state_entry_ms;
    uint32_t launch_detect_ms;
    uint32_t powered_entry_ms;  // G4: time POWERED was entered

    float    prev_velocity_ms;
    float    peak_velocity_ms;  // G3: max velocity seen in POWERED+COAST

    bool     drogue_fired;
    bool     main_fired;
    bool     tvc_enabled;

    // G1 — pad-rest latch
    bool     pad_rest_satisfied;
    uint32_t pad_rest_start_ms;

    // Fault flags
    bool     imu_fault;
} FlightSM;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void fsm_init(FlightSM *fsm);

/**
 * Update state machine. Call every loop iteration.
 * @param altitude_m  Estimated altitude AGL [m] — used for G5 main deploy gate.
 */
void fsm_update(FlightSM *fsm,
                float accel_up_g,
                float velocity_ms,
                float accel_mag_g,
                float gyro_rate_dps,
                float altitude_m,
                bool  imu_valid);

/**
 * Arm. Only valid from STATE_IDLE.
 * @param arm_sense_ok  true if PIN_ARM_SENSE analog read confirms pyro power present.
 * @return true if arm accepted.
 */
bool fsm_arm(FlightSM *fsm, PyroState *pyro, bool arm_sense_ok);

/** Disarm — valid from ARMED only. */
void fsm_disarm(FlightSM *fsm, PyroState *pyro);

/** Emergency abort — safes all outputs, sets STATE_ABORT. */
void fsm_abort(FlightSM *fsm);

/** Returns true on the first call after a state transition (one-shot). */
bool fsm_state_changed(FlightSM *fsm);

/** How long the FSM has been in the current state [ms]. */
uint32_t fsm_time_in_state(const FlightSM *fsm);
