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

// G2 — Launch detection, sized for this motor/airframe (F-15 @ 0.9 kg:
// peak accel reading ~2.9 g, sustained ~1.6 g). Threshold sits between the
// pad-rest ceiling (1.10 g) and the worst-case peak (~2.45 g, weak motor).
// Only needs to hold for a short bump-rejection window — the altitude gain
// check below is what actually confirms a real launch.
#define LAUNCH_ACCEL_THRESHOLD_G   1.8f
#define LAUNCH_ACCEL_MS            100    // accel must hold this long (bump rejection)

// G2b — Altitude confirmation: a real launch must also show altitude gain,
// not just an accel spike (rules out a bump/knock while sitting on the pad).
#define LAUNCH_ALT_DELTA_M         2.0f    // m AGL gain required, measured from the launch baseline (pad_rest_baseline_alt_m)
#define LAUNCH_CONFIRM_MS          1000    // total window (from accel latch start) to see that gain

// POWERED → COAST: motor burnout
#define BURNOUT_ACCEL_THRESHOLD_G  0.5f

// G1 — Pad-rest precondition (must latch before launch detection is armed)
#define PAD_REST_MS                2000   // ms of continuous pad-rest
#define PAD_REST_ACCEL_LOW_G       0.90f
#define PAD_REST_ACCEL_HIGH_G      1.10f
#define PAD_REST_GYRO_DPS          5.0f
#define PAD_REST_ACCEL_UP_G        0.85f  // vehicle must be vertical

// G3 — Flight-proof velocity latch. All simulated cases (weak motor to
// heavy) peak between 16 and 24 m/s, so 25 was unreachable — set below
// the weakest case with margin.
#define MIN_FLIGHT_VELOCITY_MS     10.0f  // m/s — unreachable on the ground

// G4 — Minimum coast time before apogee gates are allowed to fire, so a
// noisy velocity/altitude reading right at burnout can't look like apogee.
#define COAST_APOGEE_MIN_MS        500

// G5 — Altitude lockout for main deploy. NOTE: the real gate lives in
// pyro.h's PYRO_MAIN_MIN_ALT_M — this build is single-deploy, so that
// floor is 0 (ground protection comes from the launch-confirm gates
// above, not an altitude floor on the apogee charge).

// G6 — Independent apogee backstop: fire on altitude drop from peak, not
// just on velocity crossing zero, in case the integrated-velocity
// estimate is noisy or the IMU has faulted (see fsm_update()'s IMU-fault
// handling — TVC gets disabled in flight, but apogee detection must not).
#define APOGEE_BARO_DROP_M              3.0f

// G7 — Timeout backstop, measured from LAUNCH (powered_entry_ms), not
// from COAST entry — a short/weak flight can land before an
// entry-relative timeout would ever fire.
#define APOGEE_TIMEOUT_FROM_LAUNCH_MS   7000

// Landing detection
#define LANDED_ACCEL_LOW_G         0.75f
#define LANDED_ACCEL_HIGH_G        1.35f
#define LANDED_GYRO_THRESHOLD_DPS  10.0f
#define LANDED_TIME_MS             5000

// --------------------------------------------------------
// STATES
// --------------------------------------------------------

typedef enum {
    STATE_IDLE          = 0,   // on the pad, disarmed (SW401 open)
    STATE_ARMED         = 1,   // on the pad, armed (SW401 closed) — see fsm_set_armed()
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
    uint32_t powered_entry_ms;  // time POWERED was entered; also the G7 timeout reference

    float    prev_velocity_ms;
    float    peak_velocity_ms;  // G3: max velocity seen in POWERED+COAST
    float    peak_altitude_m;   // G6: max altitude seen in POWERED+COAST, for the baro-drop backstop

    bool     drogue_fired;
    bool     main_fired;
    bool     tvc_enabled;

    // G1 — pad-rest latch (evaluated in IDLE; once true, gates launch
    // detection directly — no arm step in between)
    bool     pad_rest_satisfied;
    uint32_t pad_rest_start_ms;
    float    pad_rest_baseline_alt_m;  // launch baseline: the armed ground reference once captured, else the altitude snapshot at pad-rest latch
    bool     baseline_from_arm;        // set by fsm_set_launch_baseline(); pad-rest latches stop overwriting the baseline

    // Fault flags
    bool     imu_fault;
} FlightSM;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void fsm_init(FlightSM *fsm);

/**
 * Update state machine. Call every loop iteration.
 * @param altitude_m  Estimated altitude AGL [m] — used for the apogee baro-drop backstop.
 */
void fsm_update(FlightSM *fsm,
                float accel_up_g,
                float velocity_ms,
                float accel_mag_g,
                float gyro_rate_dps,
                float altitude_m,
                bool  imu_valid);

/**
 * Reflect the arming switch (SW401) into the FSM. Only transitions
 * IDLE <-> ARMED — a no-op once launch detection has latched or flight
 * has begun. The switch is a hardware interlock (it cuts PYRO PWR); this
 * is display/logging only and must never gate whether a pyro can fire —
 * see main_control_loop.cpp's arming-switch handling for the full policy.
 */
void fsm_set_armed(FlightSM *fsm, bool armed);

/**
 * Lock the launch baseline to the ground reference captured after the
 * SW401 arming edge (ALT_GROUND_SAMPLES averaged readings). Later pad-rest
 * latches keep it instead of taking a fresh single-tick snapshot, until
 * the switch is opened again. Only call while still on the pad.
 * @param altitude_m  Estimated altitude at the new reference (0 right after alt_set_ground()).
 */
void fsm_set_launch_baseline(FlightSM *fsm, float altitude_m);

/**
 * Replace the pad-rest altitude snapshot — call after alt_calibrate_finish()
 * re-syncs the altitude estimate on the pad-rest latch, so the baseline
 * matches the re-synced altitude rather than the pre-calibration one.
 * No-op once the armed ground reference is locked (fsm_set_launch_baseline()).
 */
void fsm_set_pad_rest_baseline(FlightSM *fsm, float altitude_m);

/** Emergency abort — safes all outputs, sets STATE_ABORT. Pad-side faults only. */
void fsm_abort(FlightSM *fsm);

/** Returns true on the first call after a state transition (one-shot). */
bool fsm_state_changed(FlightSM *fsm);

/** How long the FSM has been in the current state [ms]. */
uint32_t fsm_time_in_state(const FlightSM *fsm);
