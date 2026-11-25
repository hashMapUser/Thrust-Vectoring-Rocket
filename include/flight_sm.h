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

// G1 — Arming settle. The arming switch (SW401) is closed with the rocket
// already on the rail; launch detection is armed only once the switch has
// stayed closed this long, so the crew walking away, the igniter going in
// and the rail being touched can't count. Opening the switch disarms it,
// and the next close starts the wait again.
#define ARM_SETTLE_MS              30000
//
// The same window calibrates the accelerometer bias (main_control_loop.cpp):
// the rocket starts and ends it at rest, so the true vertical accel averages
// to exactly 1 g over it even if it was handled partway through — whatever
// is left over is sensor bias.

// G2 — Launch detection: a large AND sustained acceleration along the
// airframe. The reading is low-pass filtered first (LAUNCH_ACCEL_TAU_MS)
// so motor vibration, which shakes single 125 Hz samples by a g or more,
// can't break the hold. It must then stay above the threshold for the
// whole of LAUNCH_CONFIRM_MS — a knock is over long before that; only a
// burning motor keeps it up.
//
// Threshold vs the certified F-15 curve: its lowest sustained thrust
// (~13.0 N, late in the burn) is 1.71 g at 0.775 kg and 1.47 g at 0.90 kg,
// and the thrust spike carries it well above that through the first 0.9 s.
// 1.5 g is therefore held for practically the whole burn at the as-built
// mass, and through the first ~2.8 s even 125 g heavy — far longer than
// the hold. Well above the 1 g a rocket sitting on the rail reads.
#define LAUNCH_ACCEL_THRESHOLD_G   1.5f
#define LAUNCH_ACCEL_TAU_MS        40     // low-pass time constant on the launch accel
#define LAUNCH_CONFIRM_MS          500    // filtered accel must stay above LAUNCH_RELEASE_G this long

// Once latched, the hold is only broken if the filtered accel falls back
// below this — not the trigger threshold — so vibration riding on a
// marginal thrust level (heavy rocket, late in the spike) doesn't cancel a
// real launch. Still well above the 1 g at rest, so a knock that has
// passed is caught. Checked offline against the F-15 curve with up to
// 1.5 g of vibration noise: a big gain in confirmations, no extra knocks.
#define LAUNCH_RELEASE_G           1.2f

// G2b — Climb backstop: a launch the accel trigger missed (odd motor, an
// accelerometer problem, ignition during the 30 s settle) still shows up
// as the barometer climbing fast. Measured on the raw baro altitude —
// independent of the accelerometer, and frozen (not drifting) if the baro
// dies — against a reference that follows the pad slowly
// (LAUNCH_CLIMB_REF_TAU_MS), so carrying the rocket uphill or a slow
// weather change never builds up 8 m: a steady 2 m/s climb lags only 6 m.
// The F-15 gets 8 m clear of the reference at T+1.5 s (as built) to
// T+2.0 s (125 g heavy and 15% weak), still in the burn. Runs in IDLE and
// ARMED at all times, the settle included.
#define LAUNCH_CLIMB_M             8.0f
#define LAUNCH_CLIMB_HOLD_MS       100     // must hold this long (one wild baro sample isn't a climb)
#define LAUNCH_CLIMB_REF_TAU_MS    3000

// G2c — Unarmed failsafe: if the motor lights with SW401 open, the same
// launch detection (G2 accel trigger, G2b climb) runs in IDLE too, with no
// settle wait, so the rocket still steers, logs and runs its recovery
// sequence. The open switch has cut PYRO PWR, so the main charge can't
// actually fire. A rocket on the pad reads 1 g, which never passes
// burnout (BURNOUT_ACCEL_THRESHOLD_G), so a false trigger on the ground
// sits in POWERED with nothing fired — reboot to clear it.

// POWERED → COAST: motor burnout
#define BURNOUT_ACCEL_THRESHOLD_G  0.5f

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

// Landing detection — all must hold for LANDED_TIME_MS: accel ~1 g, gyro
// quiet, AND vertical speed near zero. Under a main chute that barely
// swings, accel also reads ~1 g and the gyro stays quiet, so stillness
// alone would call touchdown mid-air; the speed check rules that out.
#define LANDED_ACCEL_LOW_G         0.75f
#define LANDED_ACCEL_HIGH_G        1.35f
#define LANDED_GYRO_THRESHOLD_DPS  10.0f
#define LANDED_VELOCITY_MS         2.0f    // |vertical speed| below this (descent under the main is ~4.5 m/s)
#define LANDED_TIME_MS             5000

// Backstop, measured from liftoff: by now any flight is down. Stillness
// alone is then enough (a dead baro leaves the speed estimate drifting),
// and with a dead IMU, where stillness can't be judged, nothing else is
// needed. Without it those flights would never close out their log.
#define LANDED_FALLBACK_MS         120000

// --------------------------------------------------------
// STATES
// --------------------------------------------------------

typedef enum {
    STATE_IDLE          = 0,   // on the pad, disarmed (SW401 open)
    STATE_ARMED         = 1,   // on the pad, armed (SW401 closed) — launch detection after ARM_SETTLE_MS, see fsm_set_armed()
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

    // G1 — set once SW401 has been closed for ARM_SETTLE_MS; the accel
    // trigger runs in ARMED only while this is true. Cleared on disarm.
    bool     launch_armed;

    // Any launch trigger (accel latch or climb) since the last arming
    // edge — tells the logger the flash log must not be recycled.
    bool     launch_seen;

    // G2 — low-pass filtered accel_up_g, the signal launch detection uses.
    float    launch_accel_g;
    uint32_t last_update_ms;    // previous fsm_update() call, for the filters' dt

    // G2b — slowly tracking pad altitude (raw baro) and the climb timer.
    float    climb_ref_m;       // NAN = re-seed from the next baro reading
    uint32_t climb_since_ms;    // 0 = not above LAUNCH_CLIMB_M

    // Fault flags. imu_fault latches on an invalid IMU reading: TVC goes
    // off, and so does velocity-based apogee (the velocity is integrated
    // accel) — the baro-drop and launch-timeout backstops still run.
    bool     imu_fault;

    // Set by fsm_resume() after a mid-flight watchdog reset. The velocity
    // estimate restarts from zero, so velocity-based apogee is disabled —
    // the baro-drop and launch-timeout backstops still run.
    bool     resumed;
} FlightSM;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void fsm_init(FlightSM *fsm);

/**
 * Update state machine. Call every loop iteration.
 * @param altitude_m  Estimated altitude AGL [m] — used for the apogee baro-drop backstop.
 * @param baro_alt_m  Raw barometric altitude AGL [m], last sample held (NAN before the
 *                    first) — used only by the G2b climb backstop on the pad.
 */
void fsm_update(FlightSM *fsm,
                float accel_up_g,
                float velocity_ms,
                float accel_mag_g,
                float gyro_rate_dps,
                float altitude_m,
                float baro_alt_m,
                bool  imu_valid);

/**
 * Re-seed the G2b climb reference from the next baro reading. Call when
 * the altitude reference is re-zeroed on the pad (the SW401 ground
 * capture), which steps the baro altitude and would look like a climb.
 */
void fsm_reset_climb_ref(FlightSM *fsm);

/**
 * Reflect the arming switch (SW401) into the FSM. Only transitions
 * IDLE <-> ARMED — a no-op once launch detection has latched or flight
 * has begun. Entering ARMED starts the ARM_SETTLE_MS wait, after which
 * launch detection is live; going back to IDLE cancels it. The switch is
 * also a hardware interlock (it cuts PYRO PWR) — see
 * main_control_loop.cpp's arming-switch handling for the full policy.
 */
void fsm_set_armed(FlightSM *fsm, bool armed);

/**
 * Pick a flight back up after a mid-flight watchdog reset (see
 * flight_resume.h). Call right after fsm_init(). POWERED/COAST resume as
 * COAST with TVC off — the attitude filter just restarted and can't be
 * trusted to steer. APOGEE/DESCENT/MAIN resume as MAIN, which retries the
 * main charge until it has fired (the fired flag survives in EEPROM).
 * The state change is reported by fsm_state_changed() like any other.
 *
 * @param saved            State at the last save before the reset.
 * @param ms_since_launch  Time since liftoff confirmation at that save.
 * @param peak_altitude_m  Peak altitude seen before the reset.
 * @param flight_proven    Peak velocity had passed MIN_FLIGHT_VELOCITY_MS.
 */
void fsm_resume(FlightSM *fsm, FlightState saved, uint32_t ms_since_launch,
                float peak_altitude_m, bool flight_proven);

/** Emergency abort — safes all outputs, sets STATE_ABORT. Pad-side faults only. */
void fsm_abort(FlightSM *fsm);

/** Returns true on the first call after a state transition (one-shot). */
bool fsm_state_changed(FlightSM *fsm);

/** How long the FSM has been in the current state [ms]. */
uint32_t fsm_time_in_state(const FlightSM *fsm);
