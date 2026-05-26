#include <Arduino.h>
#include "flight_sm.h"

// ============================================================
// CONVENTION
// ============================================================
// All vertical-axis inputs to this module use a "positive = up"
// convention — independent of how the IMU is physically mounted.
//
//   accel_up_g   : specific force on the rocket's vertical axis [g],
//                  with sign convention chosen so that the value is
//                  approximately +1g at rest (rocket upright, motor off)
//                  and grows to +5g…+8g under thrust.
//   velocity_ms  : vertical velocity [m/s], positive going up.
//   altitude_m   : altitude AGL [m], positive going up.
//
// The caller is responsible for mapping the physical IMU axis into
// this convention before calling fsm_update(). For example, on this
// vehicle the IMU's X axis points toward the tail (gravity acts in
// +X), so the caller does:
//
//     float accel_up_g = -imu.accel_x_g();
//     alt_update(&alt, p, accel_up_g, dt);
//     fsm_update(&fsm, accel_up_g, alt.altitude_m, alt.velocity_ms, ...);
//
// Keeping the sign-flip in the caller decouples the FSM from sensor
// mounting and matches the convention used by alt_estimator.h.

// --------------------------------------------------------
// PRIVATE — STATE TRANSITION
// --------------------------------------------------------

static void enter_state(FlightSM *fsm, FlightState new_state) {
    fsm->prev_state     = fsm->state;
    fsm->state          = new_state;
    fsm->state_entry_ms = millis();

    Serial.print("[FSM] ");
    Serial.print(STATE_NAMES[fsm->prev_state]);
    Serial.print(" → ");
    Serial.println(STATE_NAMES[new_state]);
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void fsm_init(FlightSM *fsm) {
    fsm->state            = STATE_IDLE;
    fsm->prev_state       = STATE_IDLE;
    fsm->state_entry_ms   = millis();
    fsm->launch_detect_ms = 0;
    fsm->max_altitude_m   = 0.0f;
    fsm->prev_velocity_ms = 0.0f;
    fsm->drogue_fired     = false;
    fsm->main_fired       = false;
    fsm->tvc_enabled      = false;
    fsm->imu_fault        = false;
    fsm->baro_fault       = false;
    fsm->mag_fault        = false;
}

bool fsm_arm(FlightSM *fsm) {
    if (fsm->state != STATE_IDLE) return false;
    enter_state(fsm, STATE_ARMED);
    fsm->tvc_enabled = true;
    return true;
}

void fsm_disarm(FlightSM *fsm) {
    if (fsm->state != STATE_ARMED) return;
    fsm->tvc_enabled = false;
    enter_state(fsm, STATE_IDLE);
}

void fsm_abort(FlightSM *fsm) {
    fsm->tvc_enabled = false;
    enter_state(fsm, STATE_ABORT);
    Serial.println("[FSM] ABORT — all outputs safed");
}

bool fsm_state_changed(FlightSM *fsm) {
    return fsm->state != fsm->prev_state;
}

uint32_t fsm_time_in_state(const FlightSM *fsm) {
    return millis() - fsm->state_entry_ms;
}

void fsm_update(FlightSM *fsm,
                float accel_up_g,     // see CONVENTION block at top of file
                float altitude_m,
                float velocity_ms,
                bool  imu_valid,
                bool  baro_valid) {

    uint32_t now = millis();

    // ── FAULT DETECTION ──
    // Persistent sensor failures trigger abort in powered/coast flight.
    if (!imu_valid)  fsm->imu_fault  = true;
    if (!baro_valid) fsm->baro_fault = true;

    if ((fsm->imu_fault || fsm->baro_fault) &&
        (fsm->state == STATE_POWERED || fsm->state == STATE_COAST)) {
        fsm_abort(fsm);
        return;
    }

    // Track maximum altitude — used for telemetry and as a sanity check
    // for the apogee transition.
    if (altitude_m > fsm->max_altitude_m) {
        fsm->max_altitude_m = altitude_m;
    }

    // ── STATE TRANSITIONS ──
    switch (fsm->state) {

        case STATE_IDLE:
            // Only exits via fsm_arm() — typically driven by an 'A' over
            // USB serial from the ground station.
            break;

        case STATE_ARMED:
            // Launch detection: vertical accel above launch threshold for
            // LAUNCH_ACCEL_MS continuous milliseconds. With "positive = up"
            // convention, rest is ~+1g and motor thrust drives the reading
            // well above LAUNCH_ACCEL_THRESHOLD_G (typically 2.0g–3.0g).
            if (accel_up_g > LAUNCH_ACCEL_THRESHOLD_G) {
                if (fsm->launch_detect_ms == 0) {
                    fsm->launch_detect_ms = now;
                } else if ((now - fsm->launch_detect_ms) >= LAUNCH_ACCEL_MS) {
                    enter_state(fsm, STATE_POWERED);
                    fsm->launch_detect_ms = 0;
                }
            } else {
                // Reset timer if accel drops back below threshold —
                // prevents a single spike (wind gust, handling bump) from
                // triggering launch on its own.
                fsm->launch_detect_ms = 0;
            }
            break;

        case STATE_POWERED:
            // Burnout detection: thrust gone → reading falls back toward
            // 0g (free-fall in ballistic flight). BURNOUT_ACCEL_THRESHOLD_G
            // is typically ~1.5g — below the boost reading but above the
            // post-burnout free-fall value.
            if (accel_up_g < BURNOUT_ACCEL_THRESHOLD_G) {
                fsm->tvc_enabled = false;
                enter_state(fsm, STATE_COAST);
            }
            break;

        case STATE_COAST:
            // Apogee detection: vertical velocity crosses zero (was
            // positive, now non-positive), or coast timeout as a
            // backstop.
            //
            // NOTE: this in-line zero-crossing test is the original
            // implementation. The consecutive-decrease detector in
            // apogee_detector.{h,cpp} is the replacement — once you wire
            // it into the main loop, this block becomes a fallback driven
            // by apogee_tick() returning true rather than the velocity
            // check below.
            if ((fsm->prev_velocity_ms > 0.0f && velocity_ms <= 0.0f) ||
                (fsm_time_in_state(fsm) > APOGEE_TIMEOUT_MS)) {
                enter_state(fsm, STATE_APOGEE);
            }
            break;

        case STATE_APOGEE:
            // One-shot: drogue fire is handled in the main loop via
            // fsm_state_changed() — we immediately transition to DESCENT
            // here so the pyro pulse is bounded by one main-loop iteration.
            enter_state(fsm, STATE_DESCENT);
            break;

        case STATE_DESCENT:
            // Main deploy at target AGL altitude.
            if (altitude_m <= MAIN_DEPLOY_ALT_M && altitude_m > 0.0f) {
                enter_state(fsm, STATE_MAIN);
            }
            break;

        case STATE_MAIN:
            // Landing detection: near-zero vertical velocity sustained for
            // LANDED_TIME_MS. We reset the entry time whenever velocity
            // spikes back up so a swinging chute can't trigger LANDED
            // prematurely.
            if (fabsf(velocity_ms) < LANDED_VEL_THRESHOLD_MS) {
                if (fsm_time_in_state(fsm) >= LANDED_TIME_MS) {
                    enter_state(fsm, STATE_LANDED);
                }
            } else {
                fsm->state_entry_ms = now;
            }
            break;

        case STATE_LANDED:
            // Terminal state — nothing to do
            break;

        case STATE_ABORT:
            // Terminal state — nothing to do
            break;
    }

    fsm->prev_velocity_ms = velocity_ms;
}