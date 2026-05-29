#include <Arduino.h>
#include "flight_sm.h"

// ============================================================
// CONVENTION
// ============================================================
// All vertical-axis inputs use a "positive = up" convention.
//
//   accel_up_g   : specific force on the rocket's vertical axis [g].
//                  ≈ +1g at rest, grows to +5g…+8g under thrust.
//   velocity_ms  : vertical velocity [m/s] integrated by the caller
//                  from IMU accel. Barometer not used this flight.
//
// accel_mag_g and gyro_rate_dps are magnitudes of the full 3-axis
// vectors, used only for landed detection in STATE_MAIN.

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
    fsm->prev_velocity_ms = 0.0f;
    fsm->drogue_fired     = false;
    fsm->main_fired       = false;
    fsm->tvc_enabled      = false;
    fsm->imu_fault        = false;
    fsm->switch_engaged   = false;
}

bool fsm_arm(FlightSM *fsm, PyroState *pyro) {
    if (fsm->state != STATE_IDLE) return false;
    // Continuity check removed for this flight — arm unconditionally.
    pyro->drogue_armed = true;
    pyro->main_armed   = true;
    Serial.println("[PYRO] Armed (no continuity check this flight)");
    enter_state(fsm, STATE_ARMED);
    fsm->tvc_enabled = true;
    return true;
}

void fsm_disarm(FlightSM *fsm, PyroState *pyro) {
    if (fsm->state != STATE_ARMED) return;
    pyro_disarm(pyro);
    fsm->tvc_enabled = false;
    enter_state(fsm, STATE_IDLE);
}

void fsm_abort(FlightSM *fsm) {
    fsm->tvc_enabled = false;
    enter_state(fsm, STATE_ABORT);
    Serial.println("[FSM] ABORT — all outputs safed");
}

bool fsm_state_changed(FlightSM *fsm) {
    if (fsm->state != fsm->prev_state) {
        fsm->prev_state = fsm->state;  // consume — returns false until next transition
        return true;
    }
    return false;
}

uint32_t fsm_time_in_state(const FlightSM *fsm) {
    return millis() - fsm->state_entry_ms;
}

void fsm_update(FlightSM *fsm,
                float accel_up_g,
                float velocity_ms,
                float accel_mag_g,
                float gyro_rate_dps,
                bool  imu_valid) {

    uint32_t now = millis();

    // ── FAULT DETECTION ──
    if (!imu_valid) fsm->imu_fault = true;

    if (fsm->imu_fault &&
        (fsm->state == STATE_POWERED || fsm->state == STATE_COAST)) {
        fsm_abort(fsm);
        return;
    }

    // ── STATE TRANSITIONS ──
    switch (fsm->state) {

        case STATE_IDLE:
            // Exits only via fsm_arm() — called by the RBF debounce block.
            break;

        case STATE_ARMED:
            // Launch detection: vertical accel above threshold for
            // LAUNCH_ACCEL_MS continuous milliseconds.
            if (accel_up_g > LAUNCH_ACCEL_THRESHOLD_G) {
                if (fsm->launch_detect_ms == 0) {
                    fsm->launch_detect_ms = now;
                } else if ((now - fsm->launch_detect_ms) >= LAUNCH_ACCEL_MS) {
                    enter_state(fsm, STATE_POWERED);
                    fsm->launch_detect_ms = 0;
                }
            } else {
                // Reset timer on dip — prevents a single spike from triggering.
                fsm->launch_detect_ms = 0;
            }
            break;

        case STATE_POWERED:
            // Burnout: thrust gone → reading falls toward 0 g.
            if (accel_up_g < BURNOUT_ACCEL_THRESHOLD_G) {
                fsm->tvc_enabled = false;
                enter_state(fsm, STATE_COAST);
            }
            break;

        case STATE_COAST:
            // Apogee: integrated vertical velocity crosses zero (was positive,
            // now non-positive). Guard of COAST_APOGEE_MIN_MS prevents a false
            // trigger immediately after burnout. APOGEE_TIMEOUT_MS is the backstop.
            if (fsm_time_in_state(fsm) >= COAST_APOGEE_MIN_MS) {
                if ((fsm->prev_velocity_ms > 0.0f && velocity_ms <= 0.0f) ||
                    (fsm_time_in_state(fsm) >= APOGEE_TIMEOUT_MS)) {
                    enter_state(fsm, STATE_APOGEE);
                }
            }
            break;

        case STATE_APOGEE:
            // Single-chute flight: no drogue. Fire main at apogee and
            // jump directly to STATE_MAIN for landed detection.
            // One-shot: pyro fire is handled by the main loop via
            // fsm_state_changed() before this transition executes.
            enter_state(fsm, STATE_MAIN);
            break;

        case STATE_DESCENT:
            // Not reached on single-chute flights (APOGEE → MAIN directly).
            break;

        case STATE_MAIN:
            // Landing detection: accel magnitude near 1 g (rocket sitting on
            // ground) AND angular rate near zero, held for LANDED_TIME_MS.
            // Replaces baro velocity-based detect.
            {
                bool accel_ok = (accel_mag_g >= LANDED_ACCEL_LOW_G &&
                                 accel_mag_g <= LANDED_ACCEL_HIGH_G);
                bool gyro_ok  = (gyro_rate_dps < LANDED_GYRO_THRESHOLD_DPS);

                if (accel_ok && gyro_ok) {
                    if (fsm_time_in_state(fsm) >= LANDED_TIME_MS) {
                        enter_state(fsm, STATE_LANDED);
                    }
                } else {
                    // Reset timer while still moving.
                    fsm->state_entry_ms = now;
                }
            }
            break;

        case STATE_LANDED:
            // Terminal state
            break;

        case STATE_ABORT:
            // Terminal state
            break;
    }

    fsm->prev_velocity_ms = velocity_ms;
}
