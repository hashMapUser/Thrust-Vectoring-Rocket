#include <Arduino.h>
#include "flight_sm.h"

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
    fsm->state              = STATE_IDLE;
    fsm->prev_state         = STATE_IDLE;
    fsm->state_entry_ms     = millis();
    fsm->launch_detect_ms   = 0;
    fsm->powered_entry_ms   = 0;
    fsm->prev_velocity_ms   = 0.0f;
    fsm->peak_velocity_ms   = 0.0f;
    fsm->drogue_fired       = false;
    fsm->main_fired         = false;
    fsm->tvc_enabled        = false;
    fsm->pad_rest_satisfied = false;
    fsm->pad_rest_start_ms  = 0;
    fsm->imu_fault          = false;
}

bool fsm_arm(FlightSM *fsm, PyroState *pyro, bool arm_sense_ok) {
    if (fsm->state != STATE_IDLE) return false;

    // T3: Refuse IDLE→ARMED unless pyro power is confirmed present (SW401 closed).
    if (!arm_sense_ok) {
        Serial.println("[FSM] ARM REJECTED — ARM_SENSE below threshold");
        return false;
    }

    pyro_arm(pyro);
    enter_state(fsm, STATE_ARMED);
    fsm->tvc_enabled        = true;
    fsm->pad_rest_satisfied = false;
    fsm->pad_rest_start_ms  = 0;
    fsm->peak_velocity_ms   = 0.0f;
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
        fsm->prev_state = fsm->state;
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
                float altitude_m,
                bool  imu_valid) {

    uint32_t now = millis();

    // ── FAULT DETECTION ──────────────────────────────────────────
    if (!imu_valid) fsm->imu_fault = true;

    if (fsm->imu_fault &&
        (fsm->state == STATE_POWERED || fsm->state == STATE_COAST)) {
        fsm_abort(fsm);
        return;
    }

    // ── STATE TRANSITIONS ─────────────────────────────────────────
    switch (fsm->state) {

        case STATE_IDLE:
            break;

        case STATE_ARMED: {
            // G1 — Pad-rest precondition: vehicle must be stationary and
            // vertical for PAD_REST_MS before a launch signature is accepted.
            bool accel_ok   = (accel_mag_g  >= PAD_REST_ACCEL_LOW_G &&
                                accel_mag_g  <= PAD_REST_ACCEL_HIGH_G);
            bool gyro_ok    = (gyro_rate_dps < PAD_REST_GYRO_DPS);
            bool upright_ok = (accel_up_g    > PAD_REST_ACCEL_UP_G);

            if (accel_ok && gyro_ok && upright_ok) {
                if (fsm->pad_rest_start_ms == 0)
                    fsm->pad_rest_start_ms = now;
                if ((now - fsm->pad_rest_start_ms) >= PAD_REST_MS)
                    fsm->pad_rest_satisfied = true;
            } else {
                // Any condition break clears the latch — cannot be satisfied
                // mid-carry and then immediately satisfy launch detect.
                fsm->pad_rest_start_ms  = 0;
                fsm->pad_rest_satisfied = false;
            }

            // G2 — Launch detection: 4 g sustained for 200 ms.
            // Only allowed after G1 is satisfied.
            if (fsm->pad_rest_satisfied && accel_up_g > LAUNCH_ACCEL_THRESHOLD_G) {
                if (fsm->launch_detect_ms == 0) {
                    fsm->launch_detect_ms = now;
                } else if ((now - fsm->launch_detect_ms) >= LAUNCH_ACCEL_MS) {
                    fsm->powered_entry_ms = now;
                    enter_state(fsm, STATE_POWERED);
                    fsm->launch_detect_ms = 0;
                }
            } else {
                fsm->launch_detect_ms = 0;
            }
            break;
        }

        case STATE_POWERED:
            // G3 — Track peak velocity for flight-proof latch.
            if (velocity_ms > fsm->peak_velocity_ms)
                fsm->peak_velocity_ms = velocity_ms;

            // Burnout: thrust gone → falls toward 0 g.
            if (accel_up_g < BURNOUT_ACCEL_THRESHOLD_G) {
                fsm->tvc_enabled = false;
                enter_state(fsm, STATE_COAST);
            }
            break;

        case STATE_COAST:
            // G3 — Continue tracking peak velocity.
            if (velocity_ms > fsm->peak_velocity_ms)
                fsm->peak_velocity_ms = velocity_ms;

            // Gates G3 and G4 must both be satisfied before apogee is reachable.
            {
                uint32_t time_since_powered = now - fsm->powered_entry_ms;
                bool g3 = (fsm->peak_velocity_ms > MIN_FLIGHT_VELOCITY_MS);
                bool g4 = (time_since_powered >= MIN_FLIGHT_TIME_MS) &&
                          (fsm_time_in_state(fsm) >= COAST_APOGEE_MIN_MS);

                if (g3 && g4) {
                    bool vel_apogee = (fsm->prev_velocity_ms > 0.0f && velocity_ms <= 0.0f);
                    // G7 — Timeout backstop only fires when G3+G4 are satisfied.
                    bool timeout    = (fsm_time_in_state(fsm) >= APOGEE_TIMEOUT_MS);

                    if (vel_apogee || timeout) {
                        enter_state(fsm, STATE_APOGEE);
                    }
                }
            }
            break;

        case STATE_APOGEE:
            // Single-chute flight: jump directly to STATE_MAIN.
            // Pyro fire is handled by main loop on STATE_MAIN entry.
            enter_state(fsm, STATE_MAIN);
            break;

        case STATE_DESCENT:
            // Not reached on single-chute flights.
            break;

        case STATE_MAIN:
            // Landing detection: accel magnitude near 1 g AND gyro near zero,
            // held for LANDED_TIME_MS.
            {
                bool accel_ok = (accel_mag_g >= LANDED_ACCEL_LOW_G &&
                                 accel_mag_g <= LANDED_ACCEL_HIGH_G);
                bool gyro_ok  = (gyro_rate_dps < LANDED_GYRO_THRESHOLD_DPS);

                if (accel_ok && gyro_ok) {
                    if (fsm_time_in_state(fsm) >= LANDED_TIME_MS)
                        enter_state(fsm, STATE_LANDED);
                } else {
                    fsm->state_entry_ms = now;  // reset timer while still moving
                }
            }
            break;

        case STATE_LANDED:
        case STATE_ABORT:
            break;
    }

    fsm->prev_velocity_ms = velocity_ms;
    (void)altitude_m;  // used by caller for G5 — passed to pyro_fire_main()
}
