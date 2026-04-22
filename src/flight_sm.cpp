#include <Arduino.h>
#include "flight_sm.h"

// --------------------------------------------------------
// PRIVATE — STATE TRANSITION
// --------------------------------------------------------

static void enter_state(FlightSM *fsm, FlightState new_state) {
    fsm->prev_state    = fsm->state;
    fsm->state         = new_state;
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
                float accel_g,
                float altitude_m,
                float velocity_ms,
                bool  imu_valid,
                bool  baro_valid) {

    uint32_t now = millis();

    // ── FAULT DETECTION ──
    // Persistent sensor failures trigger abort in flight
    if (!imu_valid)  fsm->imu_fault  = true;
    if (!baro_valid) fsm->baro_fault = true;

    if ((fsm->imu_fault || fsm->baro_fault) &&
        (fsm->state == STATE_POWERED || fsm->state == STATE_COAST)) {
        fsm_abort(fsm);
        return;
    }

    // Track maximum altitude for apogee detection
    if (altitude_m > fsm->max_altitude_m) {
        fsm->max_altitude_m = altitude_m;
    }

    // ── STATE TRANSITIONS ──
    switch (fsm->state) {

        case STATE_IDLE:
            // Only exits via fsm_arm() — no automatic transition
            break;

        case STATE_ARMED:
            // Launch detection: accel > threshold for LAUNCH_ACCEL_MS
            if (accel_g > LAUNCH_ACCEL_THRESHOLD_G) {
                if (fsm->launch_detect_ms == 0) {
                    fsm->launch_detect_ms = now;
                } else if ((now - fsm->launch_detect_ms) >= LAUNCH_ACCEL_MS) {
                    enter_state(fsm, STATE_POWERED);
                    fsm->launch_detect_ms = 0;
                }
            } else {
                // Reset timer if accel drops back below threshold
                fsm->launch_detect_ms = 0;
            }
            break;

        case STATE_POWERED:
            // Burnout detection: accel drops below threshold
            if (accel_g < BURNOUT_ACCEL_THRESHOLD_G) {
                fsm->tvc_enabled = false;
                enter_state(fsm, STATE_COAST);
            }
            break;

        case STATE_COAST:
            // Apogee detection: velocity crosses zero (was positive, now negative)
            // or timeout
            if ((fsm->prev_velocity_ms > 0.0f && velocity_ms <= 0.0f) ||
                (fsm_time_in_state(fsm) > APOGEE_TIMEOUT_MS)) {
                enter_state(fsm, STATE_APOGEE);
            }
            break;

        case STATE_APOGEE:
            // One-shot: fire drogue on state entry
            // Actual pyro fire is handled in main loop via fsm_state_changed()
            // Immediately transition to DESCENT
            enter_state(fsm, STATE_DESCENT);
            break;

        case STATE_DESCENT:
            // Main deploy at target AGL altitude
            if (altitude_m <= MAIN_DEPLOY_ALT_M && altitude_m > 0.0f) {
                enter_state(fsm, STATE_MAIN);
            }
            break;

        case STATE_MAIN:
            // Landing detection: near-zero velocity for LANDED_TIME_MS
            if (fabsf(velocity_ms) < LANDED_VEL_THRESHOLD_MS) {
                if (fsm_time_in_state(fsm) >= LANDED_TIME_MS) {
                    enter_state(fsm, STATE_LANDED);
                }
            } else {
                // Reset entry time if velocity spikes again
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