#include <Arduino.h>
#include "flight_sm.h"

// --------------------------------------------------------
// PRIVATE — STATE TRANSITION
// --------------------------------------------------------

static void enter_state(FlightSM *fsm, FlightState new_state) {
    fsm->prev_state     = fsm->state;
    fsm->state          = new_state;
    fsm->state_entry_ms = millis();

    Serial.print("\n========================================\n");
    Serial.print("[FSM] STATE TRANSITION: ");
    Serial.print(STATE_NAMES[fsm->prev_state]);
    Serial.print(" → ");
    Serial.println(STATE_NAMES[new_state]);
    Serial.print("========================================\n");
}

// Bench aid: non-blocking green-LED flash on pad-rest latch, so it's
// visible without a serial monitor open. No delay() here — this runs
// inside the 125 Hz flight loop and must not stall sensor reads / PID /
// pyro fire-duration timing for however long a blocking flash would take.
#define PAD_REST_LED_FLASH_MS 300
static uint32_t pad_rest_led_off_ms = 0;

static void update_pad_rest(FlightSM *fsm, float accel_mag_g, float gyro_rate_dps,
                             float accel_up_g, float altitude_m, uint32_t now) {
    if (pad_rest_led_off_ms != 0 && (int32_t)(now - pad_rest_led_off_ms) >= 0) {
        digitalWriteFast(PIN_LED_GREEN, LOW);
        pad_rest_led_off_ms = 0;
    }

    bool accel_ok   = (accel_mag_g  >= PAD_REST_ACCEL_LOW_G &&
                        accel_mag_g  <= PAD_REST_ACCEL_HIGH_G);
    bool gyro_ok    = (gyro_rate_dps < PAD_REST_GYRO_DPS);
    bool upright_ok = (accel_up_g    > PAD_REST_ACCEL_UP_G);

    if (accel_ok && gyro_ok && upright_ok) {
        if (fsm->pad_rest_start_ms == 0) {
            fsm->pad_rest_start_ms = now;
            Serial.println("[FSM] Pad rest timer started...");
        }
        if (!fsm->pad_rest_satisfied && (now - fsm->pad_rest_start_ms) >= PAD_REST_MS) {
            fsm->pad_rest_satisfied      = true;
            fsm->pad_rest_baseline_alt_m = altitude_m;
            Serial.print("[FSM] PAD REST SATISFIED. Baseline Alt: ");
            Serial.print(altitude_m);
            Serial.println(" m. Ready for launch.");

            digitalWriteFast(PIN_LED_GREEN, HIGH);
            pad_rest_led_off_ms = now + PAD_REST_LED_FLASH_MS;
        }
    } else {
        if (fsm->pad_rest_start_ms != 0 || fsm->pad_rest_satisfied) {
            Serial.println("[FSM] Pad rest LOST (movement detected). Resetting latch.");
        }
        fsm->pad_rest_start_ms  = 0;
        fsm->pad_rest_satisfied = false;
    }
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void fsm_init(FlightSM *fsm) {
    fsm->state                   = STATE_IDLE;
    fsm->prev_state              = STATE_IDLE;
    fsm->state_entry_ms          = millis();
    fsm->launch_detect_ms        = 0;
    fsm->powered_entry_ms        = 0;
    fsm->prev_velocity_ms        = 0.0f;
    fsm->peak_velocity_ms        = 0.0f;
    fsm->drogue_fired            = false;
    fsm->main_fired              = false;
    fsm->tvc_enabled             = false;
    fsm->pad_rest_satisfied      = false;
    fsm->pad_rest_start_ms       = 0;
    fsm->pad_rest_baseline_alt_m = 0.0f;
    fsm->imu_fault               = false;
    
    Serial.println("[FSM] Initialized. Awaiting pad rest.");
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
        Serial.println("[FSM] CRITICAL: IMU FAULT DURING FLIGHT!");
        fsm_abort(fsm);
        return;
    }

    // ── STATE TRANSITIONS ─────────────────────────────────────────
    switch (fsm->state) {

        case STATE_IDLE:
            // No arm step — there's no arming mechanism on this board.
            // Pad-rest gates launch detection directly: once latched,
            // watch for a real launch signature and go straight to
            // POWERED. (This used to be a two-stage IDLE -> ARMED ->
            // POWERED flow with a re-latch in between; collapsed to one
            // stage since there's nothing left to gate the ARMED step on.)
            if (fsm->pad_rest_satisfied && accel_up_g > LAUNCH_ACCEL_THRESHOLD_G) {
                if (fsm->launch_detect_ms == 0) {
                    fsm->launch_detect_ms = now;
                    Serial.println("[FSM] >> LAUNCH ACCEL DETECTED! Waiting for hold time and altitude gain...");
                } else {
                    uint32_t held_ms    = now - fsm->launch_detect_ms;
                    bool alt_confirmed  = (altitude_m - fsm->pad_rest_baseline_alt_m) >= LAUNCH_ALT_DELTA_M;

                    if (held_ms >= LAUNCH_ACCEL_MS && alt_confirmed) {
                        Serial.println("[FSM] >> LIFTOFF CONFIRMED!");
                        fsm->powered_entry_ms = now;
                        fsm->tvc_enabled      = true;
                        enter_state(fsm, STATE_POWERED);
                        fsm->launch_detect_ms = 0;
                    } else if (held_ms >= LAUNCH_CONFIRM_MS) {
                        Serial.println("[FSM] >> FALSE LAUNCH TRIGGER: Altitude gain failed. Resetting.");
                        fsm->launch_detect_ms = 0;
                        fsm->pad_rest_satisfied = false;
                    }
                }
            } else {
                fsm->launch_detect_ms = 0;
                update_pad_rest(fsm, accel_mag_g, gyro_rate_dps, accel_up_g, altitude_m, now);
            }
            break;

        // STATE_ARMED is unused — kept in the enum only so log/telemetry
        // numbering doesn't shift. The FSM never transitions into it.
        case STATE_ARMED:
            break;

        case STATE_POWERED:
            if (velocity_ms > fsm->peak_velocity_ms)
                fsm->peak_velocity_ms = velocity_ms;

            if (accel_up_g < BURNOUT_ACCEL_THRESHOLD_G) {
                Serial.println("[FSM] >> MOTOR BURNOUT DETECTED. Coasting...");
                fsm->tvc_enabled = false;
                enter_state(fsm, STATE_COAST);
            }
            break;

        case STATE_COAST:
            if (velocity_ms > fsm->peak_velocity_ms)
                fsm->peak_velocity_ms = velocity_ms;

            {
                uint32_t time_since_powered = now - fsm->powered_entry_ms;
                bool g3 = (fsm->peak_velocity_ms > MIN_FLIGHT_VELOCITY_MS);
                bool g4 = (time_since_powered >= MIN_FLIGHT_TIME_MS) &&
                          (fsm_time_in_state(fsm) >= COAST_APOGEE_MIN_MS);

                if (g3 && g4) {
                    bool vel_apogee = (fsm->prev_velocity_ms > 0.0f && velocity_ms <= 0.0f);
                    bool timeout    = (fsm_time_in_state(fsm) >= APOGEE_TIMEOUT_MS);

                    if (vel_apogee) {
                        Serial.println("[FSM] >> APOGEE DETECTED: Vertical velocity crossed 0.");
                        enter_state(fsm, STATE_APOGEE);
                    } else if (timeout) {
                        Serial.println("[FSM] >> APOGEE DETECTED: Coast timeout reached.");
                        enter_state(fsm, STATE_APOGEE);
                    }
                }
            }
            break;

        case STATE_APOGEE:
            Serial.println("[FSM] >> DEPLOYING MAIN CHUTE");
            enter_state(fsm, STATE_MAIN);
            break;

        case STATE_DESCENT:
            break;

        case STATE_MAIN:
            {
                bool accel_ok = (accel_mag_g >= LANDED_ACCEL_LOW_G &&
                                 accel_mag_g <= LANDED_ACCEL_HIGH_G);
                bool gyro_ok  = (gyro_rate_dps < LANDED_GYRO_THRESHOLD_DPS);

                if (accel_ok && gyro_ok) {
                    if (fsm_time_in_state(fsm) >= LANDED_TIME_MS) {
                        Serial.println("[FSM] >> TOUCHDOWN CONFIRMED.");
                        enter_state(fsm, STATE_LANDED);
                    }
                } else {
                    fsm->state_entry_ms = now;  
                }
            }
            break;

        case STATE_LANDED:
        case STATE_ABORT:
            break;
    }

    fsm->prev_velocity_ms = velocity_ms;
}