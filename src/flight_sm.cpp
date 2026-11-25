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

// Bench aid: non-blocking green-LED flash when launch detection arms, so
// it's visible without a serial monitor open. No delay() here — this runs
// inside the 125 Hz flight loop and must not stall sensor reads / PID /
// pyro fire-duration timing for however long a blocking flash would take.
#define LAUNCH_ARMED_LED_FLASH_MS 300
static uint32_t led_off_ms = 0;

static void update_led(uint32_t now) {
    if (led_off_ms != 0 && (int32_t)(now - led_off_ms) >= 0) {
        digitalWriteFast(PIN_LED_GREEN, LOW);
        led_off_ms = 0;
    }
}

// Liftoff, from any of the launch detectors.
static void liftoff(FlightSM *fsm, uint32_t now, const char *why) {
    Serial.print(fsm->state == STATE_IDLE ? "[FSM] >> LIFTOFF CONFIRMED WITH SW401 OPEN (unarmed failsafe) — "
                                          : "[FSM] >> LIFTOFF CONFIRMED — ");
    Serial.println(why);
    fsm->launch_seen      = true;
    fsm->powered_entry_ms = now;
    fsm->tvc_enabled      = true;
    enter_state(fsm, STATE_POWERED);
    fsm->launch_detect_ms = 0;
}

// G2b — true once the raw baro altitude has held LAUNCH_CLIMB_M above the
// slowly tracking pad reference for LAUNCH_CLIMB_HOLD_MS. The reference
// stops following while above the line, so it can't chase a real climb.
static bool climb_detected(FlightSM *fsm, float baro_alt_m, float dt_ms, uint32_t now) {
    if (isnan(baro_alt_m)) return false;
    if (isnan(fsm->climb_ref_m)) { fsm->climb_ref_m = baro_alt_m; fsm->climb_since_ms = 0; }

    if (baro_alt_m - fsm->climb_ref_m >= LAUNCH_CLIMB_M) {
        if (fsm->climb_since_ms == 0) fsm->climb_since_ms = now;
        return now - fsm->climb_since_ms >= LAUNCH_CLIMB_HOLD_MS;
    }
    fsm->climb_since_ms = 0;
    fsm->climb_ref_m += dt_ms / (LAUNCH_CLIMB_REF_TAU_MS + dt_ms) * (baro_alt_m - fsm->climb_ref_m);
    return false;
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
    fsm->peak_altitude_m         = 0.0f;
    fsm->drogue_fired            = false;
    fsm->main_fired              = false;
    fsm->tvc_enabled             = false;
    fsm->launch_armed            = false;
    fsm->launch_seen             = false;
    fsm->launch_accel_g          = 1.0f;   // at rest on the rail
    fsm->last_update_ms          = millis();
    fsm->climb_ref_m             = NAN;
    fsm->climb_since_ms          = 0;
    fsm->imu_fault               = false;
    fsm->resumed                 = false;

    Serial.println("[FSM] Initialized. Close SW401 to arm.");
}

void fsm_set_armed(FlightSM *fsm, bool armed) {
    if (armed && fsm->state == STATE_IDLE) {
        enter_state(fsm, STATE_ARMED);   // state_entry_ms starts the ARM_SETTLE_MS wait
        fsm->launch_armed = false;
        fsm->launch_seen  = false;
        Serial.print("[FSM] Armed — launch detection in ");
        Serial.print(ARM_SETTLE_MS / 1000);
        Serial.println(" s.");
    } else if (!armed && fsm->state == STATE_ARMED) {
        enter_state(fsm, STATE_IDLE);
        fsm->launch_armed = false;
    }
    // Any other state: launch already latched or flight is underway —
    // the switch's ARM_SENSE reading is logged upstream but ignored here.
}

void fsm_resume(FlightSM *fsm, FlightState saved, uint32_t ms_since_launch,
                float peak_altitude_m, bool flight_proven) {
    uint32_t now = millis();

    fsm->resumed          = true;
    fsm->tvc_enabled      = false;
    fsm->powered_entry_ms = now - ms_since_launch;   // keeps the G7 timeout on the original clock
    fsm->peak_altitude_m  = peak_altitude_m;
    // Only the G3 gate reads this — carry over whether it had been passed.
    fsm->peak_velocity_ms = flight_proven ? MIN_FLIGHT_VELOCITY_MS + 1.0f : 0.0f;

    Serial.print("[FSM] WATCHDOG RESET IN FLIGHT — resuming from ");
    Serial.print(STATE_NAMES[saved]);
    Serial.print(", ");
    Serial.print(ms_since_launch);
    Serial.println(" ms after launch.");

    FlightState target = (saved == STATE_POWERED || saved == STATE_COAST) ? STATE_COAST : STATE_MAIN;
    enter_state(fsm, target);
    fsm->prev_state = STATE_IDLE;   // let fsm_state_changed() report the entry
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

void fsm_reset_climb_ref(FlightSM *fsm) {
    fsm->climb_ref_m    = NAN;
    fsm->climb_since_ms = 0;
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
                float baro_alt_m,
                bool  imu_valid) {

    uint32_t now = millis();

    // Filter dt from the call spacing, so a slow tick doesn't change the
    // time constants.
    float dt_ms = (float)(now - fsm->last_update_ms);
    fsm->last_update_ms = now;
    if (dt_ms > 100.0f) dt_ms = 100.0f;

    update_led(now);

    // G2 — low-pass the launch accel (LAUNCH_ACCEL_TAU_MS).
    fsm->launch_accel_g += dt_ms / (LAUNCH_ACCEL_TAU_MS + dt_ms) * (accel_up_g - fsm->launch_accel_g);

    // ── FAULT DETECTION ──────────────────────────────────────────
    // lsm6dsox_read() only reports invalid after several consecutive bad
    // samples (see lsm6dsox.cpp), so this latch is fed a filtered signal,
    // not a single-glitch trigger.
    if (!imu_valid) fsm->imu_fault = true;

    if (fsm->imu_fault) {
        if (fsm->state == STATE_IDLE || fsm->state == STATE_ARMED) {
            // On the pad: nothing has flown yet, and launch detection
            // needs the IMU. Safe everything.
            Serial.println("[FSM] CRITICAL: IMU FAULT ON THE PAD!");
            fsm_abort(fsm);
            return;
        }
        if ((fsm->state == STATE_POWERED || fsm->state == STATE_COAST) &&
            fsm->tvc_enabled) {
            // In flight: stop steering, but keep recovery running.
            // fsm_abort()/pyro_safe_all() must NOT be called here — that
            // would disable the chute along with everything else. Apogee
            // still has the baro-drop and launch-timeout backstops below,
            // independent of the (possibly IMU-derived) velocity estimate.
            Serial.println("[FSM] IMU FAULT IN FLIGHT — TVC disabled, recovery continues.");
            fsm->tvc_enabled = false;
        }
    }

    // ── STATE TRANSITIONS ─────────────────────────────────────────
    switch (fsm->state) {

        // On the pad. Two ways to call liftoff:
        //   G2  accel trigger — ARMED once the G1 settle is over, and IDLE
        //       at any time (G2c unarmed failsafe);
        //   G2b climb backstop — IDLE and ARMED at all times.
        case STATE_IDLE:
        case STATE_ARMED: {
            if (fsm->state == STATE_ARMED && !fsm->launch_armed &&
                fsm_time_in_state(fsm) >= ARM_SETTLE_MS) {
                fsm->launch_armed = true;
                Serial.println("[FSM] LAUNCH DETECTION ARMED. Waiting for sustained thrust.");
                digitalWriteFast(PIN_LED_GREEN, HIGH);
                led_off_ms = now + LAUNCH_ARMED_LED_FLASH_MS;
            }

            if (climb_detected(fsm, baro_alt_m, dt_ms, now)) {
                liftoff(fsm, now, "climb backstop (baro rose 8 m).");
                break;
            }

            bool accel_live = fsm->state == STATE_IDLE || fsm->launch_armed;
            if (!accel_live) break;

            if (fsm->launch_detect_ms == 0) {
                if (fsm->launch_accel_g > LAUNCH_ACCEL_THRESHOLD_G) {
                    fsm->launch_detect_ms = now;
                    fsm->launch_seen      = true;
                    fsm->tvc_enabled      = true;   // steer from first motion, not just after confirmation
                    Serial.println(fsm->state == STATE_IDLE
                        ? "[FSM] >> LAUNCH ACCEL DETECTED WITH SW401 OPEN! TVC live. Waiting for it to hold..."
                        : "[FSM] >> LAUNCH ACCEL DETECTED! TVC live. Waiting for it to hold...");
                }
            } else if (fsm->launch_accel_g < LAUNCH_RELEASE_G) {
                // Not sustained — a knock, not a motor. Detection stays
                // armed, so a real launch right after can still trigger.
                Serial.println("[FSM] >> Accel dropped before the hold time — treating as a bump.");
                fsm->launch_detect_ms = 0;
                fsm->tvc_enabled      = false;
            } else if (now - fsm->launch_detect_ms >= LAUNCH_CONFIRM_MS) {
                liftoff(fsm, now, "sustained thrust.");
            }
            break;
        }

        case STATE_POWERED:
            if (velocity_ms > fsm->peak_velocity_ms)
                fsm->peak_velocity_ms = velocity_ms;
            if (altitude_m > fsm->peak_altitude_m)
                fsm->peak_altitude_m = altitude_m;

            if (accel_up_g < BURNOUT_ACCEL_THRESHOLD_G) {
                Serial.println("[FSM] >> MOTOR BURNOUT DETECTED. Coasting...");
                fsm->tvc_enabled = false;
                enter_state(fsm, STATE_COAST);
            }
            break;

        case STATE_COAST:
            if (velocity_ms > fsm->peak_velocity_ms)
                fsm->peak_velocity_ms = velocity_ms;
            if (altitude_m > fsm->peak_altitude_m)
                fsm->peak_altitude_m = altitude_m;

            {
                uint32_t since_launch = now - fsm->powered_entry_ms;
                bool gate = (fsm->peak_velocity_ms > MIN_FLIGHT_VELOCITY_MS) &&
                            (fsm_time_in_state(fsm) >= COAST_APOGEE_MIN_MS);

                // Velocity is integrated accel: not trusted after a reset (it
                // restarted from zero) or an IMU fault (a dead chip feeds it
                // zeros or a frozen reading, which can cross zero mid-burn).
                // The baro-drop and timeout backstops still run.
                bool vel_ok      = !fsm->resumed && !fsm->imu_fault;
                bool vel_apogee  = gate && vel_ok && (velocity_ms <= 0.0f);   // level, not edge-triggered
                bool baro_apogee = gate && ((fsm->peak_altitude_m - altitude_m) >= APOGEE_BARO_DROP_M);
                bool timeout     = (since_launch >= APOGEE_TIMEOUT_FROM_LAUNCH_MS);   // no extra gates

                if (vel_apogee) {
                    Serial.println("[FSM] >> APOGEE DETECTED: Vertical velocity crossed 0.");
                    enter_state(fsm, STATE_APOGEE);
                } else if (baro_apogee) {
                    Serial.println("[FSM] >> APOGEE DETECTED: Barometric altitude dropped from peak.");
                    enter_state(fsm, STATE_APOGEE);
                } else if (timeout) {
                    Serial.println("[FSM] >> APOGEE DETECTED: Timeout from launch reached.");
                    enter_state(fsm, STATE_APOGEE);
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
                bool still    = !fsm->imu_fault && accel_ok && gyro_ok;
                bool slow     = fabsf(velocity_ms) < LANDED_VELOCITY_MS;
                bool late     = (now - fsm->powered_entry_ms) >= LANDED_FALLBACK_MS;
                bool down     = late ? (still || fsm->imu_fault) : (still && slow);

                if (down) {
                    if (fsm_time_in_state(fsm) >= LANDED_TIME_MS) {
                        Serial.println(late ? "[FSM] >> TOUCHDOWN CONFIRMED (fallback: time since liftoff)."
                                            : "[FSM] >> TOUCHDOWN CONFIRMED.");
                        enter_state(fsm, STATE_LANDED);
                    }
                } else {
                    fsm->state_entry_ms = now;   // restart the LANDED_TIME_MS hold
                }
            }
            break;

        case STATE_LANDED:
        case STATE_ABORT:
            break;
    }

    fsm->prev_velocity_ms = velocity_ms;
}
