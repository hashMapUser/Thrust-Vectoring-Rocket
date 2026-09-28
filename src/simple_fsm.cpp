#include <Arduino.h>
#include "simple_fsm.h"

const char * const SIMPLE_STATE_NAMES[4] = { "IDLE", "ASCENT", "RECOVERY", "LANDED" };

static void enter_state(SimpleFSM *fsm, SimpleState new_state) {
    fsm->prev_state     = fsm->state;
    fsm->state          = new_state;
    fsm->state_entry_ms = millis();

    Serial.print("[SIMPLE_FSM] ");
    Serial.print(SIMPLE_STATE_NAMES[fsm->prev_state]);
    Serial.print(" -> ");
    Serial.println(SIMPLE_STATE_NAMES[new_state]);
}

void simple_fsm_init(SimpleFSM *fsm, float ground_altitude_m) {
    fsm->state             = SIMPLE_STATE_IDLE;
    fsm->prev_state        = SIMPLE_STATE_IDLE;
    fsm->state_entry_ms    = millis();
    fsm->ground_altitude_m = ground_altitude_m;
    fsm->peak_altitude_m   = ground_altitude_m;
    fsm->chute_fired       = false;
}

bool simple_fsm_state_changed(SimpleFSM *fsm) {
    if (fsm->state != fsm->prev_state) {
        fsm->prev_state = fsm->state;
        return true;
    }
    return false;
}

void simple_fsm_update(SimpleFSM *fsm, PyroState *pyro,
                       float accel_up_g, float accel_mag_g, float gyro_rate_dps,
                       float altitude_m, uint32_t now_ms) {

    if (altitude_m > fsm->peak_altitude_m) fsm->peak_altitude_m = altitude_m;

    switch (fsm->state) {

        case SIMPLE_STATE_IDLE: {
            bool high_g  = accel_up_g > SIMPLE_LAUNCH_ACCEL_G;
            bool climbed = (altitude_m - fsm->ground_altitude_m) > SIMPLE_LAUNCH_ALT_M;
            if (high_g && climbed) {
                Serial.println("[SIMPLE_FSM] Launch detected.");
                enter_state(fsm, SIMPLE_STATE_ASCENT);
            }
            break;
        }

        case SIMPLE_STATE_ASCENT: {
            float drop = fsm->peak_altitude_m - altitude_m;
            if (drop > SIMPLE_RECOVERY_DROP_M) {
                Serial.print("[SIMPLE_FSM] Apogee detected. Peak=");
                Serial.print(fsm->peak_altitude_m, 1);
                Serial.println(" m. Firing recovery chute.");
                pyro_fire_main(pyro, altitude_m);   // single-deploy: PYRO_MAIN_MIN_ALT_M is 0, no floor to enforce
                fsm->chute_fired = true;
                enter_state(fsm, SIMPLE_STATE_RECOVERY);
            }
            break;
        }

        case SIMPLE_STATE_RECOVERY: {
            bool accel_ok = (accel_mag_g >= SIMPLE_LANDED_ACCEL_LOW_G &&
                              accel_mag_g <= SIMPLE_LANDED_ACCEL_HIGH_G);
            bool gyro_ok  = (gyro_rate_dps < SIMPLE_LANDED_GYRO_DPS);

            if (accel_ok && gyro_ok) {
                if ((now_ms - fsm->state_entry_ms) >= SIMPLE_LANDED_HOLD_MS)
                    enter_state(fsm, SIMPLE_STATE_LANDED);
            } else {
                fsm->state_entry_ms = now_ms;   // reset the settle timer while still moving
            }
            break;
        }

        case SIMPLE_STATE_LANDED:
            break;
    }
}
