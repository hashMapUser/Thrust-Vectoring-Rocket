#include <Arduino.h>
#include "pyro.h"

void pyro_init(PyroState *pyro) {
    pyro->drogue_continuity     = false;
    pyro->main_continuity       = false;
    pyro->drogue_armed          = false;
    pyro->main_armed            = false;
    pyro->drogue_fired          = false;
    pyro->main_fired            = false;
    pyro->drogue_fire_start_ms  = 0;
    pyro->main_fire_start_ms    = 0;

    // Outputs start LOW — never HIGH until intentionally fired
    pinMode(PYRO_DROGUE_PIN, OUTPUT);
    pinMode(PYRO_MAIN_PIN,   OUTPUT);
    digitalWrite(PYRO_DROGUE_PIN, LOW);
    digitalWrite(PYRO_MAIN_PIN,   LOW);

    pinMode(PYRO_DROGUE_CONT_PIN, INPUT);
    pinMode(PYRO_MAIN_CONT_PIN,   INPUT);

    pyro_check_continuity(pyro);

    Serial.print("[PYRO] Drogue continuity: ");
    Serial.println(pyro->drogue_continuity ? "GOOD" : "OPEN");
    Serial.print("[PYRO] Main continuity:   ");
    Serial.println(pyro->main_continuity   ? "GOOD" : "OPEN");
}

void pyro_check_continuity(PyroState *pyro) {
    int drogue_adc = analogRead(PYRO_DROGUE_CONT_PIN);
    int main_adc   = analogRead(PYRO_MAIN_CONT_PIN);

    pyro->drogue_continuity = (drogue_adc > PYRO_CONT_GOOD);
    pyro->main_continuity   = (main_adc   > PYRO_CONT_GOOD);
}

bool pyro_arm(PyroState *pyro) {
    pyro_check_continuity(pyro);

    if (!pyro->drogue_continuity) {
        Serial.println("[PYRO] ARM REJECTED — drogue continuity failed");
        return false;
    }
    if (!pyro->main_continuity) {
        Serial.println("[PYRO] ARM REJECTED — main continuity failed");
        return false;
    }

    pyro->drogue_armed = true;
    pyro->main_armed   = true;
    Serial.println("[PYRO] Armed — both channels ready");
    return true;
}

void pyro_disarm(PyroState *pyro) {
    pyro->drogue_armed = false;
    pyro->main_armed   = false;
    digitalWrite(PYRO_DROGUE_PIN, LOW);
    digitalWrite(PYRO_MAIN_PIN,   LOW);
    Serial.println("[PYRO] Disarmed");
}

void pyro_fire_drogue(PyroState *pyro) {
    if (!pyro->drogue_armed) {
        Serial.println("[PYRO] Drogue fire blocked — not armed");
        return;
    }
    if (pyro->drogue_fired) {
        Serial.println("[PYRO] Drogue already fired");
        return;
    }

    digitalWrite(PYRO_DROGUE_PIN, HIGH);
    pyro->drogue_fire_start_ms = millis();
    pyro->drogue_fired         = true;
    Serial.println("[PYRO] DROGUE FIRED");
}

void pyro_fire_main(PyroState *pyro, float altitude_m) {
    if (!pyro->main_armed) {
        Serial.println("[PYRO] Main fire blocked — not armed");
        return;
    }
    if (pyro->main_fired) {
        Serial.println("[PYRO] Main already fired");
        return;
    }
    if (altitude_m < PYRO_MAIN_MIN_ALT_M) {
        Serial.print("[PYRO] Main fire blocked — altitude too low: ");
        Serial.println(altitude_m);
        return;
    }

    digitalWrite(PYRO_MAIN_PIN, HIGH);
    pyro->main_fire_start_ms = millis();
    pyro->main_fired         = true;
    Serial.println("[PYRO] MAIN FIRED");
}

void pyro_update(PyroState *pyro) {
    uint32_t now = millis();

    // Cut drogue pin after fire duration
    if (pyro->drogue_fired &&
        digitalRead(PYRO_DROGUE_PIN) == HIGH &&
        (now - pyro->drogue_fire_start_ms) >= PYRO_FIRE_DURATION_MS) {
        digitalWrite(PYRO_DROGUE_PIN, LOW);
        Serial.println("[PYRO] Drogue pulse complete");
    }

    // Cut main pin after fire duration
    if (pyro->main_fired &&
        digitalRead(PYRO_MAIN_PIN) == HIGH &&
        (now - pyro->main_fire_start_ms) >= PYRO_FIRE_DURATION_MS) {
        digitalWrite(PYRO_MAIN_PIN, LOW);
        Serial.println("[PYRO] Main pulse complete");
    }
}

void pyro_safe_all(PyroState *pyro) {
    digitalWrite(PYRO_DROGUE_PIN, LOW);
    digitalWrite(PYRO_MAIN_PIN,   LOW);
    pyro->drogue_armed = false;
    pyro->main_armed   = false;
    Serial.println("[PYRO] ALL OUTPUTS SAFED");
}