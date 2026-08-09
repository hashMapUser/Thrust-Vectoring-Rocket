#include <Arduino.h>
#include <EEPROM.h>
#include "pyro.h"

// --------------------------------------------------------
// PRIVATE — EEPROM helpers
// --------------------------------------------------------

static void eeprom_save_fired(const PyroState *pyro) {
    EEPROM.put(PYRO_EEPROM_ADDR, (uint16_t)PYRO_EEPROM_MAGIC);
    EEPROM.put(PYRO_EEPROM_ADDR + 2, pyro->drogue_fired);
    EEPROM.put(PYRO_EEPROM_ADDR + 3, pyro->main_fired);
}

static void eeprom_load_fired(PyroState *pyro) {
    uint16_t magic;
    EEPROM.get(PYRO_EEPROM_ADDR, magic);
    if (magic != PYRO_EEPROM_MAGIC) {
        pyro->drogue_fired = false;
        pyro->main_fired   = false;
        return;
    }
    bool df, mf;
    EEPROM.get(PYRO_EEPROM_ADDR + 2, df);
    EEPROM.get(PYRO_EEPROM_ADDR + 3, mf);
    pyro->drogue_fired = df;
    pyro->main_fired   = mf;

    if (df) Serial.println("[PYRO] EEPROM: drogue previously fired");
    if (mf) Serial.println("[PYRO] EEPROM: main previously fired");
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void pyro_init(PyroState *pyro) {
    pyro->drogue_armed         = false;
    pyro->main_armed           = false;
    pyro->drogue_firing        = false;
    pyro->main_firing          = false;
    pyro->drogue_fire_start_ms = 0;
    pyro->main_fire_start_ms   = 0;

    // Drive outputs LOW BEFORE enabling the output driver.
    // On i.MX RT1062, pinMode(OUTPUT) enables the pad against the data register;
    // if the data register has a stale HIGH the pin glitches high for one cycle.
    digitalWrite(PIN_PYRO1_FIRE, LOW);
    digitalWrite(PIN_PYRO2_FIRE, LOW);
    pinMode(PIN_PYRO1_FIRE, OUTPUT);
    pinMode(PIN_PYRO2_FIRE, OUTPUT);

    // Restore fired flags from EEPROM — prevents re-arming a spent channel after brownout.
    eeprom_load_fired(pyro);
}

bool pyro_arm(PyroState *pyro) {
    // Continuity sensing disabled (H5: sense pins have no ADC this flight).
    // ARM_SENSE gate is enforced upstream in fsm_arm() / main loop.
    pyro->drogue_armed = true;
    pyro->main_armed   = true;
    Serial.println("[PYRO] Armed");
    return true;
}

void pyro_disarm(PyroState *pyro) {
    pyro->drogue_armed = false;
    pyro->main_armed   = false;
    digitalWrite(PIN_PYRO1_FIRE, LOW);
    digitalWrite(PIN_PYRO2_FIRE, LOW);
    pyro->drogue_firing = false;
    pyro->main_firing   = false;
    Serial.println("[PYRO] Disarmed");
}

void pyro_fire_drogue(PyroState *pyro) {
    if (!pyro->drogue_armed) { Serial.println("[PYRO] Drogue fire blocked — not armed"); return; }
    if (pyro->drogue_fired)  { Serial.println("[PYRO] Drogue already fired"); return; }

    digitalWrite(PIN_PYRO2_FIRE, HIGH);
    pyro->drogue_firing        = true;
    pyro->drogue_fire_start_ms = millis();
    pyro->drogue_fired         = true;
    eeprom_save_fired(pyro);
    Serial.println("[PYRO] DROGUE FIRED");
}

void pyro_fire_main(PyroState *pyro, float altitude_m) {
    if (!pyro->main_armed)  { Serial.println("[PYRO] Main fire blocked — not armed"); return; }
    if (pyro->main_fired)   { Serial.println("[PYRO] Main already fired"); return; }
    if (altitude_m < PYRO_MAIN_MIN_ALT_M) {
        Serial.print("[PYRO] Main fire blocked — altitude too low: ");
        Serial.println(altitude_m);
        return;
    }

    digitalWrite(PIN_PYRO1_FIRE, HIGH);
    pyro->main_firing        = true;
    pyro->main_fire_start_ms = millis();
    pyro->main_fired         = true;
    eeprom_save_fired(pyro);
    Serial.println("[PYRO] MAIN FIRED");
}

void pyro_update(PyroState *pyro) {
    uint32_t now = millis();

    if (pyro->drogue_firing &&
        (now - pyro->drogue_fire_start_ms) >= PYRO_FIRE_DURATION_MS) {
        digitalWrite(PIN_PYRO2_FIRE, LOW);
        pyro->drogue_firing = false;
        Serial.println("[PYRO] Drogue pulse complete");
    }

    if (pyro->main_firing &&
        (now - pyro->main_fire_start_ms) >= PYRO_FIRE_DURATION_MS) {
        digitalWrite(PIN_PYRO1_FIRE, LOW);
        pyro->main_firing = false;
        Serial.println("[PYRO] Main pulse complete");
    }
}

void pyro_safe_all(PyroState *pyro) {
    digitalWrite(PIN_PYRO1_FIRE, LOW);
    digitalWrite(PIN_PYRO2_FIRE, LOW);
    pyro->drogue_armed  = false;
    pyro->main_armed    = false;
    pyro->drogue_firing = false;
    pyro->main_firing   = false;
    Serial.println("[PYRO] ALL OUTPUTS SAFED");
}
