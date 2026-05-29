#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "lsm6dsox.h"
#include "flight_sm.h"
#include "pyro.h"
#include "test_imu.h"

// Remove-before-flight (RBF) arming pin.
// Wire the RBF jumper between this pin and GND.
//   Jumper IN  (pin pulled LOW via jumper)  → IDLE / safe
//   Jumper OUT (pin floats HIGH via pullup) → ARMED
// Change this to any free digital pin on your Teensy.
#define ARM_SWITCH_PIN  2

static FlightSM     fsm;
static PyroState    pyros;

void setup() {
    Serial.begin(115200);

    // CS pin MUST be driven HIGH before SPI.begin().
    // If CS floats during bus init the LSM6DSOX receives garbage and
    // returns 0x00 on WHO_AM_I regardless of SPI mode or clock speed.
    pinMode(LSM6DSOX_CS_PIN, OUTPUT);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);

    // Internal pull-up: pin reads HIGH when jumper is absent (armed), LOW when shorted to GND (safe).
    pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);

    SPI.begin();
    Wire.begin();
    delay(100);

    run_imu_tests();

    pyro_init(&pyros);
    fsm_init(&fsm);
    Serial.println("All tests complete.");
    Serial.println("Pull RBF jumper to arm. Re-insert to disarm. Send 'D' or 'X' over serial for emergency disarm/abort.");
}

void loop() {
    // --- RBF arming pin — edge-triggered with 50 ms debounce ---
    static bool     last_stable_high = false;
    static bool     last_raw_high    = false;
    static uint32_t last_edge_ms     = 0;
    const  uint32_t DEBOUNCE_MS      = 50;

    bool raw_high = (digitalRead(ARM_SWITCH_PIN) == HIGH);
    if (raw_high != last_raw_high) {
        last_raw_high = raw_high;
        last_edge_ms  = millis();
    }
    if ((millis() - last_edge_ms) >= DEBOUNCE_MS && raw_high != last_stable_high) {
        last_stable_high = raw_high;
        if (raw_high) {
            if (fsm_arm(&fsm, &pyros)) {
                Serial.println("[ARM] Armed — RBF jumper pulled.");
            } else {
                Serial.println("[ARM] Arm rejected — continuity failed or not in IDLE.");
            }
        } else {
            fsm_disarm(&fsm, &pyros);
            Serial.println("[ARM] Disarmed — RBF jumper re-inserted.");
        }
    }

    // --- Serial safety overrides (disarm and abort only) ---
    if (Serial.available()) {
        char c = Serial.read();
        if      (c == 'D') { fsm_disarm(&fsm, &pyros); Serial.println("[ARM] Disarmed via serial."); }
        else if (c == 'X') { fsm_abort(&fsm);           Serial.println("[ARM] Abort triggered via serial."); }
    }
}
