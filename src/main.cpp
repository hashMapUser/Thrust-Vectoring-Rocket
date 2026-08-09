#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "lps22hb.h"
#include "lsm6dsox.h"
#include "flight_sm.h"
#include "test_imu.h"
#include "test_baro.h"

// ARM_SWITCH_PIN removed — see board_pins.h PIN_ARM_SENSE (pin 24, analog in).

static FlightSM fsm;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    // CS pin MUST be driven HIGH before SPI.begin().
    // If CS floats during bus init the LSM6DSOX receives garbage and
    // returns 0x00 on WHO_AM_I regardless of SPI mode or clock speed.
    pinMode(LSM6DSOX_CS_PIN, OUTPUT);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);

    SPI.begin();
    Wire.begin();
    delay(100);

    run_imu_tests();
    run_baro_tests();

    fsm_init(&fsm);
    Serial.println("All tests complete.");
    Serial.println("Raise ARM_SENSE (pin 24) to arm. Send 'D' or 'X' over serial for emergency disarm/abort.");
}

void loop() {

}
