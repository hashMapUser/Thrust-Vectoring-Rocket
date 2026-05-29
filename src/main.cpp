#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "bmp390.h"
#include "lsm6dsox.h"
#include "flight_sm.h"
#include "test_imu.h"
#include "test_baro.h"

// Remove-before-flight (RBF) arming pin.
// Wire the RBF jumper between this pin and GND.
//   Jumper IN  (pin pulled LOW via jumper)  → IDLE / safe
//   Jumper OUT (pin floats HIGH via pullup) → ARMED
// Change this to any free digital pin on your Teensy.
#define ARM_SWITCH_PIN  2

static BMP390_Calib bmp_cal;
static FlightSM     fsm;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

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
    run_baro_tests(&bmp_cal);

    fsm_init(&fsm);
    Serial.println("All tests complete.");
    Serial.println("Pull RBF jumper to arm. Re-insert to disarm. Send 'D' or 'X' over serial for emergency disarm/abort.");
}

void loop() {

}
