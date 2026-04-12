#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "bmp390.h"
#include "lsm6dsox.h"
#include "test_imu.h"
#include "test_baro.h"

static BMP390_Calib bmp_cal;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Wire.begin();
    SPI.begin();
    delay(100);

    Serial.println("\n========================================");
    Serial.println("  FLIGHT COMPUTER SENSOR TEST SUITE");
    Serial.println("  Teensy 4.0 / BMP390 + LSM6DSOX");
    Serial.println("========================================\n");

    run_imu_tests();
    run_baro_tests(&bmp_cal);

    Serial.println("All tests complete. Reboot to run again.");
}

void loop() {
    // Tests run once in setup() — nothing to do here
}