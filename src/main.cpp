#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "bmp390.h"
#include "lsm6dsox.h"
#include "mmc5603nj.h"
#include "test_imu.h"
#include "test_baro.h"
#include "test_mag.h"

static BMP390_Calib bmp_cal;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    // --- CS must be HIGH before SPI.begin() ---
    pinMode(LSM6DSOX_CS_PIN, OUTPUT);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);

    SPI.begin();
    // Wire  (pins 18/19) — BMP390      — initialized inside bmp390_init()
    // Wire1 (pins 16/17) — MMC5603NJ   — initialized inside mmc5603nj_init()

    delay(100);

    Serial.println("\n========================================");
    Serial.println("  FLIGHT COMPUTER SENSOR TEST SUITE");
    Serial.println("  Teensy 4.0 / BMP390 + LSM6DSOX + MMC5603NJ");
    Serial.println("========================================\n");

    run_imu_tests();
    run_baro_tests(&bmp_cal);
    run_mag_tests();

    Serial.println("All tests complete. Reboot to run again.");
}

void loop() {}