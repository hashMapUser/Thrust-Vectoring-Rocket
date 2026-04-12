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

    // --- CS pin MUST be driven HIGH before SPI.begin() ---
    // If CS floats during bus init the LSM6DSOX receives garbage
    // and will return 0x00 on WHO_AM_I regardless of mode or clock.
    pinMode(LSM6DSOX_CS_PIN, OUTPUT);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);

    SPI.begin();
    Wire.begin();
    delay(100);

    run_imu_tests();
    run_baro_tests(&bmp_cal);

    Serial.println("All tests complete. Reboot to run again.");
}

void loop() {}