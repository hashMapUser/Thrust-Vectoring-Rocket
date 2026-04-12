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


    // Raw SPI probe — bypass driver completely
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);
    SPI.begin();
    delay(100);

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    digitalWrite(10, LOW);
    SPI.transfer(0x0F | 0x80);          // WHO_AM_I read
    uint8_t raw_id = SPI.transfer(0x00);
    digitalWrite(10, HIGH);
    SPI.endTransaction();

Serial.print("RAW WHO_AM_I: 0x"); Serial.println(raw_id, HEX);
while(true);  // halt so you can read the result

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