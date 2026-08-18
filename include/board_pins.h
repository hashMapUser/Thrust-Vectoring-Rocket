#pragma once

// Teensy 4.0 pin assignments for FC V2.
// This is the ONLY file that defines pin numbers.
// Do not add #define *_PIN anywhere else.

// --- SPI0 (IMU — LSM6DSOX) ---
#define PIN_IMU_CS       10
#define PIN_IMU_MOSI     11
#define PIN_IMU_MISO     12
#define PIN_IMU_SCK      13

// --- SPI1 (microSD — DO NOT INSERT CARD, H6: SD VDD wired to 5V) ---
#define PIN_SD_CS        0
#define PIN_SD_MISO      1
#define PIN_SD_MOSI      26
#define PIN_SD_SCK       27
#define PIN_SD_CD        7    // card detect — INPUT_PULLDOWN; floats with no card

// --- SPI2 (NOR flash — GD25Q128) ---
#define PIN_FLASH_MISO   34
#define PIN_FLASH_MOSI   35
#define PIN_FLASH_CS     36
#define PIN_FLASH_SCK    37

// --- I2C0 Wire (barometer — LPS22HB) ---
#define PIN_BARO_SDA     18
#define PIN_BARO_SCL     19

// --- I2C1 Wire1 (magnetometer — MMC5603NJ, not fitted this flight) ---
#define PIN_MAG_SCL      16
#define PIN_MAG_SDA      17

// --- PWM (TVC servos) ---
#define PIN_SERVO_X      5
#define PIN_SERVO_Y      6

// --- Digital outputs ---
#define PIN_BUZZER       3
// Measured resonant peak for this board's transducer.
// Sweep 1500-4500 Hz on the bench and update to the loudest frequency.
#define BUZZER_FREQ_HZ   3450   /* CMT-1203 nominal — measure and tune */
#define PIN_PYRO1_FIRE   32   // main chute (single-deploy flight)
#define PIN_PYRO2_FIRE   31   // unused this flight

// --- Status LEDs ---
#define PIN_LED_GREEN    4
#define PIN_LED_WHITE    33
#define PIN_LED_RED      30

// --- Arming sense (analog in, A10) ---
// ~2.69 V armed (SW401 closed, PYRO_PWR present), 0 V safe
#define PIN_ARM_SENSE    24

// --- Documented-only pins (do not drive during flight) ---
// PIN_BAT_VOLTAGE    = 14  (A0, analog — no firmware use this revision)
// PIN_PYRO1_SENSE    = 22  (A8, no continuity telemetry per H5)
// PIN_PYRO2_SENSE    = 23  (A9, no continuity telemetry per H5)
