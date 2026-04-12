#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <SPI.h>

// --------------------------------------------------------
// SPI CONFIG
// --------------------------------------------------------

// --------------------------------------------------------
// PIN ASSIGNMENTS (Teensy 4.0 default SPI bus)
// --------------------------------------------------------
#define LSM6DSOX_CS_PIN       10   // Chip select — your wiring
#define LSM6DSOX_PIN_MOSI     11   // MOSI        — Teensy 4.0 hardware SPI default
#define LSM6DSOX_PIN_MISO     12   // MISO        — Teensy 4.0 hardware SPI default
#define LSM6DSOX_PIN_SCK      13   // Clock       — Teensy 4.0 hardware SPI default

// SPI Mode 3 (CPOL=1, CPHA=1) — confirmed working on Teensy 4.0
// 10 MHz is the sensor max and works reliably on the Teensy at this speed
#define LSM6DSOX_SPI_CLOCK    10000000UL
#define LSM6DSOX_SPI_SETTINGS SPISettings(LSM6DSOX_SPI_CLOCK, MSBFIRST, SPI_MODE3)

// SPI R/W flags (applied to bit 7 of the register address byte)
#define LSM6DSOX_SPI_READ     0x80   // bit 7 high  = read
#define LSM6DSOX_SPI_WRITE    0x00   // bit 7 low   = write

// --------------------------------------------------------
// REGISTER MAP
// --------------------------------------------------------
#define LSM6DSOX_REG_WHO_AM_I  0x0F   // expected: 0x6C
#define LSM6DSOX_REG_CTRL1_XL  0x10   // accelerometer ODR + full-scale
#define LSM6DSOX_REG_CTRL2_G   0x11   // gyroscope ODR + full-scale
#define LSM6DSOX_REG_CTRL3_C   0x12   // misc — IF_INC (auto-increment) lives here
#define LSM6DSOX_REG_STATUS    0x1E

// Output data — burst from OUTX_L_G to get gyro + accel in 12 bytes:
//   0x22–0x27  gyro  X/Y/Z (low byte first)
//   0x28–0x2D  accel X/Y/Z (low byte first)
#define LSM6DSOX_REG_OUTX_L_G  0x22

// --------------------------------------------------------
// REGISTER VALUES
// --------------------------------------------------------
#define LSM6DSOX_CHIP_ID          0x6C

// CTRL3_C: BDU (0x40) + IF_INC (0x04) = 0x44
//   BDU  — block data update: output registers not updated until both
//           high and low bytes have been read; prevents stale half-reads
//   IF_INC — auto-increment register address during burst reads
#define LSM6DSOX_CTRL3_INIT   0x44

// Software reset bit in CTRL3_C — write this first, wait 50 ms, then configure
#define LSM6DSOX_SW_RESET     0x01

// CTRL1_XL: ODR_XL=0111 (833 Hz), FS_XL=01 (±16 g), LPF2=0
// ±16 g chosen for flight — rockets exceed ±4 g during thrust
#define LSM6DSOX_XL_833HZ_16G    0x74

// CTRL2_G: ODR_G=0111 (833 Hz), FS_G=11 (±2000 dps)
#define LSM6DSOX_G_833HZ_2000DPS 0x7C

// --------------------------------------------------------
// SENSITIVITY
// --------------------------------------------------------
#define LSM6DSOX_GYRO_SCALE   0.070f     // dps per LSB  (±2000 dps)
#define LSM6DSOX_ACCEL_SCALE  0.000488f  // g   per LSB  (±16 g)

// --------------------------------------------------------
// DATA STRUCT
// --------------------------------------------------------

/**
 * One complete IMU sample.
 *   gx/gy/gz — angular rate  [deg/s]   (NOT roll/pitch/yaw angles)
 *   ax/ay/az — linear accel  [g]
 *   valid    — false if any SPI error occurred
 */
typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
    bool  valid;
} LSM6DSOX_Data;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * IMPORTANT — call this before SPI.begin():
 *   pinMode(LSM6DSOX_CS_PIN, OUTPUT);
 *   digitalWrite(LSM6DSOX_CS_PIN, HIGH);
 *
 * If CS floats during SPI.begin() the sensor receives garbage and won't respond.
 *
 * Issues a software reset, verifies WHO_AM_I, then configures accel + gyro.
 *
 * @return true on success; false if WHO_AM_I mismatches.
 */
bool lsm6dsox_init();

/**
 * Burst-read 12 output bytes (gyro then accel) in a single SPI transaction.
 * Scales results to deg/s and g.
 *
 * @param out  Populated on return.  Always check out->valid.
 */
void lsm6dsox_read(LSM6DSOX_Data *out);