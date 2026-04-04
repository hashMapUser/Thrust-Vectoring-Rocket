#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <SPI.h>

// --------------------------------------------------------
// SPI CONFIG
// --------------------------------------------------------

// Chip-select pin — change to match your wiring
#define LSM6DSOX_CS_PIN       10

// LSM6DSOX max SPI clock: 10 MHz
// Mode 3 (CPOL=1, CPHA=1) — sensor supports modes 0 and 3; 3 is preferred
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

// CTRL1_XL: ODR_XL=0100 (104 Hz), FS_XL=10 (±4 g), LPF2=0
#define LSM6DSOX_XL_104HZ_4G     0x48

// CTRL2_G:  ODR_G=0100 (104 Hz), FS_G=11 (±2000 dps)
#define LSM6DSOX_G_104HZ_2000DPS  0x4C

// CTRL3_C:  IF_INC=1 — enable register auto-increment during burst reads
#define LSM6DSOX_IF_INC           0x04

// --------------------------------------------------------
// SENSITIVITY
// --------------------------------------------------------
#define LSM6DSOX_GYRO_SCALE   0.070f     // dps per LSB  (±2000 dps)
#define LSM6DSOX_ACCEL_SCALE  0.000122f  // g   per LSB  (±4 g)

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
 * Configure the CS pin, verify chip ID, and set up accel + gyro.
 * SPI.begin() must be called before this.
 *
 * @return true on success; false if sensor absent or WHO_AM_I mismatch.
 */
bool lsm6dsox_init();

/**
 * Burst-read 12 output bytes (gyro then accel) in a single SPI transaction.
 * Scales results to deg/s and g.
 *
 * @param out  Populated on return.  Always check out->valid.
 */
void lsm6dsox_read(LSM6DSOX_Data *out);