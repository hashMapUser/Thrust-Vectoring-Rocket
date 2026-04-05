#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// I2C ADDRESS
// --------------------------------------------------------
#define MMC5603NJ_ADDRESS       0x30   // Fixed I2C address (not configurable)

// --------------------------------------------------------
// REGISTER MAP
// --------------------------------------------------------

// Output data (18-bit per axis, offset binary centered at 2^17)
#define MMC5603NJ_REG_XOUT0    0x00   // X[19:12]
#define MMC5603NJ_REG_XOUT1    0x01   // X[11:4]
#define MMC5603NJ_REG_YOUT0    0x02   // Y[19:12]
#define MMC5603NJ_REG_YOUT1    0x03   // Y[11:4]
#define MMC5603NJ_REG_ZOUT0    0x04   // Z[19:12]
#define MMC5603NJ_REG_ZOUT1    0x05   // Z[11:4]
#define MMC5603NJ_REG_XOUT2    0x06   // X[3:0] in bits [7:4]
#define MMC5603NJ_REG_YOUT2    0x07   // Y[3:0] in bits [7:4]
#define MMC5603NJ_REG_ZOUT2    0x08   // Z[3:0] in bits [7:4]
#define MMC5603NJ_REG_TOUT     0x09   // Temperature output

// Status
#define MMC5603NJ_REG_STATUS1  0x18

// Configuration
#define MMC5603NJ_REG_ODR      0x1A   // Output data rate (continuous mode)
#define MMC5603NJ_REG_CTRL0    0x1B   // Internal control 0
#define MMC5603NJ_REG_CTRL1    0x1C   // Internal control 1
#define MMC5603NJ_REG_CTRL2    0x1D   // Internal control 2

// Product ID (WHO_AM_I equivalent)
#define MMC5603NJ_REG_PROD_ID  0x39   // Expected value: 0x10

// --------------------------------------------------------
// REGISTER BIT MASKS
// --------------------------------------------------------

// CTRL0 bits
#define MMC5603NJ_TM_M         0x01   // Trigger a single magnetic measurement
#define MMC5603NJ_SET_COIL     0x08   // Fire SET coil (removes residual magnetization)
#define MMC5603NJ_RESET_COIL   0x10   // Fire RESET coil

// STATUS1 bits
#define MMC5603NJ_MEAS_M_DONE  0x40   // 1 = magnetic measurement complete, safe to read

// --------------------------------------------------------
// CONSTANTS
// --------------------------------------------------------
#define MMC5603NJ_CHIP_ID      0x10

// 18-bit output is offset binary: 0 field = 2^17 = 131072
// Subtract this to get a signed value before scaling
#define MMC5603NJ_ZERO_OFFSET  131072

// Sensitivity: 16384 LSB/Gauss in 16-bit mode → *4 in 18-bit = 65536 LSB/Gauss
// Scale factor to convert raw → Gauss:
#define MMC5603NJ_SCALE        (1.0f / 65536.0f)  // Gauss per LSB

#define MMC5603NJ_I2C_CLOCK    400000UL

// Max time to wait for Meas_M_Done after triggering (datasheet: ~8 ms typical)
#define MMC5603NJ_MEAS_TIMEOUT_MS 10

// --------------------------------------------------------
// DATA STRUCT
// --------------------------------------------------------

/**
 * One magnetometer sample.
 *   mag_x/y/z — magnetic field strength [Gauss]
 *   valid      — false if I2C failed or measurement timed out
 */
typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
    bool  valid;
} MMC5603NJ_Data;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Verify chip ID, fire SET coil to clear residual magnetization,
 * and leave sensor ready for on-demand measurement.
 * Wire.begin() must be called first.
 *
 * @return true on success; false if sensor absent or WHO_AM_I mismatch.
 */
bool mmc5603nj_init();

/**
 * Trigger a single measurement, poll until complete, then burst-read
 * all 9 output bytes (X, Y, Z in 18-bit) and scale to Gauss.
 *
 * @param out  Populated on return. Always check out->valid.
 */
void mmc5603nj_read(MMC5603NJ_Data *out);