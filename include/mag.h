#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// I2C ADDRESS
// --------------------------------------------------------
#define MMC5603NJ_ADDRESS        0x30   // Fixed — not configurable

// --------------------------------------------------------
// REGISTER MAP
// --------------------------------------------------------

// Output data — burst 9 bytes from 0x00 to get all three axes
// Each axis is 18-bit, spread across 2 full bytes + 4 bits in a shared byte
#define MMC5603NJ_REG_XOUT0     0x00   // X[19:12]
#define MMC5603NJ_REG_XOUT1     0x01   // X[11:4]
#define MMC5603NJ_REG_YOUT0     0x02   // Y[19:12]
#define MMC5603NJ_REG_YOUT1     0x03   // Y[11:4]
#define MMC5603NJ_REG_ZOUT0     0x04   // Z[19:12]
#define MMC5603NJ_REG_ZOUT1     0x05   // Z[11:4]
#define MMC5603NJ_REG_XOUT2     0x06   // X[3:0] in bits [7:4]
#define MMC5603NJ_REG_YOUT2     0x07   // Y[3:0] in bits [7:4]
#define MMC5603NJ_REG_ZOUT2     0x08   // Z[3:0] in bits [7:4]

// Status and control
#define MMC5603NJ_REG_STATUS1   0x18
#define MMC5603NJ_REG_ODR       0x1A   // Output data rate (continuous mode)
#define MMC5603NJ_REG_CTRL0     0x1B   // Internal control 0
#define MMC5603NJ_REG_CTRL1     0x1C   // Internal control 1
#define MMC5603NJ_REG_CTRL2     0x1D   // Internal control 2

// Product ID — WHO_AM_I equivalent
#define MMC5603NJ_REG_PROD_ID   0x39   // Expected: 0x10

// --------------------------------------------------------
// REGISTER BIT MASKS
// --------------------------------------------------------

// CTRL0 bits
#define MMC5603NJ_TM_M          0x01   // Trigger one magnetic measurement
#define MMC5603NJ_SET_COIL      0x08   // Fire SET coil (removes +offset from residual magnetization)
#define MMC5603NJ_RESET_COIL    0x10   // Fire RESET coil (removes -offset)

// STATUS1 bits
#define MMC5603NJ_MEAS_M_DONE   0x40   // 1 = measurement complete, safe to read

// --------------------------------------------------------
// CONSTANTS
// --------------------------------------------------------
#define MMC5603NJ_CHIP_ID       0x10

// 18-bit output is offset binary — zero field = 2^17 = 131072
// Subtract this before scaling to get a signed value
#define MMC5603NJ_ZERO_OFFSET   131072

// Sensitivity: 16384 LSB/Gauss in 18-bit mode
// Invert to get scale factor: Gauss per LSB
#define MMC5603NJ_SCALE         (1.0f / 16384.0f)

// I2C fast-mode clock
#define MMC5603NJ_I2C_CLOCK     400000UL

// Max poll time waiting for Meas_M_Done (datasheet typ: 8 ms)
#define MMC5603NJ_MEAS_TIMEOUT_MS 15

// --------------------------------------------------------
// PIN ASSIGNMENTS (Teensy 4.0 Wire1 bus)
// MMC5603NJ is on a separate I2C bus from the BMP390 (Wire / pins 18-19)
// --------------------------------------------------------
#define MMC5603NJ_PIN_SDA       17   // Wire1 SDA — Teensy 4.0
#define MMC5603NJ_PIN_SCL       16   // Wire1 SCL — Teensy 4.0

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
 * Verify chip ID, fire SET coil to remove residual magnetization,
 * and prepare sensor for on-demand measurements.
 * Wire must already be initialized (BMP390 init handles this on the same bus).
 *
 * @return true on success; false if sensor absent or chip ID mismatch.
 */
bool mmc5603nj_init();

/**
 * Trigger a single measurement, poll until complete, then burst-read
 * all 9 output bytes and assemble 18-bit values for each axis.
 *
 * @param out  Populated on return. Always check out->valid.
 */
void mmc5603nj_read(MMC5603NJ_Data *out);