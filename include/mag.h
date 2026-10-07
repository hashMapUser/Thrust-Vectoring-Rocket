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

// mag_poll(): don't check STATUS1 until this long after the trigger
// (conversion takes ~6.6 ms at the default bandwidth), and start a new
// measurement at most this often — 50 Hz.
#define MMC5603NJ_MEAS_TIME_MS    7
#define MMC5603NJ_POLL_PERIOD_MS  20

// --------------------------------------------------------
// PIN ASSIGNMENTS — from board_pins.h
// Wire1: SCL=PIN_MAG_SCL (16), SDA=PIN_MAG_SDA (17)
// --------------------------------------------------------
#include "board_pins.h"

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
}mag_data;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Verify chip ID, fire SET coil to remove residual magnetization,
 * and prepare sensor for on-demand measurements.
 * Wire must already be initialized (lps22hb_init handles this on the same bus).
 *
 * @return true on success; false if sensor absent or chip ID mismatch.
 */
bool mag_init();

/**
 * Trigger a single measurement, poll until complete, then burst-read
 * all 9 output bytes and assemble 18-bit values for each axis.
 *
 * @param out  Populated on return. Always check out->valid.
 */
void mag_read(mag_data *out);

/**
 * Non-blocking read for the control loop — call once per tick. Starts a
 * measurement every MMC5603NJ_POLL_PERIOD_MS, and once it has had time
 * to convert, checks STATUS1 and burst-reads it. Never waits on the
 * conversion the way mag_read() does: a tick costs at most one status
 * read, one 9-byte read and one trigger write.
 *
 * @param out  Filled only when this returns true.
 * @return true when `out` holds a new sample.
 */
bool mag_poll(mag_data *out);

/**
 * Magnetometer mounting → body frame. On FC V2 the MMC5603NJ's axes run
 * the same way as the LSM6DSOX's (confirmed on the bench: IMU/mag X is
 * pitch, Y roll along the airframe, Z yaw), so this is the same 90° turn
 * as lsm6dsox_to_body():
 *
 *   body X = -mag Y,   body Y = mag X,   body Z = mag Z
 *
 * Apply after the hard/soft-iron calibration, which is in chip axes.
 */
static inline void mag_to_body(mag_data *m) {
    float x = m->mag_x;
    m->mag_x = -m->mag_y;
    m->mag_y = x;
}