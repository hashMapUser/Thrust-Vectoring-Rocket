#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// I2C ADDRESS
// --------------------------------------------------------
#define LPS22HB_ADDRESS         0x5C   // SA0 low; use 0x5D if SA0 pulled high

// --------------------------------------------------------
// REGISTER MAP
// --------------------------------------------------------
#define LPS22HB_REG_WHO_AM_I    0x0F   // expected value: 0xB1
#define LPS22HB_REG_CTRL1       0x10   // ODR and BDU config
#define LPS22HB_REG_CTRL2       0x11   // auto-increment, software reset
#define LPS22HB_REG_STATUS      0x27   // data-ready flags

#define LPS22HB_REG_PRESS_XL    0x28   // pressure XLSB  (24-bit, little-endian)
#define LPS22HB_REG_PRESS_L     0x29
#define LPS22HB_REG_PRESS_H     0x2A   // pressure MSB
#define LPS22HB_REG_TEMP_L      0x2B   // temperature LSB (16-bit, little-endian)
#define LPS22HB_REG_TEMP_H      0x2C

// CTRL_REG1 — bits [6:4] = ODR, bit 1 = BDU
// ODR encoding: 000=off, 001=1Hz, 010=10Hz, 011=25Hz, 100=50Hz, 101=75Hz
#define LPS22HB_CTRL1_75HZ_BDU  0x52   // 75 Hz continuous, block data update on
#define LPS22HB_CTRL1_25HZ_BDU  0x32   // kept for reference

// CTRL_REG2 — bit 4 = IF_ADD_INC (auto-increment register address on burst reads)
#define LPS22HB_CTRL2_INC       0x10

// Status register bits
#define LPS22HB_STATUS_P_DA     (1 << 0)   // pressure data available
#define LPS22HB_STATUS_T_DA     (1 << 1)   // temperature data available

// Expected WHO_AM_I value
#define LPS22HB_CHIP_ID         0xB1

// I2C fast-mode clock
#define LPS22HB_I2C_CLOCK       400000UL

// lps22hb_read_samples(): give up if no fresh sample arrives within this
// long. ~4 sample periods at 75 Hz — long enough to ride out one missed
// conversion, short enough that a dead sensor fails fast.
#define LPS22HB_SAMPLE_TIMEOUT_MS  50

// PIN ASSIGNMENTS — Wire (I2C0): SDA=PIN_BARO_SDA (18), SCL=PIN_BARO_SCL (19) from board_pins.h
#include "board_pins.h"

// --------------------------------------------------------
// STRUCTS
// --------------------------------------------------------

/**
 * Single measurement result.
 * pressure_pa is in Pascals; divide by 100 for hPa.
 */
typedef struct {
    float temperature_c;
    float pressure_pa;
    bool  valid;
} LPS22HB_Data;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Verify WHO_AM_I, enable auto-increment, and configure 25 Hz ODR with BDU.
 * Call once in setup().
 *
 * @return true on success; false if the sensor is absent or I2C fails.
 */
bool lps22hb_init(void);

/**
 * Burst-read 5 bytes (pressure + temperature) and convert to engineering units.
 * The LPS22HB applies compensation internally — no calibration coefficients needed.
 *
 * @param out  Output measurement.  out->valid is false on any I2C error.
 */
void lps22hb_read(LPS22HB_Data *out);

/**
 * Blocking read of `samples` fresh pressure readings. Each one is a new
 * conversion (gated on P_DA like lps22hb_read()), so 100 samples take
 * ~1.33 s at 75 Hz. Average them with alt_ground_mean(), which drops
 * outliers. Blocks — do not call from the watchdog-fed flight loop; use
 * alt_ground_capture_*() there instead.
 *
 * @param samples      Number of fresh samples to read (> 0).
 * @param samples_hpa  Receives each reading [hPa]; must hold `samples` floats.
 * @return false if the sensor stops producing data (no fresh sample within
 *         LPS22HB_SAMPLE_TIMEOUT_MS).
 */
bool lps22hb_read_samples(uint16_t samples, float *samples_hpa);
