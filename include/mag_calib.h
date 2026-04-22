#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

// Samples collected during the rotation calibration routine.
// Rotate the sensor through a full sphere during this window.
// At ~10 Hz reads, 500 samples = 50 seconds — enough for a thorough rotation.
#define MAG_CALIB_SAMPLES        500

// EEPROM address for mag calibration (after gyro bias at addr 10, size 14)
// Gyro bias ends at addr 10 + 2 + 4 + 4 + 4 = 24. Start mag at 30.
#define MAG_CALIB_EEPROM_ADDR    30
#define MAG_CALIB_MAGIC          0xCA1Bu

// --------------------------------------------------------
// STRUCTS
// --------------------------------------------------------

/**
 * Hard and soft iron correction coefficients.
 *
 * Hard iron (offset):
 *   Constant magnetic offset added to every reading by nearby ferrous
 *   material and DC currents on the PCB. Subtract offset from raw reading.
 *
 * Soft iron (scale):
 *   Anisotropic distortion that stretches the ideal sphere of readings
 *   into an ellipsoid. Multiply by scale after subtracting offset.
 *   Stored as per-axis scale factors (diagonal of the correction matrix).
 *   Full 3x3 matrix correction is overkill for a rocket — diagonal is fine.
 *
 * Corrected value:
 *   mx_cal = (mx_raw - offset_x) * scale_x
 */
typedef struct {
    // Hard iron offsets [Gauss]
    float offset_x, offset_y, offset_z;

    // Soft iron scale factors [dimensionless]
    float scale_x, scale_y, scale_z;
} MagCalib;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Collect MAG_CALIB_SAMPLES readings while the user rotates the sensor
 * through a full sphere, then fit a bounding ellipsoid to extract
 * hard and soft iron coefficients.
 *
 * Prints progress to Serial. Takes ~50 seconds.
 *
 * @param cal  Output calibration coefficients.
 * @return true on success; false if insufficient data spread.
 */
bool mag_calibrate(MagCalib *cal);

/**
 * Apply hard and soft iron correction to a raw magnetometer reading.
 * Call this every time you read the MMC5603NJ instead of using raw values.
 *
 * @param cal          Calibration from mag_calibrate() or mag_load_calib().
 * @param mx/my/mz     Raw Gauss values from mmc5603nj_read().
 * @param cx/cy/cz     Corrected output values [Gauss].
 */
void mag_apply_calib(const MagCalib *cal,
                     float mx,  float my,  float mz,
                     float *cx, float *cy, float *cz);

/**
 * Save calibration to EEPROM.
 */
void mag_save_calib(const MagCalib *cal);

/**
 * Load calibration from EEPROM.
 * @return true if valid data found; false if uncalibrated.
 */
bool mag_load_calib(MagCalib *cal);