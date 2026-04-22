#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// COMPLEMENTARY FILTER TUNING
// --------------------------------------------------------

// Alpha blends barometer (slow/absolute) and accel (fast/drifts).
// Higher alpha = trust accel more = faster response but more drift.
// Lower alpha  = trust baro more  = slower but stable long-term.
// 0.98 is a standard starting point for a ~100 Hz loop.
#define ALT_ALPHA            0.98f

// Acceleration due to gravity [m/s²] — used to convert accel g to m/s²
#define ALT_GRAVITY          9.80665f

// Sea-level standard pressure [hPa] — used for barometric altitude formula
#define ALT_SEA_LEVEL_HPA    1013.25f

// --------------------------------------------------------
// STRUCTS
// --------------------------------------------------------

/**
 * Altitude estimator state.
 * Fuses barometer altitude (absolute, slow) with IMU vertical
 * acceleration (fast, drifts) via a complementary filter.
 */
typedef struct {
    float altitude_m;       // estimated altitude above launch site [m]
    float velocity_ms;      // estimated vertical velocity [m/s]  (+ve = up)
    float accel_bias_ms2;   // vertical accel bias estimate [m/s²]

    float ground_pressure;  // pressure at launch site [hPa] — set on arm
    float baro_altitude_m;  // raw barometric altitude [m]
    float accel_altitude_m; // integrated accel altitude [m]

    bool  initialised;
} AltEstimator;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise the estimator. Call once after sensors are ready.
 * Takes a baseline pressure reading — rocket must be stationary
 * on the launch pad at this point.
 *
 * @param est           Estimator state.
 * @param ground_hpa    Current pressure at ground level [hPa].
 */
void alt_init(AltEstimator *est, float ground_hpa);

/**
 * Update the estimator with new sensor readings.
 * Call every loop iteration at a consistent rate.
 *
 * @param est           Estimator state updated in place.
 * @param pressure_hpa  Current barometric pressure [hPa].
 * @param accel_z_g     Vertical acceleration in body frame [g].
 *                      Must be the axis pointing up when rocket is upright.
 * @param dt            Time since last call [seconds].
 */
void alt_update(AltEstimator *est,
                float pressure_hpa,
                float accel_z_g,
                float dt);

/**
 * Convert raw pressure to altitude above sea level using
 * the international barometric formula.
 *
 * @param pressure_hpa  Pressure [hPa].
 * @return Altitude [m].
 */
float pressure_to_altitude(float pressure_hpa);