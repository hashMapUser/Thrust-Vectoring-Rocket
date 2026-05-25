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
#define ALT_ALPHA               0.98f

// Acceleration due to gravity [m/s²] — used to convert accel g to m/s²
#define ALT_GRAVITY             9.80665f

// Sea-level standard pressure [hPa] — used for barometric altitude formula
#define ALT_SEA_LEVEL_HPA       1013.25f

// In-flight accel bias estimator gain [1/s]. The bias loop runs only when
// vertical velocity is below ALT_BIAS_VEL_GATE. With 0.05, the bias time
// constant is ~20 s. Gain is multiplied by dt in alt_update() so the
// convergence rate is independent of loop frequency.
#define ALT_BIAS_GAIN_HZ        0.05f

// Velocity gate for in-flight bias refinement [m/s]
#define ALT_BIAS_VEL_GATE       0.5f

// Minimum samples required for pre-arm calibration to be considered valid
#define ALT_MIN_CAL_SAMPLES     50

// --------------------------------------------------------
// STRUCTS
// --------------------------------------------------------

/**
 * Altitude estimator state.
 * Fuses barometer altitude (absolute, slow) with IMU vertical
 * acceleration (fast, drifts) via a complementary filter.
 *
 * Call sequence:
 *   alt_init(&est, current_pressure_hpa);     // once after sensors are up
 *   // -- IDLE state, rocket on pad: --
 *   alt_calibrate_sample(&est, accel_z_g);    // every loop tick
 *   ...
 *   alt_calibrate_finish(&est);               // at IDLE → ARM transition
 *   // -- flight: --
 *   alt_update(&est, p, a, dt);               // every loop tick
 */
typedef struct {
    // ---- filter state ----
    float altitude_m;       // estimated altitude above launch site [m]
    float velocity_ms;      // estimated vertical velocity [m/s] (+ = up)
    float accel_bias_ms2;   // vertical accel bias estimate [m/s²]

    // ---- last raw reading (for debug / telemetry) ----
    float baro_altitude_m;  // raw barometric altitude AGL [m]

    // ---- launch-site reference ----
    float ground_pressure;    // pressure at launch site [hPa]
    float ground_altitude_m;  // pressure_to_altitude(ground_pressure) — cached

    // ---- pre-arm calibration accumulator ----
    float    cal_accel_sum_g; // running sum of accel_z_g samples
    uint32_t cal_count;       // number of calibration samples accumulated

    // ---- flags ----
    bool initialised;
} AltEstimator;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise the estimator. Call once after sensors are ready.
 * Takes a baseline pressure reading — rocket must be stationary on the
 * launch pad at this point.
 *
 * @param est           Estimator state.
 * @param ground_hpa    Current pressure at ground level [hPa].
 */
void alt_init(AltEstimator *est, float ground_hpa);

/**
 * Accumulate one accelerometer sample for pre-arm bias calibration.
 * Call repeatedly during the IDLE state while the rocket is stationary
 * and upright on the pad. The mean accel reading during this window is
 * used to extract the bias (an ideal sensor reads exactly 1g upward at
 * rest; any deviation is the bias).
 *
 * NaN samples are ignored. No-op if the estimator is not initialised.
 *
 * @param est        Estimator state.
 * @param accel_z_g  Vertical acceleration in body frame [g].
 */
void alt_calibrate_sample(AltEstimator *est, float accel_z_g);

/**
 * Finalise pre-arm calibration. Computes the accel bias from the mean of
 * samples accumulated by alt_calibrate_sample(). Call once at the
 * IDLE → ARM (or equivalent) state transition.
 *
 * If fewer than ALT_MIN_CAL_SAMPLES were accumulated, the bias is left at
 * zero and the function returns false — the caller can either fall back
 * to the in-flight bias refinement loop or treat this as a hard fault.
 *
 * @param est  Estimator state.
 * @return     true if bias was applied from sufficient samples;
 *             false if not enough samples were collected.
 */
bool alt_calibrate_finish(AltEstimator *est);

/**
 * Update the estimator with new sensor readings.
 * Call every loop iteration at a consistent rate after calibration.
 *
 * Any NaN input is treated as a transient sensor fault: the function
 * returns without modifying state. This means an upstream BMP390 read
 * failure (which returns NaN per the driver pattern) cannot poison the
 * integrator.
 *
 * @param est           Estimator state updated in place.
 * @param pressure_hpa  Current barometric pressure [hPa].
 * @param accel_z_g     Vertical acceleration in body frame [g].
 *                      Must be the axis pointing up when rocket is upright.
 *                      (Body-frame approximation; small-angle error only.
 *                      Replace with quaternion-rotated world-frame accel
 *                      in Rev 2.)
 * @param dt            Time since last call [seconds].
 */
void alt_update(AltEstimator *est,
                float pressure_hpa,
                float accel_z_g,
                float dt);

/**
 * Convert raw pressure to altitude above sea level using the
 * international barometric formula.
 *
 * @param pressure_hpa  Pressure [hPa].
 * @return Altitude [m].
 */
float pressure_to_altitude(float pressure_hpa);