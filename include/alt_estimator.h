#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// COMPLEMENTARY FILTER TUNING
// --------------------------------------------------------

// Third-order complementary filter: the accelerometer predicts altitude and
// velocity every tick; each baro sample's error (baro − estimate) then
// corrects altitude, velocity AND the accel bias, with gains
//   k1 = 3/τ,  k2 = 3/τ²,  k3 = 1/τ³   (triple pole at −1/τ, no overshoot).
// Correcting velocity and bias from the baro is what keeps velocity from
// drifting without bound on an uncalibrated or mis-calibrated accel.
// Smaller τ trusts the baro more (faster bias learning, noisier velocity).
// 1 s: bias learned within a few seconds on the pad; with 0.07 m baro
// noise the velocity error stays under ~0.1 m/s through the F-15 flight
// in simulation.
#define ALT_FILTER_TAU_S        1.0f

// Baro innovation gate. A sample further than this from the estimate is
// treated as a glitch and skipped — a single wild sample (the bench once
// read 28 hPa ≈ 238 m off) must not kick velocity. Normal innovations are
// well under 1 m; standing the rocket up after it lay flat peaks ~2.6 m.
#define ALT_INNOV_GATE_M        10.0f

// ...unless this many consecutive samples (~0.1 s at 75 Hz) all disagree:
// then the disagreement is real (e.g. a wrong ground reference) and the
// altitude re-syncs straight to the baro. Without this, a large real error
// would be locked out forever. (Clamping the innovation instead was tried
// in simulation: it oscillates with growing amplitude on large errors.)
#define ALT_INNOV_GATE_COUNT    8

// Longest baro gap the correction step will integrate over [s] — keeps the
// first sample after an outage from over-correcting (k1·dt ≪ 1).
#define ALT_BARO_DT_MAX_S       0.1f

// alt_update() skips ticks with dt above this [s]. The first loop tick
// after setup() measures dt from boot (seconds) — integrating that in one
// step would inject a velocity error.
#define ALT_MAX_DT_S            0.1f

// Acceleration due to gravity
#define ALT_GRAVITY             9.80665f

// Sea-level standard pressure [hPa] — used for barometric altitude formula
#define ALT_SEA_LEVEL_HPA       1013.25f

// Minimum samples required for pre-arm calibration to be considered valid
#define ALT_MIN_CAL_SAMPLES     50

// Calibration is rejected if the mean reading is further than this from
// 1 g — the vehicle wasn't upright and still, so the "bias" would really be
// its orientation. The LSM6DSOX's zero-g offset is ±20 mg typ; a 10° rail
// tilt reads 0.985 g.
#define ALT_CAL_MAX_OFFSET_G    0.1f

// Fresh baro samples averaged into a ground reference — the boot reference
// and the launch baseline captured on the SW401 arming edge. ~1.33 s at the
// LPS22HB's 75 Hz ODR.
#define ALT_GROUND_SAMPLES      100

// Ground-reference outlier rejection (alt_ground_mean()): a sample further
// than this from the set's median is dropped from the average. ~4 m — a
// still board spreads ~0.02 hPa; the bad reading seen on the bench was
// 28 hPa off.
#define ALT_GROUND_OUTLIER_HPA  0.5f

// If fewer than this percentage of samples survive, the whole set is
// rejected: that many outliers means the sensor is glitching or the vehicle
// is moving, and the median itself can't be trusted.
#define ALT_GROUND_MIN_USED_PCT 80

// --------------------------------------------------------
// STRUCTS
// --------------------------------------------------------

/**
 * Altitude estimator state.
 * Fuses barometer altitude (absolute, slow) with IMU vertical
 * acceleration (fast, drifts) via a third-order complementary filter —
 * see ALT_FILTER_TAU_S.
 *
 * Call sequence:
 *   alt_init(&est, current_pressure_hpa);     // once after sensors are up
 *   alt_update(&est, p, a, dt);               // every loop tick, from boot
 *   // -- on the pad: --
 *   alt_calibrate_reset(&est);                // when the vehicle comes to rest upright
 *   alt_calibrate_sample(&est, accel_z_g);    // every tick while it stays that way
 *   alt_calibrate_finish(&est);               // once enough samples (pad-rest latch)
 */
typedef struct {
    // ---- filter state ----
    float altitude_m;       // estimated altitude above launch site [m]
    float velocity_ms;      // estimated vertical velocity [m/s] (+ = up)
    float accel_bias_ms2;   // vertical accel bias estimate [m/s²] — calibrated on the pad, then tracked from the baro

    // ---- baro correction bookkeeping ----
    float    baro_dt_s;         // time since the last applied baro correction [s]
    uint16_t baro_reject_count; // consecutive samples outside ALT_INNOV_GATE_M

    // ---- last raw reading (for debug / telemetry) ----
    float baro_altitude_m;  // raw barometric altitude AGL [m]

    // ---- launch-site reference ----
    float ground_pressure;    // pressure at launch site [hPa]
    float ground_altitude_m;  // pressure_to_altitude(ground_pressure) — cached

    // ---- pre-arm calibration accumulator ----
    float    cal_accel_sum_g; // running sum of accel_z_g samples
    uint32_t cal_count;       // number of calibration samples accumulated

    // ---- ground-reference capture (alt_ground_capture_*) ----
    float    ground_cap_samples[ALT_GROUND_SAMPLES];
    uint16_t ground_cap_count;
    uint16_t ground_cap_used;     // samples kept by the last completed set (outliers dropped)
    bool     ground_capturing;

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
 * Re-initialise mid-flight after a watchdog reset: same ground reference
 * as before the reset, but altitude starts at the current baro altitude
 * instead of zero. Velocity restarts at zero and the accel bias is
 * re-learned from the baro.
 *
 * @param est           Estimator state.
 * @param ground_hpa    Ground pressure saved before the reset [hPa].
 * @param pressure_hpa  Current pressure [hPa]; NaN to start at zero.
 */
void alt_resume(AltEstimator *est, float ground_hpa, float pressure_hpa);

/**
 * Re-reference to a new ground pressure. Altitude and velocity are zeroed —
 * the vehicle must be stationary on the pad at this pressure. The accel
 * bias and its calibration are kept.
 *
 * @param est         Estimator state.
 * @param ground_hpa  Pressure at the launch site [hPa].
 */
void alt_set_ground(AltEstimator *est, float ground_hpa);

/**
 * Outlier-rejecting mean of a set of pressure samples: samples further
 * than ALT_GROUND_OUTLIER_HPA from the median are dropped, the rest are
 * averaged. Used for every ground reference (boot, arming, bench Test H).
 *
 * @param samples_hpa  Samples [hPa]; not modified.
 * @param n            Number of samples, 1..ALT_GROUND_SAMPLES.
 * @param mean_hpa     Mean of the kept samples. Untouched on failure.
 * @param median_hpa   Optional (nullptr to skip): the set's median.
 * @param n_used       Optional (nullptr to skip): samples kept.
 * @return false if fewer than ALT_GROUND_MIN_USED_PCT % were kept, or n is
 *         out of range.
 */
bool alt_ground_mean(const float *samples_hpa, uint16_t n,
                     float *mean_hpa, float *median_hpa, uint16_t *n_used);

/** alt_ground_capture_sample() result. */
typedef enum {
    ALT_CAPTURE_PENDING = 0,   // still collecting, or no capture running
    ALT_CAPTURE_DONE,          // new ground reference applied
    ALT_CAPTURE_RESTARTED,     // too many outliers — set discarded, collecting again
} AltCaptureResult;

/**
 * Start a non-blocking ground-reference capture: feed it the loop's baro
 * reading every tick with alt_ground_capture_sample(). Restarts any
 * capture already in progress.
 */
void alt_ground_capture_start(AltEstimator *est);

/**
 * Feed one pressure sample to an in-progress capture. NaN (no fresh baro
 * data this tick) is skipped. On the ALT_GROUND_SAMPLES-th valid sample the
 * set goes through alt_ground_mean(): if it passes, the mean is applied via
 * alt_set_ground() (ALT_CAPTURE_DONE, once per capture); if too many
 * samples were outliers, the set is thrown away and collection starts over
 * (ALT_CAPTURE_RESTARTED). ground_cap_used holds the kept count either way.
 *
 * @param est           Estimator state.
 * @param pressure_hpa  This tick's pressure [hPa], or NaN.
 */
AltCaptureResult alt_ground_capture_sample(AltEstimator *est, float pressure_hpa);

/** Abandon an in-progress capture; the current ground reference stays. */
void alt_ground_capture_cancel(AltEstimator *est);

/**
 * Discard accumulated calibration samples — call when a new still, upright
 * window begins, so only samples from that window are averaged (not time
 * spent being carried or lying flat).
 */
void alt_calibrate_reset(AltEstimator *est);

/**
 * Accumulate one accelerometer sample for pre-arm bias calibration.
 * Call every tick while the rocket is stationary and upright on the pad,
 * after alt_calibrate_reset() at the start of that window. The mean accel
 * reading is used to extract the bias (an ideal sensor reads exactly 1 g
 * upward at rest; any deviation is the bias).
 *
 * NaN samples are ignored. No-op if the estimator is not initialised.
 *
 * @param est        Estimator state.
 * @param accel_z_g  Vertical acceleration in body frame [g].
 */
void alt_calibrate_sample(AltEstimator *est, float accel_z_g);

/**
 * Finalise pre-arm calibration from the samples since the last
 * alt_calibrate_reset(). On success the bias is set, velocity is zeroed and
 * altitude re-synced to the latest baro reading — the vehicle is confirmed
 * still, so both are known exactly.
 *
 * Returns false and changes nothing if fewer than ALT_MIN_CAL_SAMPLES were
 * accumulated, or the mean is more than ALT_CAL_MAX_OFFSET_G from 1 g (not
 * upright/still). The filter still learns the bias from the baro either way.
 *
 * @param est  Estimator state.
 * @return     true if the calibration was applied.
 */
bool alt_calibrate_finish(AltEstimator *est);

/**
 * Update the estimator with new sensor readings. Call every loop tick.
 *
 * NaN pressure (no fresh baro sample this tick) runs the accel prediction
 * only. NaN accel, or dt that is not in (0, ALT_MAX_DT_S], skips the tick
 * entirely. Baro samples more than ALT_INNOV_GATE_M from the estimate are
 * skipped as glitches unless ALT_INNOV_GATE_COUNT arrive in a row, which
 * re-syncs altitude to the baro.
 *
 * @param est           Estimator state updated in place.
 * @param pressure_hpa  Current barometric pressure [hPa], or NaN.
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