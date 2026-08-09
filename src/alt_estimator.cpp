#include <math.h>
#include "alt_estimator.h"


float pressure_to_altitude(float pressure_hpa) {
    // International barometric formula:
    // h = 44330 * (1 - (P / P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure_hpa / ALT_SEA_LEVEL_HPA, 0.1902949f));
}


void alt_init(AltEstimator *est, float ground_hpa) {
    est->altitude_m         = 0.0f;
    est->velocity_ms        = 0.0f;
    est->accel_bias_ms2     = 0.0f;
    est->baro_altitude_m    = 0.0f;

    // Cache the ground-pressure altitude so we don't recompute powf() on
    // every alt_update() tick.
    est->ground_pressure    = ground_hpa;
    est->ground_altitude_m  = pressure_to_altitude(ground_hpa);

    est->cal_accel_sum_g    = 0.0f;
    est->cal_count          = 0;

    est->initialised        = true;
}


void alt_calibrate_sample(AltEstimator *est, float accel_z_g) {
    if (!est->initialised) return;
    if (isnan(accel_z_g)) return;

    est->cal_accel_sum_g += accel_z_g;
    est->cal_count++;
}


bool alt_calibrate_finish(AltEstimator *est) {
    if (!est->initialised) return false;

    if (est->cal_count < ALT_MIN_CAL_SAMPLES) {
        // Not enough samples — leave bias at zero and signal the caller.
        return false;
    }

    // Mean accel reading on the pad. An ideal upright stationary sensor
    // reads exactly 1.0 g; deviation is the bias.
    float mean_accel_g = est->cal_accel_sum_g / (float)est->cal_count;

    // bias [m/s²] = (mean - 1g_expected) * gravity
    est->accel_bias_ms2 = (mean_accel_g - 1.0f) * ALT_GRAVITY;

    return true;
}


void alt_update(AltEstimator *est,
                float pressure_hpa,
                float accel_z_g,
                float dt) {

    if (!est->initialised) return;
    if (isnan(accel_z_g) || isnan(dt)) return;

    // ── INERTIAL ACCELERATION ──
    float accel_ms2       = accel_z_g * ALT_GRAVITY;
    float accel_corrected = accel_ms2 - ALT_GRAVITY - est->accel_bias_ms2;

    // ── INTEGRATE ACCELERATION → VELOCITY ──
    est->velocity_ms += accel_corrected * dt;

    if (!isnan(pressure_hpa)) {
        // ── BAROMETRIC ALTITUDE (only when sensor has new data) ──
        float baro_agl = pressure_to_altitude(pressure_hpa) - est->ground_altitude_m;
        est->baro_altitude_m = baro_agl;

        // ── COMPLEMENTARY FILTER ──
        // Blend inertial prediction with absolute baro altitude.
        // Only applied when baro data is fresh (P_DA was set).
        est->altitude_m = ALT_ALPHA * (est->altitude_m + est->velocity_ms * dt)
                        + (1.0f - ALT_ALPHA) * baro_agl;
    } else {
        // ── INERTIAL-ONLY PROPAGATION (baro not ready this tick) ──
        est->altitude_m += est->velocity_ms * dt;
    }

    // ── IN-FLIGHT BIAS REFINEMENT ──
    if (fabsf(est->velocity_ms) < ALT_BIAS_VEL_GATE) {
        est->accel_bias_ms2 += accel_corrected * ALT_BIAS_GAIN_HZ * dt;
    }
}