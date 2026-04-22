#include <math.h>
#include "alt_estimator.h"

float pressure_to_altitude(float pressure_hpa) {
    // International barometric formula:
    // h = 44330 * (1 - (P / P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure_hpa / ALT_SEA_LEVEL_HPA, 0.1902949f));
}

void alt_init(AltEstimator *est, float ground_hpa) {
    est->ground_pressure  = ground_hpa;
    est->altitude_m       = 0.0f;
    est->velocity_ms      = 0.0f;
    est->accel_bias_ms2   = 0.0f;
    est->baro_altitude_m  = 0.0f;
    est->accel_altitude_m = 0.0f;
    est->initialised      = true;
}

void alt_update(AltEstimator *est,
                float pressure_hpa,
                float accel_z_g,
                float dt) {

    if (!est->initialised) return;

    // ── BAROMETRIC ALTITUDE ──
    // Altitude above sea level minus ground altitude = AGL altitude
    float baro_asl = pressure_to_altitude(pressure_hpa);
    float baro_agl = baro_asl - pressure_to_altitude(est->ground_pressure);
    est->baro_altitude_m = baro_agl;

    // ── INERTIAL ALTITUDE ──
    // Convert vertical accel from g to m/s², subtract gravity and bias.
    // During free flight the vertical axis reads 0g (weightlessness) —
    // gravity is already accounted for by the reference frame.
    // On the ground it reads 1g upward — bias corrects for this.
    float accel_ms2 = accel_z_g * ALT_GRAVITY;

    // Remove gravity (1g = 9.80665 m/s²) and estimated bias
    float accel_corrected = accel_ms2 - ALT_GRAVITY - est->accel_bias_ms2;

    // Integrate acceleration → velocity → position
    est->velocity_ms      += accel_corrected * dt;
    est->accel_altitude_m += est->velocity_ms * dt;

    // ── COMPLEMENTARY FILTER ──
    // Blend inertial (fast) with barometric (slow/absolute).
    // The baro term continuously corrects the inertial drift.
    est->altitude_m = ALT_ALPHA * (est->altitude_m + est->velocity_ms * dt)
                    + (1.0f - ALT_ALPHA) * baro_agl;

    // ── BIAS ESTIMATION ──
    // When velocity is near zero (on ground or at apogee), slowly
    // nudge the accel bias toward whatever offset would make velocity = 0.
    // This prevents the integrator from drifting on the pad.
    if (fabsf(est->velocity_ms) < 0.5f) {
        est->accel_bias_ms2 += accel_corrected * 0.001f;
    }
}