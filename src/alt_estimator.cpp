#include <math.h>
#include "alt_estimator.h"

// Complementary filter gains — see ALT_FILTER_TAU_S.
static const float ALT_K1 = 3.0f / ALT_FILTER_TAU_S;
static const float ALT_K2 = 3.0f / (ALT_FILTER_TAU_S * ALT_FILTER_TAU_S);
static const float ALT_K3 = 1.0f / (ALT_FILTER_TAU_S * ALT_FILTER_TAU_S * ALT_FILTER_TAU_S);


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
    est->baro_dt_s          = 0.0f;
    est->baro_reject_count  = 0;

    // Cache the ground-pressure altitude so we don't recompute powf() on
    // every alt_update() tick.
    est->ground_pressure    = ground_hpa;
    est->ground_altitude_m  = pressure_to_altitude(ground_hpa);

    est->cal_accel_sum_g    = 0.0f;
    est->cal_count          = 0;

    est->ground_cap_count   = 0;
    est->ground_cap_used    = 0;
    est->ground_capturing   = false;

    est->initialised        = true;
}


void alt_resume(AltEstimator *est, float ground_hpa, float pressure_hpa) {
    alt_init(est, ground_hpa);
    if (isnan(pressure_hpa)) return;
    est->baro_altitude_m = pressure_to_altitude(pressure_hpa) - est->ground_altitude_m;
    est->altitude_m      = est->baro_altitude_m;
}


void alt_set_ground(AltEstimator *est, float ground_hpa) {
    est->ground_pressure   = ground_hpa;
    est->ground_altitude_m = pressure_to_altitude(ground_hpa);

    // On the pad at the new reference by definition. Leaving the old
    // altitude in the filter would bleed the reference step out over a few
    // ALT_FILTER_TAU_S and look like climb or sink.
    est->altitude_m        = 0.0f;
    est->velocity_ms       = 0.0f;
    est->baro_altitude_m   = 0.0f;
    est->baro_reject_count = 0;
}


bool alt_ground_mean(const float *samples_hpa, uint16_t n,
                     float *mean_hpa, float *median_hpa, uint16_t *n_used) {
    if (n == 0 || n > ALT_GROUND_SAMPLES) return false;

    // Median from a sorted copy (insertion sort — n is only ~100). Unlike
    // the mean, it barely moves for a few wild readings, so it's a safe
    // centre to measure "too far off" from.
    float sorted[ALT_GROUND_SAMPLES] = {};   // zeroed only to quiet -Wmaybe-uninitialized
    for (uint16_t i = 0; i < n; i++) {
        float    v = samples_hpa[i];
        uint16_t j = i;
        while (j > 0 && sorted[j - 1] > v) { sorted[j] = sorted[j - 1]; j--; }
        sorted[j] = v;
    }
    float median = (n & 1) ? sorted[n / 2]
                           : 0.5f * (sorted[n / 2 - 1] + sorted[n / 2]);

    // Double accumulator: 100 × ~1013 hPa in a float would round away
    // centimetres of the mean.
    double   sum  = 0.0;
    uint16_t used = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (fabsf(samples_hpa[i] - median) <= ALT_GROUND_OUTLIER_HPA) {
            sum += samples_hpa[i];
            used++;
        }
    }

    if (median_hpa) *median_hpa = median;
    if (n_used)     *n_used     = used;

    if ((uint32_t)used * 100 < (uint32_t)n * ALT_GROUND_MIN_USED_PCT) return false;
    *mean_hpa = (float)(sum / used);
    return true;
}


void alt_ground_capture_start(AltEstimator *est) {
    est->ground_cap_count = 0;
    est->ground_capturing = true;
}


AltCaptureResult alt_ground_capture_sample(AltEstimator *est, float pressure_hpa) {
    if (!est->ground_capturing) return ALT_CAPTURE_PENDING;
    if (isnan(pressure_hpa)) return ALT_CAPTURE_PENDING;

    est->ground_cap_samples[est->ground_cap_count++] = pressure_hpa;
    if (est->ground_cap_count < ALT_GROUND_SAMPLES) return ALT_CAPTURE_PENDING;

    float mean_hpa;
    if (!alt_ground_mean(est->ground_cap_samples, est->ground_cap_count,
                         &mean_hpa, nullptr, &est->ground_cap_used)) {
        est->ground_cap_count = 0;   // keep capturing with a fresh set
        return ALT_CAPTURE_RESTARTED;
    }

    est->ground_capturing = false;
    alt_set_ground(est, mean_hpa);
    return ALT_CAPTURE_DONE;
}


void alt_ground_capture_cancel(AltEstimator *est) {
    est->ground_capturing = false;
}


void alt_calibrate_reset(AltEstimator *est) {
    est->cal_accel_sum_g = 0.0f;
    est->cal_count       = 0;
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
        // Not enough samples — leave the bias alone and signal the caller.
        return false;
    }

    // Mean accel reading on the pad. An ideal upright stationary sensor
    // reads exactly 1.0 g; deviation is the bias.
    float mean_accel_g = est->cal_accel_sum_g / (float)est->cal_count;

    // Far from 1 g means the samples weren't taken upright and still —
    // using them would bake the vehicle's orientation into the bias.
    if (fabsf(mean_accel_g - 1.0f) > ALT_CAL_MAX_OFFSET_G) return false;

    // bias [m/s²] = (mean - 1g_expected) * gravity
    est->accel_bias_ms2 = (mean_accel_g - 1.0f) * ALT_GRAVITY;

    // Confirmed still: velocity is exactly zero and the baro is the truth.
    // Clears whatever error built up before calibration (e.g. the vehicle
    // lying flat before being stood up on the rail).
    est->velocity_ms = 0.0f;
    est->altitude_m  = est->baro_altitude_m;

    return true;
}


void alt_update(AltEstimator *est,
                float pressure_hpa,
                float accel_z_g,
                float dt) {

    if (!est->initialised) return;
    if (isnan(accel_z_g) || !(dt > 0.0f) || dt > ALT_MAX_DT_S) return;

    // ── PREDICT: integrate bias-corrected vertical acceleration ──
    float accel_ms2 = accel_z_g * ALT_GRAVITY - ALT_GRAVITY - est->accel_bias_ms2;
    est->altitude_m  += est->velocity_ms * dt + 0.5f * accel_ms2 * dt * dt;
    est->velocity_ms += accel_ms2 * dt;
    est->baro_dt_s   += dt;

    if (isnan(pressure_hpa)) return;   // no fresh baro sample this tick

    // ── CORRECT: baro error pulls altitude, velocity and bias ──
    float baro_agl = pressure_to_altitude(pressure_hpa) - est->ground_altitude_m;
    est->baro_altitude_m = baro_agl;
    float err = baro_agl - est->altitude_m;

    if (fabsf(err) > ALT_INNOV_GATE_M) {
        // One wild sample is a glitch — skip it. A run of them is a real
        // offset: jump altitude to the baro rather than correct through it.
        if (++est->baro_reject_count < ALT_INNOV_GATE_COUNT) return;
        est->altitude_m        = baro_agl;
        est->baro_reject_count = 0;
        est->baro_dt_s         = 0.0f;
        return;
    }
    est->baro_reject_count = 0;

    float dt_baro  = fminf(est->baro_dt_s, ALT_BARO_DT_MAX_S);
    est->baro_dt_s = 0.0f;

    est->altitude_m     += ALT_K1 * err * dt_baro;
    est->velocity_ms    += ALT_K2 * err * dt_baro;
    est->accel_bias_ms2 -= ALT_K3 * err * dt_baro;   // baro above estimate → bias set too high
}