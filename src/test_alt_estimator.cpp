// test_alt_estimator.cpp
// ======================
// Compile + behavior checks for the cleaned-up altitude estimator.

#include "alt_estimator.h"

#include <cstdio>
#include <cmath>
#include <cstdlib>


static int failures = 0;

static void check(const char *name, bool cond) {
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", name);
    if (!cond) failures++;
}

static bool approx(float a, float b, float tol) {
    return std::fabs(a - b) < tol;
}


int main(void) {
    const float P0 = 1013.25f;   // hPa at sea level → simulated pad
    const float DT = 0.02f;      // 50 Hz

    // ============================================================
    // 1. Init caches ground altitude
    // ============================================================
    std::printf("Test 1: init caches ground altitude\n");
    {
        AltEstimator est;
        alt_init(&est, P0);
        check("initialised flag set",  est.initialised);
        check("ground_pressure stored", approx(est.ground_pressure, P0, 0.001f));
        check("ground_altitude cached at ~0 m for sea-level pressure",
              approx(est.ground_altitude_m, 0.0f, 0.1f));
        check("accel_bias starts at 0", approx(est.accel_bias_ms2, 0.0f, 1e-6f));
    }

    // ============================================================
    // 2. Calibration with no bias → bias remains ~0
    // ============================================================
    std::printf("Test 2: calibration with perfect 1g samples\n");
    {
        AltEstimator est;
        alt_init(&est, P0);
        for (int i = 0; i < 200; i++) {
            alt_calibrate_sample(&est, 1.0f);
        }
        bool ok = alt_calibrate_finish(&est);
        check("finish returns true with >= ALT_MIN_CAL_SAMPLES", ok);
        check("computed bias is ~0", approx(est.accel_bias_ms2, 0.0f, 1e-4f));
    }

    // ============================================================
    // 3. Calibration with simulated +0.01g sensor bias
    // ============================================================
    std::printf("Test 3: calibration recovers a known bias\n");
    {
        AltEstimator est;
        alt_init(&est, P0);
        for (int i = 0; i < 200; i++) {
            alt_calibrate_sample(&est, 1.01f);
        }
        bool ok = alt_calibrate_finish(&est);
        check("finish returns true", ok);
        // 0.01 g * 9.80665 = 0.0980665 m/s²
        check("computed bias matches expected 0.0981 m/s²",
              approx(est.accel_bias_ms2, 0.0980665f, 1e-4f));
    }

    // ============================================================
    // 4. Insufficient samples → finish returns false, bias stays 0
    // ============================================================
    std::printf("Test 4: insufficient calibration samples\n");
    {
        AltEstimator est;
        alt_init(&est, P0);
        for (int i = 0; i < 10; i++) {  // less than ALT_MIN_CAL_SAMPLES
            alt_calibrate_sample(&est, 1.01f);
        }
        bool ok = alt_calibrate_finish(&est);
        check("finish returns false with too few samples", !ok);
        check("bias remains at 0", approx(est.accel_bias_ms2, 0.0f, 1e-6f));
    }

    // ============================================================
    // 5. After calibration, velocity stays near zero on the pad
    //    (bias should fully compensate the sensor offset)
    // ============================================================
    std::printf("Test 5: post-calibration pad-static velocity drift\n");
    {
        AltEstimator est;
        alt_init(&est, P0);
        // Simulate sensor bias of +0.005 g
        const float pad_reading_g = 1.005f;
        for (int i = 0; i < 200; i++) {
            alt_calibrate_sample(&est, pad_reading_g);
        }
        alt_calibrate_finish(&est);

        // Now run alt_update for 10 seconds of "pad" time
        for (int i = 0; i < 500; i++) {
            alt_update(&est, P0, pad_reading_g, DT);
        }
        std::printf("    velocity after 10 s on pad: %.4f m/s\n",
                    (double)est.velocity_ms);
        std::printf("    altitude after 10 s on pad: %.4f m\n",
                    (double)est.altitude_m);
        check("velocity drift < 0.05 m/s", std::fabs(est.velocity_ms) < 0.05f);
        check("altitude drift < 1 m",      std::fabs(est.altitude_m)  < 1.0f);
    }

    // ============================================================
    // 6. NaN inputs leave state untouched
    // ============================================================
    std::printf("Test 6: NaN guards\n");
    {
        AltEstimator est;
        alt_init(&est, P0);
        for (int i = 0; i < 200; i++) alt_calibrate_sample(&est, 1.0f);
        alt_calibrate_finish(&est);

        // Drive in a normal sample
        alt_update(&est, P0, 1.0f, DT);
        float alt_before = est.altitude_m;
        float vel_before = est.velocity_ms;

        // NaN pressure
        alt_update(&est, NAN, 1.0f, DT);
        check("NaN pressure leaves altitude unchanged",
              approx(est.altitude_m, alt_before, 1e-6f));
        check("NaN pressure leaves velocity unchanged",
              approx(est.velocity_ms, vel_before, 1e-6f));

        // NaN accel
        alt_update(&est, P0, NAN, DT);
        check("NaN accel leaves altitude unchanged",
              approx(est.altitude_m, alt_before, 1e-6f));

        // NaN dt
        alt_update(&est, P0, 1.0f, NAN);
        check("NaN dt leaves altitude unchanged",
              approx(est.altitude_m, alt_before, 1e-6f));

        // Verify state is still usable after NaN events
        for (int i = 0; i < 50; i++) alt_update(&est, P0, 1.0f, DT);
        check("estimator recovers and produces finite altitude",
              std::isfinite(est.altitude_m));
    }

    // ============================================================
    // 7. Bias gain is rate-independent
    //    Run identical scenario at 50 Hz and 100 Hz, check that the
    //    converged bias is the same after equal wall-clock time.
    // ============================================================
    std::printf("Test 7: rate-independent bias convergence\n");
    {
        const float TRUE_BIAS_G = 0.02f;   // sensor reads 1.02 g on the pad
        const float WALL_TIME_S = 30.0f;

        auto run = [&](float fs) {
            AltEstimator est;
            alt_init(&est, P0);
            // Skip pre-arm cal → exercise the in-flight refinement
            float dt = 1.0f / fs;
            int n_ticks = (int)(WALL_TIME_S * fs);
            for (int i = 0; i < n_ticks; i++) {
                alt_update(&est, P0, 1.0f + TRUE_BIAS_G, dt);
            }
            return est.accel_bias_ms2;
        };

        float bias_50Hz  = run(50.0f);
        float bias_100Hz = run(100.0f);
        std::printf("    bias after 30 s @  50 Hz: %.4f m/s²\n", (double)bias_50Hz);
        std::printf("    bias after 30 s @ 100 Hz: %.4f m/s²\n", (double)bias_100Hz);
        check("50 Hz and 100 Hz converge to same bias (within 5%)",
              approx(bias_50Hz, bias_100Hz, 0.05f * std::fabs(bias_50Hz) + 0.001f));
    }

    std::printf("\n");
    if (failures == 0) {
        std::printf("ALL TESTS PASSED.\n");
        return 0;
    } else {
        std::printf("%d test(s) FAILED.\n", failures);
        return 1;
    }
}