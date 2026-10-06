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

        // NaN pressure = accel-only prediction; at rest that changes nothing
        alt_update(&est, NAN, 1.0f, DT);
        check("NaN pressure (accel-only, at rest) leaves altitude unchanged",
              approx(est.altitude_m, alt_before, 1e-6f));
        check("NaN pressure (accel-only, at rest) leaves velocity unchanged",
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
    // 7. Uncalibrated bias is learned from the baro, at any loop rate
    //    No pad calibration: the filter must find the true bias and keep
    //    velocity at zero. (The old filter's gated refinement stalled at
    //    ~13% of the bias once velocity drifted past 0.5 m/s — and this
    //    test still passed, because it only compared the two rates.)
    // ============================================================
    std::printf("Test 7: bias learned from the baro, rate-independent\n");
    {
        const float TRUE_BIAS_G   = 0.02f;   // sensor reads 1.02 g on the pad
        const float TRUE_BIAS_MS2 = TRUE_BIAS_G * ALT_GRAVITY;
        const float WALL_TIME_S   = 30.0f;

        auto run = [&](float fs, AltEstimator *est) {
            alt_init(est, P0);
            float dt = 1.0f / fs;
            int n_ticks = (int)(WALL_TIME_S * fs);
            for (int i = 0; i < n_ticks; i++) {
                alt_update(est, P0, 1.0f + TRUE_BIAS_G, dt);
            }
        };

        AltEstimator e50, e100;
        run(50.0f, &e50);
        run(100.0f, &e100);
        std::printf("    @  50 Hz: bias %.4f m/s², velocity %+.4f m/s\n",
                    (double)e50.accel_bias_ms2, (double)e50.velocity_ms);
        std::printf("    @ 100 Hz: bias %.4f m/s², velocity %+.4f m/s\n",
                    (double)e100.accel_bias_ms2, (double)e100.velocity_ms);
        check("50 Hz learns the true bias (within 2%)",
              approx(e50.accel_bias_ms2, TRUE_BIAS_MS2, 0.02f * TRUE_BIAS_MS2));
        check("100 Hz learns the true bias (within 2%)",
              approx(e100.accel_bias_ms2, TRUE_BIAS_MS2, 0.02f * TRUE_BIAS_MS2));
        check("velocity held at ~0 (no drift)",
              std::fabs(e50.velocity_ms) < 0.01f && std::fabs(e100.velocity_ms) < 0.01f);
        check("altitude held at ~0",
              std::fabs(e50.altitude_m) < 0.05f && std::fabs(e100.altitude_m) < 0.05f);
    }

    // ============================================================
    // 8. Ground-reference capture: averages ALT_GROUND_SAMPLES valid
    //    samples, skips NaN, re-references and zeroes altitude/velocity
    // ============================================================
    std::printf("Test 8: ground-reference capture\n");
    {
        const float PAD_HPA = 985.0f;   // pad ~240 m above the boot reference
        AltEstimator est;
        alt_init(&est, P0);
        // Let the filter settle against the wrong (boot) reference
        for (int i = 0; i < 500; i++) alt_update(&est, PAD_HPA, 1.0f, DT);
        check("altitude off by ~240 m before capture", est.altitude_m > 200.0f);

        alt_ground_capture_start(&est);
        int completions = 0;
        for (int i = 0; i < ALT_GROUND_SAMPLES - 1; i++) {
            // Alternate ±0.1 hPa around the pad pressure, with NaN gaps
            if (alt_ground_capture_sample(&est, NAN) != ALT_CAPTURE_PENDING) completions++;
            if (alt_ground_capture_sample(&est, PAD_HPA + ((i & 1) ? 0.1f : -0.1f)) != ALT_CAPTURE_PENDING) completions++;
        }
        check("not complete before ALT_GROUND_SAMPLES valid samples",
              completions == 0 && est.ground_capturing);
        AltCaptureResult r = alt_ground_capture_sample(&est, PAD_HPA + 0.1f);
        check("completes on the last sample", r == ALT_CAPTURE_DONE && !est.ground_capturing);
        check("ground pressure is the mean",
              approx(est.ground_pressure, PAD_HPA, 0.01f));
        check("all samples kept", est.ground_cap_used == ALT_GROUND_SAMPLES);
        check("altitude zeroed", approx(est.altitude_m, 0.0f, 1e-6f));
        check("velocity zeroed", approx(est.velocity_ms, 0.0f, 1e-6f));
        check("further samples ignored once complete",
              alt_ground_capture_sample(&est, PAD_HPA) == ALT_CAPTURE_PENDING);

        for (int i = 0; i < 500; i++) alt_update(&est, PAD_HPA, 1.0f, DT);
        check("altitude stays near 0 on the pad after capture",
              std::fabs(est.altitude_m) < 0.5f);

        // Cancel keeps the current reference
        alt_ground_capture_start(&est);
        for (int i = 0; i < 50; i++) alt_ground_capture_sample(&est, 900.0f);
        alt_ground_capture_cancel(&est);
        check("cancel keeps previous reference",
              approx(est.ground_pressure, PAD_HPA, 0.01f) && !est.ground_capturing);
    }

    // ============================================================
    // 9. Outlier rejection: a wild reading is dropped from the mean;
    //    too many outliers rejects the whole set
    // ============================================================
    std::printf("Test 9: ground-reference outlier rejection\n");
    {
        const float TRUE_HPA = 1013.5f;
        float s[ALT_GROUND_SAMPLES];
        float mean = -1.0f, median = -1.0f;
        uint16_t used = 0;

        // Two glitches like the one seen on the bench (28 hPa low / high)
        for (int i = 0; i < ALT_GROUND_SAMPLES; i++) s[i] = TRUE_HPA + ((i & 1) ? 0.02f : -0.02f);
        s[0]  = 985.5f;
        s[57] = 1041.5f;
        bool ok = alt_ground_mean(s, ALT_GROUND_SAMPLES, &mean, &median, &used);
        check("set with 2 glitches accepted", ok);
        check("glitches dropped", used == ALT_GROUND_SAMPLES - 2);
        check("mean unaffected by glitches", approx(mean, TRUE_HPA, 0.005f));
        check("median is the true pressure", approx(median, TRUE_HPA, 0.03f));

        // 30 % outliers — the median still lands on the good cluster, but
        // a set this bad isn't trusted
        for (int i = 0; i < ALT_GROUND_SAMPLES; i++) s[i] = (i < 30) ? 985.5f : TRUE_HPA;
        mean = -1.0f;
        ok = alt_ground_mean(s, ALT_GROUND_SAMPLES, &mean, nullptr, &used);
        check("set with 30% outliers rejected", !ok && used == 70);
        check("mean untouched on rejection", mean == -1.0f);

        check("n = 0 rejected", !alt_ground_mean(s, 0, &mean, nullptr, nullptr));

        // Capture path: a bad set restarts collection instead of applying
        AltEstimator est;
        alt_init(&est, P0);
        alt_ground_capture_start(&est);
        AltCaptureResult r = ALT_CAPTURE_PENDING;
        for (int i = 0; i < ALT_GROUND_SAMPLES; i++)
            r = alt_ground_capture_sample(&est, (i < 30) ? 985.5f : TRUE_HPA);
        check("bad set restarts capture",
              r == ALT_CAPTURE_RESTARTED && est.ground_capturing &&
              est.ground_cap_count == 0 && approx(est.ground_pressure, P0, 0.001f));

        for (int i = 0; i < ALT_GROUND_SAMPLES; i++)
            r = alt_ground_capture_sample(&est, (i == 10) ? 985.5f : TRUE_HPA);
        check("good set after restart applies, glitch dropped",
              r == ALT_CAPTURE_DONE && !est.ground_capturing &&
              est.ground_cap_used == ALT_GROUND_SAMPLES - 1 &&
              approx(est.ground_pressure, TRUE_HPA, 0.001f));
    }

    // ============================================================
    // 10. Calibration window: rejects non-upright samples, reset discards
    //     old ones, success zeroes velocity and re-syncs altitude to baro
    // ============================================================
    std::printf("Test 10: calibration window and sanity check\n");
    {
        AltEstimator est;
        alt_init(&est, P0);

        // Lying flat: mean ~0 g — must be rejected, bias untouched
        for (int i = 0; i < 200; i++) alt_calibrate_sample(&est, 0.0f);
        check("flat-lying samples rejected", !alt_calibrate_finish(&est));
        check("bias untouched on rejection", approx(est.accel_bias_ms2, 0.0f, 1e-6f));

        // New still window: reset, then upright samples only
        alt_calibrate_reset(&est);
        for (int i = 0; i < 200; i++) alt_calibrate_sample(&est, 1.01f);
        est.velocity_ms     = 3.0f;    // error built up before the window
        est.altitude_m      = 2.5f;
        est.baro_altitude_m = 0.1f;    // latest baro AGL
        check("upright window accepted after reset", alt_calibrate_finish(&est));
        check("bias from the window only (0.01 g)",
              approx(est.accel_bias_ms2, 0.01f * ALT_GRAVITY, 1e-4f));
        check("velocity zeroed", approx(est.velocity_ms, 0.0f, 1e-6f));
        check("altitude re-synced to baro", approx(est.altitude_m, 0.1f, 1e-6f));
    }

    // ============================================================
    // 11. Pad scenario: power on lying flat for 10 s, then stood up on
    //     the rail. Old filter: velocity +374 m/s, altitude +145 m by
    //     launch. Now bounded before calibration, exact after.
    // ============================================================
    std::printf("Test 11: lying flat after power-on, then upright\n");
    {
        const float LOOP_DT = 1.0f / 125.0f;
        AltEstimator est;
        alt_init(&est, P0);
        float max_v = 0.0f;
        for (int i = 0; i < 10 * 125; i++) alt_update(&est, P0, 0.0f, LOOP_DT);
        check("velocity bounded while lying flat", std::fabs(est.velocity_ms) < 1.0f);

        alt_calibrate_reset(&est);     // pad-rest timer starts as it's stood up
        for (int i = 0; i < 2 * 125; i++) {
            alt_calibrate_sample(&est, 1.0f);
            alt_update(&est, P0, 1.0f, LOOP_DT);
            if (std::fabs(est.velocity_ms) > max_v) max_v = std::fabs(est.velocity_ms);
        }
        std::printf("    peak |velocity| while settling: %.2f m/s\n", (double)max_v);
        check("pad-rest calibration accepted", alt_calibrate_finish(&est));
        for (int i = 0; i < 60 * 125; i++) alt_update(&est, P0, 1.0f, LOOP_DT);
        std::printf("    after 60 s upright: velocity %+.4f m/s, altitude %+.4f m\n",
                    (double)est.velocity_ms, (double)est.altitude_m);
        check("velocity ~0 at launch", std::fabs(est.velocity_ms) < 0.01f);
        check("altitude ~0 at launch", std::fabs(est.altitude_m)  < 0.05f);
    }

    // ============================================================
    // 12. Baro glitch gate and dt guard
    // ============================================================
    std::printf("Test 12: glitch gate, re-sync, dt guard\n");
    {
        const float GLITCH_HPA = 985.5f;   // ~238 m off, as seen on the bench
        AltEstimator est;
        alt_init(&est, P0);
        for (int i = 0; i < 200; i++) alt_calibrate_sample(&est, 1.0f);
        alt_calibrate_finish(&est);
        for (int i = 0; i < 100; i++) alt_update(&est, P0, 1.0f, DT);

        alt_update(&est, GLITCH_HPA, 1.0f, DT);
        check("single glitch doesn't move velocity", std::fabs(est.velocity_ms) < 0.01f);
        check("single glitch doesn't move altitude", std::fabs(est.altitude_m)  < 0.01f);

        // A sustained offset is real: re-sync after ALT_INNOV_GATE_COUNT
        for (int i = 0; i < ALT_INNOV_GATE_COUNT; i++) alt_update(&est, GLITCH_HPA, 1.0f, DT);
        float glitch_agl = pressure_to_altitude(GLITCH_HPA) - est.ground_altitude_m;
        check("sustained offset re-syncs altitude to baro",
              approx(est.altitude_m, glitch_agl, 0.5f));
        check("re-sync leaves velocity alone", std::fabs(est.velocity_ms) < 0.01f);

        // First loop tick after setup() measures dt from boot — skipped
        float alt_before = est.altitude_m, vel_before = est.velocity_ms;
        alt_update(&est, P0, 1.5f, 4.0f);
        check("multi-second dt skipped",
              est.altitude_m == alt_before && est.velocity_ms == vel_before);
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