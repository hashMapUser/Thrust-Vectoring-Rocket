#pragma once

#include <Arduino.h>
#include <math.h>
#include "mmc5603nj.h"

// --------------------------------------------------------
// THRESHOLDS
// --------------------------------------------------------

// Earth's magnetic field strength varies by location but is
// typically 0.25–0.65 Gauss. These bounds catch obvious failures.
#define MAG_MAGNITUDE_MIN   0.10f   // Gauss — well below any real location
#define MAG_MAGNITUDE_MAX   0.80f   // Gauss — well above any real location

// Noise: peak-to-peak variation across a static window
#define MAG_NOISE_MAX       0.05f   // Gauss

// Minimum response when rotating near a ferrous object
#define MAG_RESPONSE_MIN    0.02f   // Gauss

#define MAG_SAMPLE_COUNT    30

// --------------------------------------------------------
// HELPERS
// --------------------------------------------------------

static void mag_print_pass(const char *test) {
    Serial.print("  [PASS] "); Serial.println(test);
}

static void mag_print_fail(const char *test, const char *reason) {
    Serial.print("  [FAIL] "); Serial.print(test);
    Serial.print(" — "); Serial.println(reason);
}

// --------------------------------------------------------
// TESTS
// --------------------------------------------------------

static bool test_mag_init() {
    Serial.println("  Running: MMC5603NJ init / chip ID");
    if (!mmc5603nj_init()) {
        mag_print_fail("Mag init", "mmc5603nj_init() returned false — check I2C address and SDA/SCL");
        return false;
    }
    mag_print_pass("Mag init / chip ID");
    return true;
}

static bool test_mag_read_valid() {
    Serial.println("  Running: Magnetometer single read");
    MMC5603NJ_Data d;
    mmc5603nj_read(&d);
    if (!d.valid) {
        mag_print_fail("Single read", "read returned valid=false — possible I2C timeout");
        return false;
    }
    Serial.print("    mx="); Serial.print(d.mag_x, 4);
    Serial.print(" my="); Serial.print(d.mag_y, 4);
    Serial.print(" mz="); Serial.println(d.mag_z, 4);
    mag_print_pass("Magnetometer single read");
    return true;
}

/**
 * Test 3 — Field magnitude plausibility
 * Earth's field should give a vector magnitude in a known range.
 * Out of range means the 18-bit assembly or zero-offset subtraction is wrong.
 */
static bool test_mag_magnitude() {
    Serial.println("  Running: Field magnitude plausibility");

    MMC5603NJ_Data d;
    mmc5603nj_read(&d);
    if (!d.valid) {
        mag_print_fail("Field magnitude", "read invalid");
        return false;
    }

    float mag = sqrtf(d.mag_x*d.mag_x + d.mag_y*d.mag_y + d.mag_z*d.mag_z);
    Serial.print("    |B| = "); Serial.print(mag, 4); Serial.println(" Gauss");

    if (mag < MAG_MAGNITUDE_MIN || mag > MAG_MAGNITUDE_MAX) {
        Serial.print("    Expected between ");
        Serial.print(MAG_MAGNITUDE_MIN); Serial.print(" and ");
        Serial.println(MAG_MAGNITUDE_MAX);
        mag_print_fail("Field magnitude", "out of range — check 18-bit assembly and zero offset");
        return false;
    }
    mag_print_pass("Field magnitude plausibility");
    return true;
}

/**
 * Test 4 — Static noise floor
 * All axes should be quiet when the board is still.
 */
static bool test_mag_noise() {
    Serial.println("  Running: Magnetometer noise floor");

    float min_x =  9999, max_x = -9999;
    float min_y =  9999, max_y = -9999;
    float min_z =  9999, max_z = -9999;

    for (int i = 0; i < MAG_SAMPLE_COUNT; i++) {
        MMC5603NJ_Data d;
        mmc5603nj_read(&d);
        if (!d.valid) {
            mag_print_fail("Noise floor", "read failed mid-sample");
            return false;
        }
        if (d.mag_x < min_x) min_x = d.mag_x;
        if (d.mag_x > max_x) max_x = d.mag_x;
        if (d.mag_y < min_y) min_y = d.mag_y;
        if (d.mag_y > max_y) max_y = d.mag_y;
        if (d.mag_z < min_z) min_z = d.mag_z;
        if (d.mag_z > max_z) max_z = d.mag_z;
    }

    float noise_x = max_x - min_x;
    float noise_y = max_y - min_y;
    float noise_z = max_z - min_z;

    Serial.print("    P-P noise: x="); Serial.print(noise_x, 4);
    Serial.print(" y="); Serial.print(noise_y, 4);
    Serial.print(" z="); Serial.println(noise_z, 4);

    if (noise_x > MAG_NOISE_MAX || noise_y > MAG_NOISE_MAX || noise_z > MAG_NOISE_MAX) {
        Serial.print("    Threshold: "); Serial.println(MAG_NOISE_MAX);
        mag_print_fail("Noise floor", "noise too high — check I2C lines and power supply");
        return false;
    }
    mag_print_pass("Magnetometer noise floor");
    return true;
}

/**
 * Test 5 — Axis response
 * Bring a phone or small magnet near the sensor on the X axis.
 * Checks that the reading actually changes — confirms all three
 * axes are live and the 18-bit assembly is routing bytes correctly.
 */
static bool test_mag_response() {
    Serial.println("  Running: Magnetometer axis response");

    MMC5603NJ_Data baseline;
    mmc5603nj_read(&baseline);

    Serial.println("    >>> Bring a phone face-down close to the sensor now <<<");
    delay(4000);

    MMC5603NJ_Data stimulated;
    mmc5603nj_read(&stimulated);

    float dx = fabsf(stimulated.mag_x - baseline.mag_x);
    float dy = fabsf(stimulated.mag_y - baseline.mag_y);
    float dz = fabsf(stimulated.mag_z - baseline.mag_z);
    float delta = sqrtf(dx*dx + dy*dy + dz*dz);

    Serial.print("    Baseline:   mx="); Serial.print(baseline.mag_x, 4);
    Serial.print(" my="); Serial.print(baseline.mag_y, 4);
    Serial.print(" mz="); Serial.println(baseline.mag_z, 4);
    Serial.print("    Stimulated: mx="); Serial.print(stimulated.mag_x, 4);
    Serial.print(" my="); Serial.print(stimulated.mag_y, 4);
    Serial.print(" mz="); Serial.println(stimulated.mag_z, 4);
    Serial.print("    Delta magnitude: "); Serial.println(delta, 4);

    // Guard against NaN propagating from failed reads
    if (isnan(delta) || !baseline.valid || !stimulated.valid) {
        mag_print_fail("Axis response", "one or both reads invalid — fix init first");
        return false;
    }
    if (delta < MAG_RESPONSE_MIN) {
        mag_print_fail("Axis response", "no response to stimulus — sensor may be returning stale data");
        return false;
    }
    mag_print_pass("Magnetometer axis response");
    return true;
}

// --------------------------------------------------------
// ENTRY POINT
// --------------------------------------------------------

static void run_mag_tests() {
    Serial.println("========================================");
    Serial.println("  MAGNETOMETER TEST SUITE (MMC5603NJ)");
    Serial.println("  Keep board STILL during tests 1-4");
    Serial.println("========================================");

    uint8_t passed = 0;
    uint8_t total  = 5;

    if (test_mag_init())        passed++;
    if (test_mag_read_valid())  passed++;
    if (test_mag_magnitude())   passed++;
    if (test_mag_noise())       passed++;
    if (test_mag_response())    passed++;

    Serial.println("----------------------------------------");
    Serial.print("  MAG result: ");
    Serial.print(passed); Serial.print("/"); Serial.print(total);
    Serial.println(passed == total ? "  ALL PASSED" : "  FAILURES DETECTED");
    Serial.println("========================================\n");
}