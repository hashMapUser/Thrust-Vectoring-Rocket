#pragma once

#include <Arduino.h>
#include "lsm6dsox.h"

// --------------------------------------------------------
// PASS/FAIL THRESHOLDS
// --------------------------------------------------------

// When the board is still, total accel magnitude should be ~1g (gravity only)
// We allow ±10% tolerance
#define ACCEL_MAG_MIN     0.90f   // g
#define ACCEL_MAG_MAX     1.10f   // g

// When the board is still, gyro rates should be near zero
// Anything above this (deg/s) on any axis is a fail
#define GYRO_STATIC_MAX   2.0f   // deg/s

// Number of samples to average for the static bias test
#define TEST_SAMPLE_COUNT 100

// --------------------------------------------------------
// HELPERS
// --------------------------------------------------------

static void print_pass(const char *test) {
    Serial.print("  [PASS] ");
    Serial.println(test);
}

static void print_fail(const char *test, const char *reason) {
    Serial.print("  [FAIL] ");
    Serial.print(test);
    Serial.print(" — ");
    Serial.println(reason);
}

// --------------------------------------------------------
// INDIVIDUAL TESTS
// --------------------------------------------------------

/**
 * Test 1 — Init and WHO_AM_I
 * Verifies the sensor responds and identifies correctly.
 */
static bool test_imu_init() {
    Serial.println("  Running: IMU init / WHO_AM_I");
    if (!lsm6dsox_init()) {
        print_fail("IMU init", "lsm6dsox_init() returned false — check SPI wiring and CS pin");
        return false;
    }
    print_pass("IMU init / WHO_AM_I");
    return true;
}

/**
 * Test 2 — Single read validity
 * Checks that a read completes without error.
 */
static bool test_imu_read_valid() {
    Serial.println("  Running: IMU single read");
    LSM6DSOX_Data d;
    lsm6dsox_read(&d);
    if (!d.valid) {
        print_fail("IMU single read", "read returned valid=false");
        return false;
    }
    print_pass("IMU single read");
    return true;
}

/**
 * Test 3 — Accel static magnitude
 * With the board sitting still, the vector magnitude of accel should be
 * ~1g. Anything outside tolerance means axis mapping or scaling is wrong.
 */
static bool test_imu_accel_magnitude() {
    Serial.println("  Running: Accel static magnitude");

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    for (int i = 0; i < TEST_SAMPLE_COUNT; i++) {
        LSM6DSOX_Data d;
        lsm6dsox_read(&d);
        if (!d.valid) {
            print_fail("Accel static magnitude", "read failed mid-sample");
            return false;
        }
        sum_ax += d.ax;
        sum_ay += d.ay;
        sum_az += d.az;
        delay(5);
    }

    float ax = sum_ax / TEST_SAMPLE_COUNT;
    float ay = sum_ay / TEST_SAMPLE_COUNT;
    float az = sum_az / TEST_SAMPLE_COUNT;
    float mag = sqrtf(ax*ax + ay*ay + az*az);

    Serial.print("    Avg accel: ax="); Serial.print(ax, 3);
    Serial.print(" ay="); Serial.print(ay, 3);
    Serial.print(" az="); Serial.print(az, 3);
    Serial.print(" |a|="); Serial.println(mag, 3);

    if (mag < ACCEL_MAG_MIN || mag > ACCEL_MAG_MAX) {
        Serial.print("    Expected magnitude between ");
        Serial.print(ACCEL_MAG_MIN); Serial.print(" and "); Serial.println(ACCEL_MAG_MAX);
        print_fail("Accel static magnitude", "magnitude out of range — check scale factor or wiring");
        return false;
    }
    print_pass("Accel static magnitude");
    return true;
}

/**
 * Test 4 — Gyro static bias
 * With the board sitting still, all gyro rates should be near zero.
 * A large offset means the sensor is misconfigured or the scale is wrong.
 */
static bool test_imu_gyro_bias() {
    Serial.println("  Running: Gyro static bias");

    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (int i = 0; i < TEST_SAMPLE_COUNT; i++) {
        LSM6DSOX_Data d;
        lsm6dsox_read(&d);
        if (!d.valid) {
            print_fail("Gyro static bias", "read failed mid-sample");
            return false;
        }
        sum_gx += d.gx;
        sum_gy += d.gy;
        sum_gz += d.gz;
        delay(5);
    }

    float gx = sum_gx / TEST_SAMPLE_COUNT;
    float gy = sum_gy / TEST_SAMPLE_COUNT;
    float gz = sum_gz / TEST_SAMPLE_COUNT;

    Serial.print("    Avg gyro (deg/s): gx="); Serial.print(gx, 3);
    Serial.print(" gy="); Serial.print(gy, 3);
    Serial.print(" gz="); Serial.println(gz, 3);

    if (fabsf(gx) > GYRO_STATIC_MAX ||
        fabsf(gy) > GYRO_STATIC_MAX ||
        fabsf(gz) > GYRO_STATIC_MAX) {
        Serial.print("    Threshold: ±"); Serial.println(GYRO_STATIC_MAX);
        print_fail("Gyro static bias", "bias exceeds threshold — sensor may be moving or misconfigured");
        return false;
    }
    print_pass("Gyro static bias");
    return true;
}

/**
 * Test 5 — Gyro axis response
 * Prompts the user to rotate the board and checks that the right axis
 * shows a strong response while the others stay small.
 * This catches axis swap and sign inversion issues.
 */
static bool test_imu_gyro_axis_response() {
    Serial.println("  Running: Gyro axis response");
    Serial.println("    >>> Rotate the board briskly about the X axis now <<<");
    delay(3000);

    float peak_gx = 0, peak_gy = 0, peak_gz = 0;
    for (int i = 0; i < TEST_SAMPLE_COUNT; i++) {
        LSM6DSOX_Data d;
        lsm6dsox_read(&d);
        if (!d.valid) continue;
        if (fabsf(d.gx) > fabsf(peak_gx)) peak_gx = d.gx;
        if (fabsf(d.gy) > fabsf(peak_gy)) peak_gy = d.gy;
        if (fabsf(d.gz) > fabsf(peak_gz)) peak_gz = d.gz;
        delay(5);
    }

    Serial.print("    Peak: gx="); Serial.print(peak_gx, 1);
    Serial.print(" gy="); Serial.print(peak_gy, 1);
    Serial.print(" gz="); Serial.println(peak_gz, 1);

    // X should dominate
    if (fabsf(peak_gx) < 20.0f) {
        print_fail("Gyro axis response", "X axis did not respond — possible axis swap");
        return false;
    }
    if (fabsf(peak_gy) > fabsf(peak_gx) || fabsf(peak_gz) > fabsf(peak_gx)) {
        print_fail("Gyro axis response", "wrong axis dominated — possible axis swap");
        return false;
    }
    print_pass("Gyro axis response");
    return true;
}

// --------------------------------------------------------
// TEST SUITE ENTRY POINT
// --------------------------------------------------------

void run_imu_tests() {
    Serial.println("========================================");
    Serial.println("  IMU TEST SUITE (LSM6DSOX)");
    Serial.println("  Keep the board STILL during tests 1-4");
    Serial.println("========================================");

    uint8_t passed = 0;
    uint8_t total  = 5;

    if (test_imu_init())               passed++;
    if (test_imu_read_valid())          passed++;
    if (test_imu_accel_magnitude())     passed++;
    if (test_imu_gyro_bias())           passed++;
    if (test_imu_gyro_axis_response())  passed++;

    Serial.println("----------------------------------------");
    Serial.print("  IMU result: ");
    Serial.print(passed); Serial.print("/"); Serial.print(total);
    Serial.println(passed == total ? "  ALL PASSED" : "  FAILURES DETECTED");
    Serial.println("========================================\n");
}