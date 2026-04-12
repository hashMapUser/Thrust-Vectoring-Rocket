#pragma once

#include <Arduino.h>
#include "bmp390.h"

// --------------------------------------------------------
// PASS/FAIL THRESHOLDS
// --------------------------------------------------------

// Plausible sea-level pressure range (hPa)
// Covers extreme weather — if you're at altitude, lower PRESS_MIN
#define PRESS_MIN_HPA     850.0f
#define PRESS_MAX_HPA     1084.0f

// Plausible ambient temperature range (°C)
// Adjust if you're in an unusually cold or hot environment
#define TEMP_MIN_C        -10.0f
#define TEMP_MAX_C         60.0f

// Max pressure noise across a short static window (hPa)
// Should be < 0.05 hPa at rest at 1x OSR
#define PRESS_NOISE_MAX   0.5f

// Number of samples for noise and drift tests
#define BARO_SAMPLE_COUNT 50

// --------------------------------------------------------
// HELPERS
// --------------------------------------------------------

static void baro_print_pass(const char *test) {
    Serial.print("  [PASS] ");
    Serial.println(test);
}

static void baro_print_fail(const char *test, const char *reason) {
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
 * Verifies the sensor responds on I2C and the chip ID matches 0x60.
 */
static bool test_baro_init(BMP390_Calib *cal) {
    Serial.println("  Running: BMP390 init / WHO_AM_I");
    if (!bmp390_init(cal)) {
        baro_print_fail("BMP390 init", "bmp390_init() returned false — check I2C address and SDA/SCL");
        return false;
    }
    baro_print_pass("BMP390 init / WHO_AM_I");
    return true;
}

/**
 * Test 2 — Single read validity
 * Checks that a read completes without error.
 */
static bool test_baro_read_valid(BMP390_Calib *cal) {
    Serial.println("  Running: Barometer single read");
    BMP390_Data d;
    bmp390_read(cal, &d);
    if (!d.valid) {
        baro_print_fail("Barometer single read", "read returned valid=false");
        return false;
    }
    baro_print_pass("Barometer single read");
    return true;
}

/**
 * Test 3 — Temperature plausibility
 * Checks the compensated temperature is within a believable room range.
 * An out-of-range value usually means the calibration parse went wrong.
 */
static bool test_baro_temperature(BMP390_Calib *cal) {
    Serial.println("  Running: Temperature plausibility");
    BMP390_Data d;
    bmp390_read(cal, &d);

    Serial.print("    Temperature: "); Serial.print(d.temperature_c, 2); Serial.println(" °C");

    if (!d.valid) {
        baro_print_fail("Temperature plausibility", "read invalid");
        return false;
    }
    if (d.temperature_c < TEMP_MIN_C || d.temperature_c > TEMP_MAX_C) {
        Serial.print("    Expected between "); Serial.print(TEMP_MIN_C);
        Serial.print(" and "); Serial.println(TEMP_MAX_C);
        baro_print_fail("Temperature plausibility", "value out of range — check calibration math");
        return false;
    }
    baro_print_pass("Temperature plausibility");
    return true;
}

/**
 * Test 4 — Pressure plausibility
 * Checks the compensated pressure is within a physically reasonable range.
 */
static bool test_baro_pressure(BMP390_Calib *cal) {
    Serial.println("  Running: Pressure plausibility");
    BMP390_Data d;
    bmp390_read(cal, &d);

    float hpa = d.pressure_pa / 100.0f;
    Serial.print("    Pressure: "); Serial.print(hpa, 2); Serial.println(" hPa");

    if (!d.valid) {
        baro_print_fail("Pressure plausibility", "read invalid");
        return false;
    }
    if (hpa < PRESS_MIN_HPA || hpa > PRESS_MAX_HPA) {
        Serial.print("    Expected between "); Serial.print(PRESS_MIN_HPA);
        Serial.print(" and "); Serial.println(PRESS_MAX_HPA);
        baro_print_fail("Pressure plausibility", "value out of range — check calibration math");
        return false;
    }
    baro_print_pass("Pressure plausibility");
    return true;
}

/**
 * Test 5 — Pressure noise floor
 * Takes 50 readings and measures peak-to-peak variation.
 * Large noise means the I2C bus is unstable or the sensor is disturbed.
 */
static bool test_baro_noise(BMP390_Calib *cal) {
    Serial.println("  Running: Pressure noise floor");

    float min_hpa =  9999.0f;
    float max_hpa = -9999.0f;

    for (int i = 0; i < BARO_SAMPLE_COUNT; i++) {
        BMP390_Data d;
        bmp390_read(cal, &d);
        if (!d.valid) {
            baro_print_fail("Pressure noise floor", "read failed mid-sample");
            return false;
        }
        float hpa = d.pressure_pa / 100.0f;
        if (hpa < min_hpa) min_hpa = hpa;
        if (hpa > max_hpa) max_hpa = hpa;
        delay(20);
    }

    float noise = max_hpa - min_hpa;
    Serial.print("    Peak-to-peak noise: "); Serial.print(noise, 4); Serial.println(" hPa");

    if (noise > PRESS_NOISE_MAX) {
        Serial.print("    Threshold: "); Serial.println(PRESS_NOISE_MAX);
        baro_print_fail("Pressure noise floor", "noise too high — check I2C lines for interference");
        return false;
    }
    baro_print_pass("Pressure noise floor");
    return true;
}

/**
 * Test 6 — Pressure/altitude change detection
 * Prompts the user to cover the sensor with their hand (changes local
 * pressure slightly) and checks the sensor actually responds to a
 * real-world stimulus.
 */
static bool test_baro_response(BMP390_Calib *cal) {
    Serial.println("  Running: Pressure response to stimulus");

    // Baseline
    BMP390_Data baseline;
    bmp390_read(cal, &baseline);
    float base_hpa = baseline.pressure_pa / 100.0f;

    Serial.println("    >>> Cup your hand tightly over the sensor now <<<");
    delay(4000);

    BMP390_Data stimulated;
    bmp390_read(cal, &stimulated);
    float stim_hpa = stimulated.pressure_pa / 100.0f;

    float delta = fabsf(stim_hpa - base_hpa);
    Serial.print("    Baseline: "); Serial.print(base_hpa, 3); Serial.println(" hPa");
    Serial.print("    Stimulated: "); Serial.print(stim_hpa, 3); Serial.println(" hPa");
    Serial.print("    Delta: "); Serial.print(delta, 4); Serial.println(" hPa");

    if (delta < 0.02f) {
        baro_print_fail("Pressure response", "no response detected — sensor may be stuck or not reading live data");
        return false;
    }
    baro_print_pass("Pressure response to stimulus");
    return true;
}

// --------------------------------------------------------
// TEST SUITE ENTRY POINT
// --------------------------------------------------------

void run_baro_tests(BMP390_Calib *cal) {
    Serial.println("========================================");
    Serial.println("  BAROMETER TEST SUITE (BMP390)");
    Serial.println("  Keep the board STILL during tests 1-5");
    Serial.println("========================================");

    uint8_t passed = 0;
    uint8_t total  = 6;

    if (test_baro_init(cal))           passed++;
    if (test_baro_read_valid(cal))     passed++;
    if (test_baro_temperature(cal))    passed++;
    if (test_baro_pressure(cal))       passed++;
    if (test_baro_noise(cal))          passed++;
    if (test_baro_response(cal))       passed++;

    Serial.println("----------------------------------------");
    Serial.print("  BARO result: ");
    Serial.print(passed); Serial.print("/"); Serial.print(total);
    Serial.println(passed == total ? "  ALL PASSED" : "  FAILURES DETECTED");
    Serial.println("========================================\n");
}