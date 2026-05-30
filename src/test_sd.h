#pragma once

#include <Arduino.h>
#include <SD.h>
#include "logger.h"   // SD_CS_PIN, SD_CD_PIN

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

#define SD_TEST_FILENAME    "SD_TEST.TMP"
#define SD_TEST_PAYLOAD     "SDTEST_OK_1234567890"
#define SD_TEST_PAYLOAD_LEN 20

// --------------------------------------------------------
// HELPERS
// --------------------------------------------------------

static void sd_print_pass(const char *test) {
    Serial.print("  [PASS] "); Serial.println(test);
}

static void sd_print_fail(const char *test, const char *reason) {
    Serial.print("  [FAIL] "); Serial.print(test);
    Serial.print(" — "); Serial.println(reason);
}

// --------------------------------------------------------
// TESTS
// --------------------------------------------------------

/**
 * Test 1 — Card detect
 * Pin 24 is pulled HIGH by default; inserting the card pulls it LOW.
 * Fails if no card is physically present.
 */
static bool test_sd_card_detect() {
    Serial.println("  Running: SD card detect (pin 24)");
    pinMode(SD_CD_PIN, INPUT_PULLUP);
    if (digitalRead(SD_CD_PIN) == HIGH) {
        sd_print_fail("Card detect", "pin 24 is HIGH — insert SD card and retry");
        return false;
    }
    sd_print_pass("SD card detect");
    return true;
}

/**
 * Test 2 — Initialisation
 * Calls SD.begin() with the correct CS pin (pin 7).
 */
static bool test_sd_init() {
    Serial.println("  Running: SD init (CS pin 8)");
    if (!SD.begin(SD_CS_PIN)) {
        sd_print_fail("SD init", "SD.begin() failed — check CS wiring and card format (FAT32)");
        return false;
    }
    sd_print_pass("SD init");
    return true;
}

/**
 * Test 3 — Write
 * Creates a small test file and writes a known payload.
 */
static bool test_sd_write() {
    Serial.println("  Running: SD write");

    if (SD.exists(SD_TEST_FILENAME)) SD.remove(SD_TEST_FILENAME);

    File f = SD.open(SD_TEST_FILENAME, FILE_WRITE);
    if (!f) {
        sd_print_fail("SD write", "could not create " SD_TEST_FILENAME);
        return false;
    }
    size_t written = f.print(SD_TEST_PAYLOAD);
    f.flush();
    f.close();

    if (written != SD_TEST_PAYLOAD_LEN) {
        sd_print_fail("SD write", "byte count mismatch — possible card error");
        return false;
    }
    sd_print_pass("SD write");
    return true;
}

/**
 * Test 4 — Read back and verify
 * Opens the file written in test 3 and checks the contents byte-for-byte.
 */
static bool test_sd_read() {
    Serial.println("  Running: SD read-back verify");

    File f = SD.open(SD_TEST_FILENAME, FILE_READ);
    if (!f) {
        sd_print_fail("SD read-back", "could not open " SD_TEST_FILENAME " for reading");
        return false;
    }

    char buf[SD_TEST_PAYLOAD_LEN + 1];
    memset(buf, 0, sizeof(buf));
    size_t got = f.readBytes(buf, SD_TEST_PAYLOAD_LEN);
    f.close();

    Serial.print("    Read: \""); Serial.print(buf); Serial.println("\"");

    if (got != SD_TEST_PAYLOAD_LEN || memcmp(buf, SD_TEST_PAYLOAD, SD_TEST_PAYLOAD_LEN) != 0) {
        sd_print_fail("SD read-back", "content mismatch — card may be corrupted or wiring flaky");
        return false;
    }
    sd_print_pass("SD read-back verify");
    return true;
}

/**
 * Test 5 — File exists / remove
 * Verifies SD.exists() returns true for the test file, then removes it
 * and confirms it is gone. Checks the directory layer works correctly.
 */
static bool test_sd_exists_remove() {
    Serial.println("  Running: SD exists / remove");

    if (!SD.exists(SD_TEST_FILENAME)) {
        sd_print_fail("SD exists", SD_TEST_FILENAME " not found — did write test pass?");
        return false;
    }

    SD.remove(SD_TEST_FILENAME);

    if (SD.exists(SD_TEST_FILENAME)) {
        sd_print_fail("SD remove", "file still present after SD.remove()");
        return false;
    }
    sd_print_pass("SD exists / remove");
    return true;
}

/**
 * Test 6 — Incremental filename generation
 * Simulates what logger_init() does to find the next FLIGHTXXX.CSV slot.
 * Writes two dummy files and checks that the third slot number is correct.
 */
static bool test_sd_filename_sequence() {
    Serial.println("  Running: Incremental filename sequence");

    const char *f1 = "FLIGHT_001.CSV";
    const char *f2 = "FLIGHT_002.CSV";

    if (SD.exists(f1)) SD.remove(f1);
    if (SD.exists(f2)) SD.remove(f2);

    File a = SD.open(f1, FILE_WRITE); if (a) { a.print("x"); a.close(); }
    File b = SD.open(f2, FILE_WRITE); if (b) { b.print("x"); b.close(); }

    // Walk the same logic as find_filename() in logger.cpp
    char found[20];
    bool ok = false;
    for (int i = 1; i <= 999; i++) {
        snprintf(found, sizeof(found), "FLIGHT_%03d.CSV", i);
        if (!SD.exists(found)) { ok = true; break; }
    }

    SD.remove(f1);
    SD.remove(f2);

    Serial.print("    Next slot: "); Serial.println(found);

    if (!ok || strcmp(found, "FLIGHT_003.CSV") != 0) {
        sd_print_fail("Filename sequence", "expected FLIGHT_003.CSV as next slot");
        return false;
    }
    sd_print_pass("Incremental filename sequence");
    return true;
}

// --------------------------------------------------------
// ENTRY POINT
// --------------------------------------------------------

static void run_sd_tests() {
    Serial.println("========================================");
    Serial.println("  SD CARD TEST SUITE");
    Serial.println("  CS=pin 8   CD=pin 24");
    Serial.println("========================================");

    uint8_t passed = 0;
    uint8_t total  = 6;

    if (test_sd_card_detect())        passed++;
    if (test_sd_init())               passed++;
    if (test_sd_write())              passed++;
    if (test_sd_read())               passed++;
    if (test_sd_exists_remove())      passed++;
    if (test_sd_filename_sequence())  passed++;

    Serial.println("----------------------------------------");
    Serial.print("  SD result: ");
    Serial.print(passed); Serial.print("/"); Serial.print(total);
    Serial.println(passed == total ? "  ALL PASSED" : "  FAILURES DETECTED");
    Serial.println("========================================\n");
}
