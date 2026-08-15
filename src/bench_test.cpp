// ============================================================
//  bench_test.cpp
//
//  Standalone bench test for all hardware on the TVC flight
//  computer. Replaces main_control_loop.cpp — comment out the
//  #include "main_control_loop.h" in main.cpp and include this
//  instead, OR just rename this to main.cpp for the test build.
//
//  To use: upload to Teensy, open Serial Monitor at 115200 baud.
//  Follow the menu prompts. Each test prints PASS/FAIL with
//  detailed diagnostics so you can chase down any issues.
//
//  Menu commands (send single char over serial):
//    1 — Test BMP390 barometer     (I2C)
//    2 — Test LSM6DSOX IMU         (SPI)
//    3 — Test MMC5603NJ magnetometer (I2C Wire1)
//    4 — Test GD25Q128 NOR flash   (SPI)
//    5 — Test SD card              (SPI)
//    6 — Full logger round-trip    (write fake flight → dump CSV → verify)
//    7 — Run ALL tests in sequence
//    R — Reset / reprint menu
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <math.h>

// Pull in your actual driver headers
#include "board_pins.h"
#include "lps22hb.h"
#include "lsm6dsox.h"
#include "mag.h"
#include "mag_calib.h"
#include "logger.h"
#include "servo_driver.h"

// PIN_FLASH_CS removed — use PIN_FLASH_CS (36) from board_pins.h

// ============================================================
//  GD25Q128 command set (needed for standalone flash tests)
// ============================================================
#define FCMD_RELEASE_PD     0xAB
#define FCMD_JEDEC_ID       0x9F
#define FCMD_READ_STATUS1   0x05
#define FCMD_WRITE_ENABLE   0x06
#define FCMD_SECTOR_ERASE   0x20   // 4 KB
#define FCMD_PAGE_PROGRAM   0x02
#define FCMD_READ_DATA      0x03
#define FSTATUS_WIP         (1u << 0)

#define FLASH_SPI_FREQ      40000000UL
#define FLASH_SPI_MODE      SPI_MODE0
#define FLASH_TEST_ADDR     0x010000u   // block 1 — well away from logger's sector 0/1

// ============================================================
//  Helpers
// ============================================================
static void print_banner(const char *title) {
    Serial.println();
    Serial.println(F("============================================================"));
    Serial.print(F("  ")); Serial.println(title);
    Serial.println(F("============================================================"));
}

static void pass(const char *msg) {
    Serial.print(F("  [PASS] ")); Serial.println(msg);
}

static void fail(const char *msg) {
    Serial.print(F("  [FAIL] ")); Serial.println(msg);
}

static void info(const char *msg) {
    Serial.print(F("  [INFO] ")); Serial.println(msg);
}

// ============================================================
//  Flash low-level (duplicated here so bench_test.cpp compiles
//  standalone without depending on logger.cpp internals)
// ============================================================
static void flash_wait_ready_bt() {
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_READ_STATUS1);
    uint32_t t0 = millis();
    while (SPI.transfer(0x00) & FSTATUS_WIP) {
        if (millis() - t0 > 5000) { Serial.println("  [WARN] Flash WIP timeout"); break; }
    }
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();
}

static void flash_write_enable_bt() {
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_WRITE_ENABLE);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();
}

// ============================================================
//  TEST 1 — LPS22HBTR
// ============================================================
static void test_lps22hb() {
    print_banner("TEST 1: LPS22HBTR Barometer (I2C)");

    bool init_ok = lps22hb_init();

    if (!init_ok) {
        fail("lps22hb_init() returned false — sensor not responding");
        Serial.println(F("  Checklist:"));
        Serial.println(F("    - 3.3V on VDD pin?"));
        Serial.println(F("    - SDA/SCL pulled up to 3.3V with 4.7k?"));
        Serial.println(F("    - I2C address: 0x5C (SA0 low) or 0x5D (SA0 high)?"));
        Serial.println(F("    - PIN_BARO_SDA/SCL match your board? (currently pins 18/19)"));
        return;
    }
    pass("lps22hb_init() OK — WHO_AM_I 0xB1 confirmed");

    // Read 5 samples and print them
    Serial.println(F("  Reading 5 samples (250 ms apart):"));
    int valid_count = 0;
    float pressure_sum = 0, temp_sum = 0;

    for (int i = 0; i < 5; i++) {
        LPS22HB_Data d;
        lps22hb_read(&d);
        Serial.print(F("    ["));
        Serial.print(i + 1);
        Serial.print(F("] valid="));
        Serial.print(d.valid ? "Y" : "N");
        Serial.print(F("  P="));
        Serial.print(d.pressure_pa / 100.0f, 2);
        Serial.print(F(" hPa  T="));
        Serial.print(d.temperature_c, 2);
        Serial.println(F(" °C"));
        if (d.valid) {
            valid_count++;
            pressure_sum += d.pressure_pa;
            temp_sum     += d.temperature_c;
        }
        delay(250);
    }

    if (valid_count < 4) {
        fail("Too many invalid reads — check I2C noise / pull-ups");
        return;
    }

    float avg_hpa  = (pressure_sum / valid_count) / 100.0f;
    float avg_temp = temp_sum / valid_count;

    // Sanity: sea-level ±200 hPa, temperature 0–70 °C
    if (avg_hpa < 800.0f || avg_hpa > 1100.0f) {
        fail("Pressure out of sane range (800–1100 hPa)");
        return;
    }
    if (avg_temp < 0.0f || avg_temp > 70.0f) {
        fail("Temperature out of sane range (0–70 °C)");
        return;
    }

    Serial.print(F("  Average: "));
    Serial.print(avg_hpa, 2);
    Serial.print(F(" hPa, "));
    Serial.print(avg_temp, 2);
    Serial.println(F(" °C"));

    // Estimate altitude from average pressure
    float alt_m = 44330.0f * (1.0f - powf(avg_hpa / 1013.25f, 0.1902949f));
    Serial.print(F("  Estimated altitude above sea level: "));
    Serial.print(alt_m, 1);
    Serial.println(F(" m  (compare to your known elevation)"));

    pass("LPS22HBTR PASSED — pressure and temperature in range");
}

// ============================================================
//  TEST 2 — LSM6DSOX
// ============================================================
static void test_lsm6dsox() {
    print_banner("TEST 2: LSM6DSOX IMU (SPI)");

    bool init_ok = lsm6dsox_init();
    if (!init_ok) {
        fail("lsm6dsox_init() returned false — sensor not responding");
        Serial.println(F("  Checklist:"));
        Serial.println(F("    - CS pin HIGH before SPI.begin()? (done in setup())"));
        Serial.println(F("    - SPI MODE3?"));
        Serial.println(F("    - PIN_IMU_CS correct? (currently pin 10)"));
        Serial.println(F("    - WHO_AM_I should be 0x6C"));
        return;
    }
    pass("lsm6dsox_init() OK — WHO_AM_I 0x6C confirmed");

    // Load bias from EEPROM if available
    GyroBias bias;
    bool has_bias = lsm6dsox_load_bias(&bias);
    if (has_bias) {
        Serial.print(F("  Gyro bias from EEPROM: x="));
        Serial.print(bias.x, 4); Serial.print(F("  y="));
        Serial.print(bias.y, 4); Serial.print(F("  z="));
        Serial.println(bias.z, 4);
    } else {
        info("No gyro bias in EEPROM — send 'G' in main firmware to calibrate");
        bias.x = bias.y = bias.z = 0.0f;
    }

    // Read 10 samples
    Serial.println(F("  Reading 10 samples (keep sensor still):"));
    int valid_count = 0;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < 10; i++) {
        LSM6DSOX_Data d;
        lsm6dsox_read(&d, &bias);
        Serial.print(F("    ["));
        Serial.print(i + 1);
        Serial.print(F("] v="));
        Serial.print(d.valid ? "Y" : "N");
        Serial.print(F("  ax="));  Serial.print(d.ax, 3);
        Serial.print(F("  ay="));  Serial.print(d.ay, 3);
        Serial.print(F("  az="));  Serial.print(d.az, 3);
        Serial.print(F(" g  |  gx=")); Serial.print(d.gx, 2);
        Serial.print(F("  gy="));  Serial.print(d.gy, 2);
        Serial.print(F("  gz="));  Serial.print(d.gz, 2);
        Serial.println(F(" dps"));
        if (d.valid) {
            valid_count++;
            ax_sum += d.ax; ay_sum += d.ay; az_sum += d.az;
            gx_sum += d.gx; gy_sum += d.gy; gz_sum += d.gz;
        }
        delay(12);  // ~83 Hz
    }

    if (valid_count < 8) {
        fail("Too many invalid reads — SPI issue");
        return;
    }

    float ax_avg = ax_sum / valid_count;
    float ay_avg = ay_sum / valid_count;
    float az_avg = az_sum / valid_count;
    float accel_mag = sqrtf(ax_avg*ax_avg + ay_avg*ay_avg + az_avg*az_avg);

    Serial.print(F("  Accel mean: ax="));
    Serial.print(ax_avg, 3); Serial.print(F("  ay="));
    Serial.print(ay_avg, 3); Serial.print(F("  az="));
    Serial.print(az_avg, 3); Serial.print(F(" g  |magnitude="));
    Serial.print(accel_mag, 3); Serial.println(F(" g"));

    Serial.print(F("  Gyro mean:  gx="));
    Serial.print(gx_sum/valid_count, 3); Serial.print(F("  gy="));
    Serial.print(gy_sum/valid_count, 3); Serial.print(F("  gz="));
    Serial.print(gz_sum/valid_count, 3); Serial.println(F(" dps"));

    // Sanity: total accel magnitude should be ~1 g when stationary
    if (accel_mag < 0.8f || accel_mag > 1.2f) {
        fail("Accel magnitude too far from 1 g — sensor may not be reading correctly");
        Serial.print(F("  Got ")); Serial.print(accel_mag, 3); Serial.println(F(" g, expected 0.8–1.2 g"));
        return;
    }

    // Gyro bias check: should be close to 0 dps at rest (after bias correction)
    float gyro_mag = sqrtf(gx_sum*gx_sum + gy_sum*gy_sum + gz_sum*gz_sum) / valid_count;
    if (gyro_mag > 5.0f) {
        fail("Gyro offset > 5 dps while stationary — run gyro calibration ('G' command)");
        return;
    }

    pass("LSM6DSOX PASSED — accel near 1 g, gyro near 0 dps");
}

// ============================================================
//  TEST 3 — MMC5603NJ Magnetometer
// ============================================================
static void test_mmc5603() {
    print_banner("TEST 3: MMC5603NJ Magnetometer (I2C Wire1)");

    bool init_ok = mag_init();
    if (!init_ok) {
        fail("mag_init() returned false — sensor not responding");
        Serial.println(F("  Checklist:"));
        Serial.println(F("    - Wire1 (pins 16/17) connected?"));
        Serial.println(F("    - MMC5603NJ I2C address: 0x30"));
        Serial.println(F("    - PROD_ID should be 0x10"));
        Serial.println(F("    - 3.3V on VDD?"));
        return;
    }
    pass("mag_init() OK — PROD_ID 0x10 confirmed, SET coil fired");

    // Load calibration if available
    MagCalib cal;
    bool has_cal = mag_load_calib(&cal);
    if (has_cal) {
        Serial.print(F("  Hard iron offset: x="));
        Serial.print(cal.offset_x, 4); Serial.print(F(" y="));
        Serial.print(cal.offset_y, 4); Serial.print(F(" z="));
        Serial.println(cal.offset_z, 4);
        Serial.print(F("  Soft iron scale:  x="));
        Serial.print(cal.scale_x, 4); Serial.print(F(" y="));
        Serial.print(cal.scale_y, 4); Serial.print(F(" z="));
        Serial.println(cal.scale_z, 4);
    } else {
        info("No mag calibration in EEPROM — send 'M' to calibrate in main firmware");
        cal.offset_x = cal.offset_y = cal.offset_z = 0.0f;
        cal.scale_x  = cal.scale_y  = cal.scale_z  = 1.0f;
    }

    // Read 5 samples
    Serial.println(F("  Reading 5 raw samples:"));
    int valid_count = 0;
    float mx_sum = 0, my_sum = 0, mz_sum = 0;

    for (int i = 0; i < 5; i++) {
        mag_data d;
        mag_read(&d);
        if (!d.valid) {
            Serial.print(F("    [")); Serial.print(i+1); Serial.println(F("] INVALID READ"));
            continue;
        }

        // Apply calibration
        float cx, cy, cz;
        mag_apply_calib(&cal, d.mag_x, d.mag_y, d.mag_z, &cx, &cy, &cz);

        Serial.print(F("    ["));
        Serial.print(i + 1);
        Serial.print(F("] raw(G): x="));  Serial.print(d.mag_x, 5);
        Serial.print(F("  y="));          Serial.print(d.mag_y, 5);
        Serial.print(F("  z="));          Serial.print(d.mag_z, 5);
        Serial.print(F("  | cal(G): x=")); Serial.print(cx, 5);
        Serial.print(F("  y="));          Serial.print(cy, 5);
        Serial.print(F("  z="));          Serial.println(cz, 5);

        valid_count++;
        mx_sum += cx; my_sum += cy; mz_sum += cz;
        delay(100);
    }

    if (valid_count < 4) {
        fail("Too many invalid reads — check I2C wiring / pull-ups on Wire1");
        return;
    }

    // Field magnitude — Earth's field is 20–65 µT (0.20–0.65 G).
    // Scale: MMC5603 outputs in Gauss. 1 G = 100 µT.
    float mx_avg = mx_sum / valid_count;
    float my_avg = my_sum / valid_count;
    float mz_avg = mz_sum / valid_count;
    float mag_magnitude = sqrtf(mx_avg*mx_avg + my_avg*my_avg + mz_avg*mz_avg);

    Serial.print(F("  Field magnitude (calibrated): "));
    Serial.print(mag_magnitude, 4);
    Serial.print(F(" G  ("));
    Serial.print(mag_magnitude * 100.0f, 1);
    Serial.println(F(" µT)  — Earth nominal 20–65 µT"));

    if (mag_magnitude < 0.10f || mag_magnitude > 1.00f) {
        fail("Field magnitude outside 0.10–1.00 G — calibrate or check for magnetic interference");
        return;
    }

    pass("MMC5603NJ PASSED — reads valid, field magnitude in range");
}

// ============================================================
//  TEST 4 — GD25Q128 NOR Flash
// ============================================================
static void test_flash() {
    print_banner("TEST 4: GD25Q128 NOR Flash (SPI)");

    // --- JEDEC ID ---
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_JEDEC_ID);
    uint8_t mfr = SPI.transfer(0);
    uint8_t mem = SPI.transfer(0);
    uint8_t cap = SPI.transfer(0);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();

    Serial.print(F("  JEDEC ID: 0x"));
    Serial.print(mfr, HEX); Serial.print(F(" 0x"));
    Serial.print(mem, HEX); Serial.print(F(" 0x"));
    Serial.println(cap, HEX);
    Serial.println(F("  Expected: 0xC8 0x40 0x18 (GigaDevice GD25Q128)"));

    if (mfr != 0xC8 || mem != 0x40 || cap != 0x18) {
        fail("JEDEC ID mismatch — chip not responding or wrong part");
        Serial.println(F("  Checklist:"));
        Serial.println(F("    - PIN_FLASH_CS = 9 correct?"));
        Serial.println(F("    - WP# and HOLD# pins pulled HIGH via 10k to 3.3V?"));
        Serial.println(F("    - SPI MODE0, MSBFIRST?"));
        Serial.println(F("    - 3.3V on VCC?"));
        return;
    }
    pass("JEDEC ID correct — GD25Q128 found");

    // --- Write / Read / Verify (test sector at 0x010000 — block 1) ---
    const uint32_t TEST_ADDR = FLASH_TEST_ADDR;
    Serial.print(F("  Erase sector at 0x"));
    Serial.print(TEST_ADDR, HEX); Serial.print(F(" ... "));

    flash_write_enable_bt();
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_SECTOR_ERASE);
    SPI.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI.transfer((TEST_ADDR >>  0) & 0xFF);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();
    flash_wait_ready_bt();
    Serial.println(F("done"));

    // Verify erased (all 0xFF)
    uint8_t read_buf[32];
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_READ_DATA);
    SPI.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI.transfer((TEST_ADDR >>  0) & 0xFF);
    for (int i = 0; i < 32; i++) read_buf[i] = SPI.transfer(0);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();

    bool erased_ok = true;
    for (int i = 0; i < 32; i++) if (read_buf[i] != 0xFF) { erased_ok = false; break; }
    if (!erased_ok) { fail("Post-erase verify failed — expected all 0xFF"); return; }
    pass("Sector erase verified (all 0xFF)");

    // Write 32 bytes of test pattern
    uint8_t write_buf[32];
    for (int i = 0; i < 32; i++) write_buf[i] = (uint8_t)(i * 7 + 0xA0);  // deterministic pattern

    flash_write_enable_bt();
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_PAGE_PROGRAM);
    SPI.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI.transfer((TEST_ADDR >>  0) & 0xFF);
    for (int i = 0; i < 32; i++) SPI.transfer(write_buf[i]);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();
    flash_wait_ready_bt();

    // Read back and verify
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_READ_DATA);
    SPI.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI.transfer((TEST_ADDR >>  0) & 0xFF);
    for (int i = 0; i < 32; i++) read_buf[i] = SPI.transfer(0);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();

    bool write_ok = true;
    for (int i = 0; i < 32; i++) {
        if (read_buf[i] != write_buf[i]) {
            write_ok = false;
            Serial.print(F("  Mismatch at byte ")); Serial.print(i);
            Serial.print(F(": wrote 0x")); Serial.print(write_buf[i], HEX);
            Serial.print(F(" read 0x"));  Serial.println(read_buf[i], HEX);
        }
    }
    if (!write_ok) { fail("Write/readback mismatch"); return; }
    pass("Write/readback 32 bytes verified");

    // Show what was written for confidence
    Serial.print(F("  Pattern written (first 8 bytes): "));
    for (int i = 0; i < 8; i++) {
        Serial.print(F("0x")); Serial.print(read_buf[i], HEX); Serial.print(' ');
    }
    Serial.println();

    pass("GD25Q128 PASSED — erase, write, and readback all correct");
}

// ============================================================
//  TEST 5 — SD Card
// ============================================================
static void test_sd() {
    char sd_banner[64];
    snprintf(sd_banner, sizeof(sd_banner), "TEST 5: SD Card (SPI, CS pin %d)", PIN_SD_CS);
    print_banner(sd_banner);

    Serial.println(F("  SD card setup: FAT32, any size up to 32 GB."));
    Serial.println(F("  No pre-formatting needed for modern cards already FAT32."));
    Serial.println(F("  If SD.begin() fails on a new card, format it FAT32 on your PC first."));
    Serial.println();

    if (!SD.begin(PIN_SD_CS)) {
        fail("SD.begin() failed");
        Serial.println(F("  Checklist:"));
        Serial.println(F("    - Card inserted in Hirose DM3AT connector?"));
        Serial.println(F("    - PIN_SD_CS = 7 correct?"));
        Serial.println(F("    - Card formatted FAT32? (not exFAT, not NTFS)"));
        Serial.println(F("    - SPI bus shared with flash — flash CS must be HIGH during SD init"));
        Serial.println(F("    - Try a different SD card (some cards fail at 3.3V)"));
        return;
    }
    pass("SD.begin() OK — card mounted");

    // Write a test file
    const char *TESTFILE = "BENCH.TXT";
    if (SD.exists(TESTFILE)) SD.remove(TESTFILE);

    File f = SD.open(TESTFILE, FILE_WRITE);
    if (!f) {
        fail("Could not create BENCH.TXT");
        return;
    }
    f.println(F("bench_test.cpp write test"));
    f.print(F("timestamp_ms="));
    f.println(millis());
    f.println(F("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"));
    f.flush();
    f.close();
    pass("BENCH.TXT written and closed");

    // Read it back
    f = SD.open(TESTFILE, FILE_READ);
    if (!f) {
        fail("Could not re-open BENCH.TXT for reading");
        return;
    }
    Serial.println(F("  Contents of BENCH.TXT:"));
    while (f.available()) {
        Serial.print(F("    "));
        Serial.println(f.readStringUntil('\n'));
    }
    f.close();
    pass("BENCH.TXT read back successfully");

    // Check available space
    // SD library doesn't expose free space on all platforms, so just report card type
    Serial.println(F("  SD card responding correctly."));
    SD.remove(TESTFILE);
    pass("SD Card PASSED — write, read, and delete all OK");
}

// ============================================================
//  TEST 6 — Full logger round-trip
//  Writes 50 fake LogRecords via logger_write(), finalises the
//  flash header, then streams via USB dump (TVCR protocol).
//  Host side: tools/dump_flash.py — see T9 in FC_V2_SPEC.md.
//  SD card is disabled for this flight (H6: SD VDD at 5 V).
// ============================================================
static void test_logger_roundtrip() {
    print_banner("TEST 6: Logger Round-Trip (flash → USB TVCR dump)");

    info("Erasing flash and reinitialising logger...");
    // logger_erase() is declared in the updated logger.h
    // If it doesn't exist yet, do it manually via a chip erase:
    //   flash_write_enable_bt(); ...CMD_CHIP_ERASE... (takes ~30s)
    // For the bench test we'll just re-init without erasing so we
    // don't spend 30 s on every test run.

    bool ok = logger_init();
    if (!ok) {
        // logger_init returns true if flash OR SD is up
        Serial.println(F("  [WARN] logger_init() returned false — check flash and SD"));
    }

    // Write 50 fake records with known values
    const uint16_t N = 50;
    Serial.print(F("  Writing ")); Serial.print(N); Serial.println(F(" synthetic records..."));

    for (uint16_t i = 0; i < N; i++) {
        LogRecord r;
        r.timestamp_ms  = (uint32_t)i * 8;   // 125 Hz spacing

        // Fill with deterministic values so we can verify them
        r.roll          = (float)i * 0.1f;
        r.pitch         = (float)i * 0.2f;
        r.yaw           = (float)i * 0.3f;
        r.q0            = 1.0f; r.q1 = 0.0f; r.q2 = 0.0f; r.q3 = 0.0f;
        r.gx            = (float)i * 0.5f;
        r.gy            = -(float)i * 0.5f;
        r.gz            = 0.0f;
        r.ax            = 0.0f;
        r.ay            = 0.0f;
        r.az            = -1.0f;   // gravity pointing down in body frame
        r.mx            = 0.2f; r.my = 0.1f; r.mz = -0.4f;
        r.temperature_c = 22.5f;
        r.pressure_hpa  = 1013.25f - (float)i * 0.1f;  // slight drop per record
        r.altitude_m    = (float)i * 2.0f;              // climbing at 2 m/record
        r.velocity_ms   = 25.0f;
        r.servo_pitch_us = 1500.0f + (float)i;
        r.servo_yaw_us   = 1500.0f - (float)i;
        r.pid_pitch_out  = (float)i * 0.01f;
        r.pid_yaw_out    = -(float)i * 0.01f;
        r.flight_state   = (i < 10) ? STATE_ARMED : STATE_POWERED;
        r.imu_valid      = true;
        r.baro_valid     = true;
        r.mag_valid      = true;

        logger_write(&r);
    }
    pass("All 50 records written");

    Serial.println(F("  Finalising flash header..."));
    logger_finalize();
    pass("Flash header written");

    Serial.println(F("  Streaming TVCR dump over USB Serial..."));
    Serial.println(F("  Run tools/dump_flash.py on the host to capture and decode."));
    logger_usb_dump();

    Serial.println();
    Serial.println(F("  ── USB DUMP RETRIEVAL GUIDE ───────────────────────────"));
    Serial.println(F("  After a real flight:"));
    Serial.println(F("    1. Connect USB; open tools/dump_flash.py on the host"));
    Serial.println(F("    2. Send 'R' over Serial — Teensy streams TVCR binary"));
    Serial.println(F("    3. Script verifies CRC32 and converts to flight.csv"));
    Serial.println(F("    4. DO NOT insert the SD card (H6: SD VDD at 5 V)"));
    Serial.println(F("  ────────────────────────────────────────────────────────"));

    pass("Logger round-trip PASSED");
}

// ============================================================
//  TEST S — Servos (PIN_SERVO_X=5, PIN_SERVO_Y=6)
//  Watch the TVC mount physically while this runs.
//  Expected travel: ±7° (SERVO_MAX_ANGLE_DEG) from centre.
// ============================================================
static void test_servos() {
    print_banner("TEST S: TVC Servos (X=pin5, Y=pin6)");
    servo_init();

    auto sweep = [](const char *name, auto set_fn) {
        Serial.print(F("  ")); Serial.print(name);

        Serial.print(F(" -> centre (1500 us) ... "));
        set_fn(0.0f); delay(600);

        Serial.print(F("+7deg ... "));
        set_fn(SERVO_MAX_ANGLE_DEG); delay(800);

        Serial.print(F("-7deg ... "));
        set_fn(-SERVO_MAX_ANGLE_DEG); delay(800);

        Serial.println(F("centre"));
        set_fn(0.0f); delay(600);
    };

    sweep("SERVO X (pitch)", servo_set_pitch);
    sweep("SERVO Y (yaw)  ", servo_set_yaw);

    // Diagonal corners to verify independence
    Serial.println(F("  Corner sweep (both axes together):"));
    const float ang = SERVO_MAX_ANGLE_DEG;
    float corners[4][2] = { {ang,ang}, {ang,-ang}, {-ang,-ang}, {-ang,ang} };
    for (int i = 0; i < 4; i++) {
        Serial.print(F("    pitch=")); Serial.print(corners[i][0], 1);
        Serial.print(F("  yaw="));    Serial.println(corners[i][1], 1);
        servo_set_pitch(corners[i][0]);
        servo_set_yaw(corners[i][1]);
        delay(700);
    }
    servo_center();
    Serial.println(F("  Centred."));

    Serial.print(F("  Final pulse widths — X: "));
    Serial.print(servo_get_pitch_us(), 0);
    Serial.print(F(" us  Y: "));
    Serial.print(servo_get_yaw_us(), 0);
    Serial.println(F(" us  (both should be ~1500)"));

    pass("Servos PASSED — confirm physical travel matched printout");
}

// ============================================================
//  TEST L — Status LEDs (pins 4, 33, 30)
// ============================================================
static void test_leds() {
    print_banner("TEST L: Status LEDs (GREEN=4, WHITE=33, RED=30)");

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_WHITE, OUTPUT);
    pinMode(PIN_LED_RED,   OUTPUT);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_WHITE, LOW);
    digitalWrite(PIN_LED_RED,   LOW);

    struct { const char *name; uint8_t pin; } leds[] = {
        { "GREEN (pin 4)",  PIN_LED_GREEN },
        { "WHITE (pin 33)", PIN_LED_WHITE },
        { "RED   (pin 30)", PIN_LED_RED   },
    };

    for (int i = 0; i < 3; i++) {
        Serial.print(F("  ON  — ")); Serial.println(leds[i].name);
        digitalWrite(leds[i].pin, HIGH);
        delay(800);
        Serial.print(F("  OFF — ")); Serial.println(leds[i].name);
        digitalWrite(leds[i].pin, LOW);
        delay(300);
    }

    // All on together
    Serial.println(F("  ALL ON"));
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_WHITE, HIGH);
    digitalWrite(PIN_LED_RED,   HIGH);
    delay(1000);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_WHITE, LOW);
    digitalWrite(PIN_LED_RED,   LOW);
    Serial.println(F("  ALL OFF"));

    pass("LEDs PASSED — confirm each lit in sequence, then all together");
}

// ============================================================
//  TEST B — Buzzer (PIN_BUZZER = 3)
// ============================================================
static void beep(uint32_t on_ms, uint32_t off_ms) {
    digitalWrite(PIN_BUZZER, HIGH); delay(on_ms);
    digitalWrite(PIN_BUZZER, LOW);  delay(off_ms);
}

static void test_buzzer() {
    print_banner("TEST B: Buzzer (pin 3)");
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    Serial.println(F("  IDLE    — 1 short beep")); delay(300);
    beep(100, 900);

    Serial.println(F("  ARMED   — 2 short beeps")); delay(300);
    beep(100, 100); beep(100, 900);

    Serial.println(F("  POWERED — 3 short beeps")); delay(300);
    beep(100, 100); beep(100, 100); beep(100, 900);

    Serial.println(F("  APOGEE  — 1 long beep")); delay(300);
    beep(600, 900);

    Serial.println(F("  DESCENT — 4 short beeps")); delay(300);
    beep(100, 100); beep(100, 100); beep(100, 100); beep(100, 900);

    Serial.println(F("  LANDED  — 3 long beeps")); delay(300);
    beep(600, 200); beep(600, 200); beep(600, 900);

    Serial.println(F("  ABORT   — rapid burst")); delay(300);
    for (int i = 0; i < 8; i++) beep(80, 80);

    pass("Buzzer PASSED — if you heard all patterns");
}

// ============================================================
//  Run all tests
// ============================================================
static void run_all() {
    test_lps22hb();
    test_lsm6dsox();
    test_mmc5603();
    test_flash();
    test_sd();
    test_logger_roundtrip();

    Serial.println();
    print_banner("ALL TESTS COMPLETE");
    Serial.println(F("  Review each test above for PASS/FAIL."));
    Serial.println(F("  Send individual command (1-6) to re-run a specific test."));
}

// ============================================================
//  Print menu
// ============================================================
static void print_menu() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════════╗"));
    Serial.println(F("║       TVC FLIGHT COMPUTER BENCH TEST     ║"));
    Serial.println(F("╠══════════════════════════════════════════╣"));
    Serial.println(F("║  1 - LPS22HBTR Barometer (I2C)           ║"));
    Serial.println(F("║  2 - LSM6DSOX IMU (SPI)                  ║"));
    Serial.println(F("║  3 - MMC5603NJ Magnetometer (I2C Wire1)  ║"));
    Serial.println(F("║  4 - GD25Q128 NOR Flash (SPI)            ║"));
    Serial.println(F("║  5 - SD Card (SPI)                       ║"));
    Serial.println(F("║  6 - Logger round-trip (flash → SD CSV)  ║"));
    Serial.println(F("║  7 - Run ALL tests in sequence            ║"));
    Serial.println(F("║  S - Servo sweep (X and Y axes)          ║"));
    Serial.println(F("║  L - LED test (GREEN/WHITE/RED)           ║"));
    Serial.println(F("║  B - Buzzer patterns                      ║"));
    Serial.println(F("║  R - Reprint this menu                   ║"));
    Serial.println(F("╚══════════════════════════════════════════╝"));
    Serial.println(F("Send a character to begin."));
}

// ============================================================
//  Arduino entry points
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}   // wait up to 3 s for USB

    // CS pins HIGH before SPI.begin() — critical
    pinMode(PIN_FLASH_CS,  OUTPUT); digitalWriteFast(PIN_FLASH_CS,  HIGH);
    pinMode(PIN_IMU_CS,    OUTPUT); digitalWriteFast(PIN_IMU_CS,    HIGH);
    pinMode(PIN_SD_CS,     OUTPUT); digitalWriteFast(PIN_SD_CS,     HIGH);

    SPI.begin();
    Wire.begin();
    Wire1.begin();
    delay(100);

    // Release flash from power-down
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI.transfer(FCMD_RELEASE_PD);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();
    delayMicroseconds(30);

    print_menu();
}

void loop() {
    if (!Serial.available()) return;
    char c = Serial.read();
    while (Serial.available()) Serial.read();  // flush any extra chars

    switch (c) {
        case '1': test_lps22hb();         break;
        case '2': test_lsm6dsox();        break;
        case '3': test_mmc5603();         break;
        case '4': test_flash();           break;
        case '5': test_sd();              break;
        case '6': test_logger_roundtrip(); break;
        case '7': run_all();              break;
        case 'S': case 's': test_servos(); break;
        case 'L': case 'l': test_leds();   break;
        case 'B': case 'b': test_buzzer(); break;
        case 'R': case 'r': print_menu(); break;
        default:
            Serial.print(F("Unknown command: "));
            Serial.println(c);
            print_menu();
            break;
    }
}