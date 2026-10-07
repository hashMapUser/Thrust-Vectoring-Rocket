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
//    1 — Test LPS22HBTR barometer  (I2C)
//    2 — Test LSM6DSOX IMU         (SPI)
//    3 — Test MMC5603NJ magnetometer (I2C Wire1)
//    4 — Test GD25Q128 NOR flash   (SPI)
//    5 — Test SD card              (card detect → sensor data → flash/RAM → SD CSV)
//    6 — Full logger round-trip    (write fake flight → dump CSV → verify)
//    7 — Run ALL tests in sequence
//    H — Barometer lift-height test (averaged baseline, lift prompt, 2.5 s delay, averaged delta)
//    M — Magnetometer calibration (rotate ~50 s, then 'Y' to save to EEPROM)
//    C — Pyro continuity test (LEDs as visual aid, pyro battery must be live)
//    A — ARM_SENSE readout (plain vs pull-down, then live as flight until a key)
//    U — USB log dump (RAM buffer -> Serial CSV, no SD card needed)
//    R — Reset / reprint menu
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <EEPROM.h>
#include <math.h>

// Pull in your actual driver headers
#include "board_pins.h"
#include "lps22hb.h"
#include "alt_estimator.h"   // ALT_GROUND_SAMPLES — Test H matches the flight baseline
#include "lsm6dsox.h"
#include "mag.h"
#include "mag_calib.h"
#include "logger.h"
#include "servo_driver.h"
#include "buzzer.h"
#include "pyro.h"
#include "flash.h"

extern "C" const buzzer_hal_t BUZZER_HAL_TEENSY;

// SdFat, bound explicitly to &SPI1 — the SD card is wired to SPI1
// (pins 0/1/26/27). SD.begin(csPin) has no way to select a non-default
// bus: it always drives SPI0 (11/12/13, the IMU's bus) regardless of CS
// pin. See src/logger.cpp for the same fix in the flight logger.
#define BENCH_SD_CLOCK_MHZ 16
#define BENCH_SD_CONFIG SdSpiConfig(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(BENCH_SD_CLOCK_MHZ), &SPI1)
static SdFs _bench_sd;

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
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_READ_STATUS1);
    uint32_t t0 = millis();
    while (SPI2.transfer(0x00) & FSTATUS_WIP) {
        if (millis() - t0 > 5000) { Serial.println("  [WARN] Flash WIP timeout"); break; }
    }
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();
}

static void flash_write_enable_bt() {
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_WRITE_ENABLE);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();
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
        info("No mag calibration in EEPROM — send 'M' to calibrate");
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
//  TEST M — Magnetometer hard/soft-iron calibration
//  Runs mag_calibrate() (~50 s of rotation), shows the result, and
//  saves it to EEPROM only on a 'Y' — a bad run never overwrites a
//  good calibration.
// ============================================================
#define MAG_CAL_CONFIRM_TIMEOUT_MS 30000

static void test_mag_calibrate() {
    print_banner("TEST M: Magnetometer Calibration (MMC5603NJ)");

    if (!mag_init()) {
        fail("mag_init() returned false — sensor not responding");
        return;
    }

    Serial.println(F("  Fit the board in the airframe as it will fly (battery, motor"));
    Serial.println(F("  case, servos) — their iron is what this corrects for. Stand"));
    Serial.println(F("  clear of desks, laptops and steel, then rotate it slowly through"));
    Serial.println(F("  every orientation: a full roll about each axis, nose up and down."));

    MagCalib cal;
    if (!mag_calibrate(&cal)) {
        fail("Calibration failed — nothing saved. Cover more orientations and retry.");
        return;
    }

    Serial.println(F("  Send Y to save to EEPROM, anything else to discard."));
    while (Serial.available()) Serial.read();   // drop anything typed during the rotation
    int c = -1;
    uint32_t t0 = millis();
    while (millis() - t0 < MAG_CAL_CONFIRM_TIMEOUT_MS) {
        if (!Serial.available()) continue;
        c = Serial.read();
        if (c != '\r' && c != '\n') break;
        c = -1;
    }
    if (c != 'Y' && c != 'y') {
        info("Not saved — EEPROM calibration unchanged");
        return;
    }

    mag_save_calib(&cal);
    MagCalib readback;
    if (mag_load_calib(&readback) && memcmp(&readback, &cal, sizeof cal) == 0) {
        pass("Mag calibration saved and read back — run Test 3 to check it");
    } else {
        fail("EEPROM read-back mismatch — calibration not stored correctly");
    }
}

// ============================================================
//  TEST 4 — GD25Q128 NOR Flash
// ============================================================
static void test_flash() {
    print_banner("TEST 4: GD25Q128 NOR Flash (SPI)");

    // --- JEDEC ID ---
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_JEDEC_ID);
    uint8_t mfr = SPI2.transfer(0);
    uint8_t mem = SPI2.transfer(0);
    uint8_t cap = SPI2.transfer(0);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();

    Serial.print(F("  JEDEC ID: 0x"));
    Serial.print(mfr, HEX); Serial.print(F(" 0x"));
    Serial.print(mem, HEX); Serial.print(F(" 0x"));
    Serial.println(cap, HEX);
    Serial.println(F("  Expected: 0xC8 0x40 0x18 (GigaDevice GD25Q128)"));

    if (mfr != 0xC8 || mem != 0x40 || cap != 0x18) {
        fail("JEDEC ID mismatch — chip not responding or wrong part");
        Serial.println(F("  Checklist:"));
        Serial.println(F("    - PIN_FLASH_CS = 36 correct?"));
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
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_SECTOR_ERASE);
    SPI2.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  0) & 0xFF);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();
    flash_wait_ready_bt();
    Serial.println(F("done"));

    // Verify erased (all 0xFF)
    uint8_t read_buf[32];
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_READ_DATA);
    SPI2.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  0) & 0xFF);
    for (int i = 0; i < 32; i++) read_buf[i] = SPI2.transfer(0);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();

    bool erased_ok = true;
    for (int i = 0; i < 32; i++) if (read_buf[i] != 0xFF) { erased_ok = false; break; }
    if (!erased_ok) { fail("Post-erase verify failed — expected all 0xFF"); return; }
    pass("Sector erase verified (all 0xFF)");

    // Write 32 bytes of test pattern
    uint8_t write_buf[32];
    for (int i = 0; i < 32; i++) write_buf[i] = (uint8_t)(i * 7 + 0xA0);  // deterministic pattern

    flash_write_enable_bt();
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_PAGE_PROGRAM);
    SPI2.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  0) & 0xFF);
    for (int i = 0; i < 32; i++) SPI2.transfer(write_buf[i]);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();
    flash_wait_ready_bt();

    // Read back and verify
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_READ_DATA);
    SPI2.transfer((TEST_ADDR >> 16) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  8) & 0xFF);
    SPI2.transfer((TEST_ADDR >>  0) & 0xFF);
    for (int i = 0; i < 32; i++) read_buf[i] = SPI2.transfer(0);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();

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
//  TEST 5 — SD Card (card detect → capture → stage → SD CSV)
//  1. Card detect on PIN_SD_CD — stops here if no card is seated.
//  2. Mount the card on SPI1.
//  3. Capture SD_BENCH_RECORDS LogRecords at 125 Hz. Each sensor that
//     initialises gives real readings; any that doesn't is filled with
//     synthetic data. The imu/baro/mag_valid CSV columns say which:
//     1 = real reading, 0 = synthetic.
//  4. Stage the records in the GD25Q128 flash if it passes a write/
//     readback probe, otherwise in a Teensy RAM buffer.
//  5. Transfer staging → BENCH_SD.CSV on the card, then re-open the
//     file and verify the row count.
// ============================================================
#define SD_CD_PRESENT_LEVEL  LOW    // switch closes to GND with a card in; R701 pulls up
#define SD_CD_DEBOUNCE_MS    50
#define SD_BENCH_RECORDS     250    // 2 s at 125 Hz
#define SD_BENCH_PERIOD_US   8000
#define SD_BENCH_FLASH_ADDR  0x100000u   // 1 MB in — clear of TEST 4's sector at 0x010000
#define SD_BENCH_FILE        "BENCH_SD.CSV"

static LogRecord _sd_ram_stage[SD_BENCH_RECORDS];   // fallback staging when flash is down

// Flash staging: records packed back-to-back as raw LogRecord bytes,
// streamed through a one-page buffer.
static uint8_t  _sd_page[FLASH_PAGE_SIZE];
static uint16_t _sd_page_fill;
static uint32_t _sd_flash_addr;

// FNV-1a over every staged byte — compared on write vs. readback so a
// flaky flash shows up as a checksum mismatch, not silently bad CSV.
static uint32_t fnv1a(uint32_t h, const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

static bool sd_card_detected() {
    pinMode(PIN_SD_CD, INPUT);   // R701 is the pull-up — plain INPUT
    int level = digitalRead(PIN_SD_CD);
    uint32_t t0 = millis(), start = t0;
    // Level must hold for SD_CD_DEBOUNCE_MS; give up after 1 s of bounce
    while (millis() - t0 < SD_CD_DEBOUNCE_MS && millis() - start < 1000) {
        int now = digitalRead(PIN_SD_CD);
        if (now != level) { level = now; t0 = millis(); }
    }
    Serial.print(F("  SD_CD (pin ")); Serial.print(PIN_SD_CD);
    Serial.print(F(") = "));          Serial.println(level == HIGH ? F("HIGH") : F("LOW"));
    return level == SD_CD_PRESENT_LEVEL;
}

// JEDEC ID + erase/program/readback of 16 bytes in the staging sector.
// A chip that IDs correctly but can't hold data still counts as down.
static bool sd_flash_usable() {
    if (!flash_init()) return false;
    uint8_t w[16], r[16];
    for (int i = 0; i < 16; i++) w[i] = (uint8_t)(0x5A ^ (i * 17));
    flash_erase_sector(SD_BENCH_FLASH_ADDR);
    flash_page_program(SD_BENCH_FLASH_ADDR, w, sizeof(w));
    flash_read(SD_BENCH_FLASH_ADDR, r, sizeof(r));
    return memcmp(w, r, sizeof(w)) == 0;
}

static void sd_flash_stage_begin() {
    const uint32_t bytes = SD_BENCH_RECORDS * sizeof(LogRecord);
    for (uint32_t a = SD_BENCH_FLASH_ADDR; a < SD_BENCH_FLASH_ADDR + bytes; a += FLASH_SECTOR_SIZE)
        flash_erase_sector(a);
    _sd_page_fill  = 0;
    _sd_flash_addr = SD_BENCH_FLASH_ADDR;
}

static void sd_flash_stage_write(const LogRecord *r) {
    const uint8_t *p = (const uint8_t *)r;
    for (size_t k = 0; k < sizeof(LogRecord); k++) {
        _sd_page[_sd_page_fill++] = p[k];
        if (_sd_page_fill == FLASH_PAGE_SIZE) {
            flash_page_program(_sd_flash_addr, _sd_page, FLASH_PAGE_SIZE);
            _sd_flash_addr += FLASH_PAGE_SIZE;
            _sd_page_fill = 0;
        }
    }
}

static void sd_flash_stage_end() {
    if (_sd_page_fill) flash_page_program(_sd_flash_addr, _sd_page, _sd_page_fill);
}

// One record: real data from each working sensor, synthetic otherwise.
// No attitude filter or controller runs on the bench, so attitude, servo
// and PID columns hold neutral values.
static void sd_bench_fill(LogRecord *r, uint16_t i, bool imu_ok, bool baro_ok,
                          bool mag_ok, float *p0_hpa) {
    memset(r, 0, sizeof(*r));
    r->timestamp_ms = millis();
    const float t = i * (SD_BENCH_PERIOD_US / 1e6f);

    LSM6DSOX_Data imu = {};
    if (imu_ok) lsm6dsox_read(&imu, nullptr);
    if (imu_ok && imu.valid) {
        r->gx = imu.gx; r->gy = imu.gy; r->gz = imu.gz;
        r->ax = imu.ax; r->ay = imu.ay; r->az = imu.az;
        r->imu_valid = true;
    } else {
        r->gx = 10.0f * sinf(TWO_PI * 0.5f * t);   // slow synthetic coning
        r->gy = 10.0f * cosf(TWO_PI * 0.5f * t);
        r->gz = 0.0f;
        r->ax = 0.0f; r->ay = 0.0f; r->az = 1.0f;
    }

    LPS22HB_Data baro = {};
    if (baro_ok) lps22hb_read(&baro);
    if (baro_ok && baro.valid) {
        r->temperature_c = baro.temperature_c;
        r->pressure_hpa  = baro.pressure_pa / 100.0f;
        r->baro_valid    = true;
    } else {
        r->temperature_c = 22.5f;
        r->pressure_hpa  = 1013.25f - 0.5f * t;    // ~4 m/s synthetic climb
    }
    if (i == 0) *p0_hpa = r->pressure_hpa;
    r->altitude_m = 44330.0f * (1.0f - powf(r->pressure_hpa / *p0_hpa, 0.1902949f));

    mag_data m = {};
    if (mag_ok) mag_read(&m);
    if (mag_ok && m.valid) {
        r->mx = m.mag_x; r->my = m.mag_y; r->mz = m.mag_z;
        r->mag_valid = true;
    } else {
        r->mx = 0.2f; r->my = 0.1f; r->mz = -0.4f;
    }

    r->q0 = 1.0f;
    r->servo_pitch_us = 1500.0f;
    r->servo_yaw_us   = 1500.0f;
    r->flight_state   = STATE_IDLE;
}

static void test_sd() {
    char sd_banner[64];
    snprintf(sd_banner, sizeof(sd_banner), "TEST 5: SD Card (SPI1, CS pin %d, CD pin %d)",
             PIN_SD_CS, PIN_SD_CD);
    print_banner(sd_banner);

    // --- 1. Card detect ---
    if (!sd_card_detected()) {
        fail("No card detected on SD_CD — insert a card and re-run '5'");
        Serial.println(F("  If a card IS inserted: check the DM3AT detect switch and R701."));
        return;
    }
    pass("Card detected");

    // --- 2. Mount ---
    SPI1.setMISO(PIN_SD_MISO);
    SPI1.setMOSI(PIN_SD_MOSI);
    SPI1.setSCK(PIN_SD_SCK);

    if (!_bench_sd.begin(BENCH_SD_CONFIG)) {
        fail("SD init failed");
        _bench_sd.initErrorPrint(&Serial);
        Serial.println(F("  Checklist:"));
        Serial.println(F("    - PIN_SD_CS = 0 correct?"));
        Serial.println(F("    - Card formatted FAT32/exFAT?"));
        Serial.println(F("    - Try a different SD card (some cards fail at 3.3V)"));
        return;
    }
    pass("SD init OK — card mounted");

    // --- 3. Sensors: real where they work, synthetic where they don't ---
    info("Bringing up sensors...");
    const bool imu_ok  = lsm6dsox_init();
    const bool baro_ok = lps22hb_init();
    const bool mag_ok  = mag_init();
    Serial.print(F("  IMU : ")); Serial.println(imu_ok  ? F("real")  : F("SYNTHETIC (init failed)"));
    Serial.print(F("  BARO: ")); Serial.println(baro_ok ? F("real")  : F("SYNTHETIC (init failed)"));
    Serial.print(F("  MAG : ")); Serial.println(mag_ok  ? F("real")  : F("SYNTHETIC (init failed)"));

    // --- 4. Staging: flash if it works, RAM if not ---
    const bool use_flash = sd_flash_usable();
    if (use_flash) {
        info("Flash write/readback OK — staging records in GD25Q128");
        sd_flash_stage_begin();
    } else {
        info("Flash down — staging records in Teensy RAM buffer");
    }

    Serial.print(F("  Capturing ")); Serial.print(SD_BENCH_RECORDS);
    Serial.println(F(" records at 125 Hz..."));

    float p0_hpa = 1013.25f;
    uint32_t sum_wr = 2166136261u;
    uint32_t next_us = micros();
    for (uint16_t i = 0; i < SD_BENCH_RECORDS; i++) {
        while ((int32_t)(micros() - next_us) < 0) {}
        next_us += SD_BENCH_PERIOD_US;

        LogRecord r;
        sd_bench_fill(&r, i, imu_ok, baro_ok, mag_ok, &p0_hpa);
        sum_wr = fnv1a(sum_wr, (const uint8_t *)&r, sizeof(r));
        if (use_flash) sd_flash_stage_write(&r);
        else           _sd_ram_stage[i] = r;
    }
    if (use_flash) sd_flash_stage_end();
    pass(use_flash ? "Records staged in flash" : "Records staged in RAM");

    // --- 5. Transfer staging → SD ---
    if (_bench_sd.exists(SD_BENCH_FILE)) _bench_sd.remove(SD_BENCH_FILE);
    FsFile f = _bench_sd.open(SD_BENCH_FILE, O_WRONLY | O_CREAT | O_TRUNC);
    if (!f) {
        fail("Could not create " SD_BENCH_FILE);
        return;
    }
    logger_print_csv_header(f);

    uint32_t sum_rd = 2166136261u;
    for (uint16_t i = 0; i < SD_BENCH_RECORDS; i++) {
        LogRecord r;
        if (use_flash) flash_read(SD_BENCH_FLASH_ADDR + (uint32_t)i * sizeof(LogRecord),
                                  (uint8_t *)&r, sizeof(r));
        else           r = _sd_ram_stage[i];
        sum_rd = fnv1a(sum_rd, (const uint8_t *)&r, sizeof(r));
        logger_print_csv_row(f, &r);
    }
    f.sync();
    const uint32_t file_bytes = (uint32_t)f.fileSize();
    f.close();

    if (sum_rd != sum_wr) {
        fail("Staging checksum mismatch — data read back from staging is corrupt");
        Serial.print(F("  wrote 0x")); Serial.print(sum_wr, HEX);
        Serial.print(F("  read 0x"));  Serial.println(sum_rd, HEX);
        return;
    }
    pass("Staging readback checksum matches");

    Serial.print(F("  Wrote ")); Serial.print(SD_BENCH_FILE);
    Serial.print(F(" (")); Serial.print(file_bytes); Serial.println(F(" bytes)"));

    // --- Verify: re-open, count rows, echo the first few ---
    f = _bench_sd.open(SD_BENCH_FILE, O_RDONLY);
    if (!f) {
        fail("Could not re-open " SD_BENCH_FILE " for reading");
        return;
    }
    Serial.println(F("  First rows of the file:"));
    uint32_t lines = 0;
    char line[320];
    while (f.available()) {
        int n = f.fgets(line, sizeof(line));
        if (n <= 0) break;
        if (lines < 4) { Serial.print(F("    ")); Serial.print(line); }
        lines++;
    }
    f.close();

    Serial.print(F("  Rows (excluding header): ")); Serial.print(lines ? lines - 1 : 0);
    Serial.print(F(" / expected ")); Serial.println(SD_BENCH_RECORDS);
    if (lines != SD_BENCH_RECORDS + 1u) {
        fail("Row count mismatch — SD write incomplete");
        return;
    }
    pass("SD Card PASSED — detect, capture, stage, write, and readback all OK");
    Serial.println(F("  " SD_BENCH_FILE " is left on the card for inspection."));
}

// ============================================================
//  TEST 6 — Full logger round-trip
//  Writes 50 fake LogRecords via logger_write() (RAM ring buffer only),
//  then logger_finalize() dumps them straight to FLIGHT_XXX.CSV on the
//  SD card. GD25Q128 flash is not used for flight logging — it's wired
//  incorrectly on this board (see FC_V2_1_PIN_REFERENCE.md).
// ============================================================
static void test_logger_roundtrip() {
    print_banner("TEST 6: Logger Round-Trip (RAM buffer → SD CSV)");

    info("Reinitialising logger (SD init on SPI1 + next FLIGHT_XXX.CSV slot)...");

    bool ok = logger_init();
    if (!ok) {
        Serial.println(F("  [WARN] logger_init() returned false — check SD card"));
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

    Serial.println(F("  Finalising (SD if ready, USB serial fallback if not)..."));
    logger_finalize();

    if (ok) {
        pass("CSV written to SD");
        Serial.println();
        Serial.println(F("  ── SD RETRIEVAL GUIDE ─────────────────────────────────"));
        Serial.println(F("  After a real flight:"));
        Serial.println(F("    1. Eject the SD card (or send 'R' to force a dump early)"));
        Serial.println(F("    2. Read FLIGHT_XXX.CSV directly on a PC — no host script"));
        Serial.println(F("       needed, it's already plain CSV"));
        Serial.println(F("    3. FLIGHT_XXX.LOG holds the state-transition checkpoints"));
        Serial.println(F("  ────────────────────────────────────────────────────────"));
    } else {
        pass("No SD card — fell back to USB serial dump above (check for the "
             "BEGIN/END FLIGHT CSV markers)");
    }

    pass("Logger round-trip PASSED");
}

// ============================================================
//  TEST U — USB Log Dump
//  Streams whatever is currently in the RAM ring buffer as CSV over
//  serial, on demand — the retrieval path when there's no SD card.
//  Run '6' first if the buffer is empty; this just dumps, it doesn't
//  write any records itself.
// ============================================================
static void test_usb_dump() {
    print_banner("TEST U: USB Log Dump (RAM buffer -> Serial CSV)");
    Serial.print(F("  Records currently buffered: "));
    Serial.println(logger_record_count());
    logger_usb_dump();
    pass("USB dump complete — check for the BEGIN/END FLIGHT CSV markers above");
}

// ============================================================
//  TEST S — Servos (PIN_SERVO_X=5, PIN_SERVO_Y=6)
//  Watch the TVC mount physically while this runs.
//  Expected travel: ±72° pitch, ±36° yaw (SERVO_*_MAX_ANGLE_DEG) from centre.
// ============================================================
static void test_servos() {
    print_banner("TEST S: TVC Servos (X=pin5, Y=pin6)");
    servo_init();

    auto sweep = [](const char *name, auto set_fn, float max_angle_deg, float centre_us) {
        Serial.print(F("  ")); Serial.print(name);

        Serial.print(F(" -> centre (")); Serial.print(centre_us, 0); Serial.print(F(" us) ... "));
        set_fn(0.0f); delay(600);

        Serial.print(max_angle_deg, 1); Serial.print(F("deg ... "));
        set_fn(max_angle_deg); delay(800);

        Serial.print(-max_angle_deg, 1); Serial.print(F("deg ... "));
        set_fn(-max_angle_deg); delay(800);

        Serial.println(F("centre"));
        set_fn(0.0f); delay(600);
    };

    sweep("SERVO X (pitch)", servo_set_pitch, SERVO_PITCH_MAX_ANGLE_DEG, SERVO_PITCH_CENTER_US);
    sweep("SERVO Y (yaw)  ", servo_set_yaw,   SERVO_YAW_MAX_ANGLE_DEG,   SERVO_YAW_CENTER_US);

    // Diagonal corners to verify independence
    Serial.println(F("  Corner sweep (both axes together):"));
    const float pitch_ang = SERVO_PITCH_MAX_ANGLE_DEG;
    const float yaw_ang   = SERVO_YAW_MAX_ANGLE_DEG;
    float corners[4][2] = { {pitch_ang,yaw_ang}, {pitch_ang,-yaw_ang}, {-pitch_ang,-yaw_ang}, {-pitch_ang,yaw_ang} };
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
    Serial.print(F(" us  (X should be ~"));
    Serial.print(SERVO_PITCH_CENTER_US);
    Serial.print(F(", Y should be ~"));
    Serial.print(SERVO_YAW_CENTER_US);
    Serial.println(F(")"));

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
//  TEST C — Pyro Continuity (LEDs as visual aid)
//  Requires the pyro battery connected (SW401 closed) — continuity
//  can't be measured through a dead divider. Not run by '7' since it
//  needs pyro power live and physical eyes on the LEDs.
// ============================================================

// ARM_SENSE divider: 10K/4.7K, ratio 0.3197. Same math as
// main_control_loop.cpp's read_pack_voltage() — duplicated here so this
// harness compiles standalone, matching this file's existing pattern for
// the flash command set.
#define BENCH_ARM_SENSE_DIVIDER_RATIO 0.3197f

static void test_pyro_continuity() {
    print_banner("TEST C: Pyro Continuity (GREEN=OK, RED=OPEN, WHITE=testing)");

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_WHITE, OUTPUT);
    pinMode(PIN_LED_RED,   OUTPUT);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_WHITE, LOW);
    digitalWrite(PIN_LED_RED,   LOW);

    analogReadResolution(12);
    pinMode(PIN_ARM_SENSE,   INPUT_PULLDOWN);   // as flight — an open pin reads 0 V, not junk
    pinMode(PIN_PYRO1_SENSE, INPUT);
    pinMode(PIN_PYRO2_SENSE, INPUT);

    float arm_sense_v = analogRead(PIN_ARM_SENSE) * 3.30f / 4095.0f;
    float pack_v       = arm_sense_v / BENCH_ARM_SENSE_DIVIDER_RATIO;
    Serial.print(F("  Pyro pack voltage (via ARM_SENSE): "));
    Serial.print(pack_v, 2);
    Serial.println(F(" V"));

    if (pack_v < 3.0f) {
        Serial.println(F("  [WARN] Pack reads too low for a live check —"));
        Serial.println(F("         connect the pyro battery (SW401 closed) and retry."));
    }

    struct { const char *name; uint8_t sense_pin; } channels[] = {
        { "PYRO1 (MAIN, used this flight)", PIN_PYRO1_SENSE },
        { "PYRO2 (unused this flight)",     PIN_PYRO2_SENSE },
    };

    bool main_ok = false;
    bool all_ok  = true;

    for (uint8_t i = 0; i < 2; i++) {
        digitalWrite(PIN_LED_WHITE, HIGH);   // WHITE = this channel under test
        bool ok = pyro_check_continuity(channels[i].sense_pin, pack_v);
        if (i == 0) main_ok = ok;
        all_ok = all_ok && ok;

        Serial.print(F("  ")); Serial.print(channels[i].name);
        Serial.print(F(": ")); Serial.println(ok ? F("CONTINUITY OK") : F("OPEN / NO MATCH"));

        digitalWrite(PIN_LED_GREEN, ok ? HIGH : LOW);
        digitalWrite(PIN_LED_RED,   ok ? LOW  : HIGH);
        delay(1500);
        digitalWrite(PIN_LED_WHITE, LOW);
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED,   LOW);
        delay(300);
    }

    // Final hold: light the result for the channel actually used this
    // flight, so there's a lasting visual readout at the pad.
    digitalWrite(PIN_LED_GREEN, main_ok ? HIGH : LOW);
    digitalWrite(PIN_LED_RED,   main_ok ? LOW  : HIGH);
    Serial.println(F("  GREEN = main chute continuity OK, RED = OPEN. Send any key to clear."));

    if (all_ok) {
        pass("Pyro Continuity PASSED — both channels show continuity");
    } else if (main_ok) {
        fail("Pyro Continuity — PYRO2 open (unused this flight, but check wiring)");
    } else {
        fail("Pyro Continuity — MAIN chute e-match reads OPEN. Do not arm.");
    }
}

// ============================================================
//  TEST A — ARM_SENSE readout (pin 24 / A10)
//  Shows what the flight firmware sees on ARM_SENSE. Samples the pin as
//  plain INPUT, then with the internal pull-down on — the way
//  main_control_loop.cpp sets it up. A plain-INPUT reading well above the
//  pull-down one means nothing is driving the pin: it floats, and only
//  the pull-down keeps it from reading as SW401 closed (buzzer on). Ends
//  with a live stream, with the firmware's debounced verdict, so the
//  switch can be flipped while watching.
// ============================================================

// Same threshold and debounce as main_control_loop.cpp.
#define BENCH_ARM_SENSE_ARMED_V   1.2f
#define BENCH_ARM_DEBOUNCE_MS     100
#define BENCH_ARM_LOOP_MS         8      // flight loop period (125 Hz)
#define ARM_SAMPLE_COUNT          10
#define ARM_SAMPLE_INTERVAL_MS    200
#define ARM_STREAM_INTERVAL_MS    250
#define ARM_FLOAT_MARGIN_V        0.2f   // plain-vs-pull-down gap that means floating

struct ArmSampleStats { float mean_v; uint8_t above; };

static float arm_counts_to_v(uint16_t raw) { return raw * 3.30f / 4095.0f; }

static void print_arm_reading(uint16_t raw) {
    float v = arm_counts_to_v(raw);
    Serial.print(v, 2);
    Serial.print(F(" V  (raw "));
    Serial.print(raw);
    Serial.print(F(", pack "));
    Serial.print(v / BENCH_ARM_SENSE_DIVIDER_RATIO, 2);
    Serial.print(F(" V)  "));
    Serial.print(v > BENCH_ARM_SENSE_ARMED_V ? F("ABOVE threshold") : F("below threshold"));
}

static ArmSampleStats sample_arm_sense(uint8_t mode, const char *label) {
    pinMode(PIN_ARM_SENSE, mode);
    delay(20);   // let the pull settle the pin before the first read
    Serial.print(F("  ")); Serial.println(label);

    uint16_t lo = 4095, hi = 0;
    uint32_t sum = 0;
    ArmSampleStats s = { 0.0f, 0 };
    for (uint8_t i = 0; i < ARM_SAMPLE_COUNT; i++) {
        uint16_t raw = analogRead(PIN_ARM_SENSE);
        if (raw < lo) lo = raw;
        if (raw > hi) hi = raw;
        sum += raw;
        if (arm_counts_to_v(raw) > BENCH_ARM_SENSE_ARMED_V) s.above++;
        Serial.print(F("    ")); print_arm_reading(raw); Serial.println();
        delay(ARM_SAMPLE_INTERVAL_MS);
    }
    s.mean_v = arm_counts_to_v(sum / ARM_SAMPLE_COUNT);

    Serial.print(F("    min "));   Serial.print(arm_counts_to_v(lo), 2);
    Serial.print(F(" V, max "));   Serial.print(arm_counts_to_v(hi), 2);
    Serial.print(F(" V, mean "));  Serial.print(s.mean_v, 2);
    Serial.print(F(" V — "));      Serial.print(s.above);
    Serial.print(F(" of "));       Serial.print(ARM_SAMPLE_COUNT);
    Serial.println(F(" above threshold"));
    return s;
}

static void test_arm_sense() {
    print_banner("TEST A: ARM_SENSE readout (pin 24 / A10)");
    Serial.print(F("  Flight threshold: "));
    Serial.print(BENCH_ARM_SENSE_ARMED_V, 2);
    Serial.print(F(" V (pack "));
    Serial.print(BENCH_ARM_SENSE_ARMED_V / BENCH_ARM_SENSE_DIVIDER_RATIO, 2);
    Serial.print(F(" V), debounce "));
    Serial.print(BENCH_ARM_DEBOUNCE_MS);
    Serial.println(F(" ms. Above it = SW401 CLOSED = buzzer on."));

    analogReadResolution(12);
    ArmSampleStats plain = sample_arm_sense(INPUT,
        "Plain INPUT, no pull — is anything driving the pin?");
    ArmSampleStats pd    = sample_arm_sense(INPUT_PULLDOWN,
        "INPUT_PULLDOWN (~100k to GND) — how the flight firmware reads it:");

    bool floating = pd.mean_v < ARM_FLOAT_MARGIN_V &&
                    plain.mean_v - pd.mean_v > ARM_FLOAT_MARGIN_V;
    if (floating) {
        info("Nothing is driving ARM_SENSE (it floats without the pull-down) — SW401/divider not connected?");
    }
    if (pd.above) {
        info("Flight firmware reads SW401 CLOSED — buzzer on (right only if the switch is closed)");
    } else {
        pass("Flight firmware reads SW401 OPEN — buzzer silent");
    }

    // Live readout in the flight configuration (pin left as
    // INPUT_PULLDOWN from the last pass), run through the same debounce
    // as main_control_loop.cpp, so the line says what the flight
    // firmware would decide — flip SW401 and watch it change.
    Serial.println(F("  Live (INPUT_PULLDOWN, as flight) — send any key to stop:"));
    bool     raw_prev   = false;
    bool     armed      = false;
    uint32_t edge_ms    = 0;
    uint32_t last_print = 0;
    while (true) {
        if (Serial.available()) {
            int c = Serial.read();
            if (c != '\r' && c != '\n') break;   // ignore the line ending from the 'A' command
        }
        uint32_t now = millis();
        uint16_t raw = analogRead(PIN_ARM_SENSE);
        bool above = arm_counts_to_v(raw) > BENCH_ARM_SENSE_ARMED_V;
        if (above != raw_prev) { edge_ms = now; raw_prev = above; }
        if (now - edge_ms >= BENCH_ARM_DEBOUNCE_MS) armed = above;

        if (now - last_print >= ARM_STREAM_INTERVAL_MS) {
            last_print = now;
            Serial.print(F("    ")); print_arm_reading(raw);
            Serial.println(armed ? F("  -> firmware: ARMED, buzzer ON")
                                 : F("  -> firmware: SAFE, buzzer off"));
        }
        delay(BENCH_ARM_LOOP_MS);
    }
    info("Live readout stopped");
}

// ============================================================
//  TEST H — Barometer Lift-Height Test
//  Captures a baseline pressure, prompts you to physically lift the
//  barometer, then 2.5 s later averages a second reading while you hold
//  it up and reports the height change. Both readings average
//  ALT_GROUND_SAMPLES fresh samples with outliers dropped — exactly the
//  flight launch baseline's alt_ground_mean() — so a single wild sample
//  can't decide the result.
// ============================================================
#define BARO_LIFT_WAIT_MS            2500
#define BARO_SAMPLES_PER_LINE        5

// Every reading in the set, '*' marking the outliers alt_ground_mean()
// dropped, plus min/max/spread. Printed after the capture so Serial
// output can't delay the sampling.
static void print_baro_samples(const float *samples_hpa, uint16_t n, float median_hpa) {
    float lo = samples_hpa[0], hi = samples_hpa[0];
    for (uint16_t i = 0; i < n; i++) {
        if (i % BARO_SAMPLES_PER_LINE == 0) Serial.print(F("    "));
        Serial.print('[');
        if (i + 1 < 100) Serial.print(' ');
        if (i + 1 < 10)  Serial.print(' ');
        Serial.print(i + 1);
        Serial.print(F("] "));
        Serial.print(samples_hpa[i], 3);
        bool outlier = fabsf(samples_hpa[i] - median_hpa) > ALT_GROUND_OUTLIER_HPA;
        Serial.print(outlier ? F("* ") : F("  "));
        if (i % BARO_SAMPLES_PER_LINE == BARO_SAMPLES_PER_LINE - 1 || i == n - 1) Serial.println();

        if (samples_hpa[i] < lo) lo = samples_hpa[i];
        if (samples_hpa[i] > hi) hi = samples_hpa[i];
    }
    Serial.print(F("    min ")); Serial.print(lo, 3);
    Serial.print(F("  max "));   Serial.print(hi, 3);
    Serial.print(F("  spread ")); Serial.print(hi - lo, 3);
    Serial.print(F("  median ")); Serial.print(median_hpa, 3);
    Serial.println(F(" hPa"));
}

// Read one set of ALT_GROUND_SAMPLES, print it, and average it the same way
// the flight ground reference does. Prints its own FAIL on error.
static bool baro_lift_capture(const __FlashStringHelper *label, float *mean_hpa) {
    float samples_hpa[ALT_GROUND_SAMPLES];
    if (!lps22hb_read_samples(ALT_GROUND_SAMPLES, samples_hpa)) {
        fail("Sensor stopped producing data — check I2C / try again");
        return false;
    }

    float    median_hpa;
    uint16_t used = 0;
    bool ok = alt_ground_mean(samples_hpa, ALT_GROUND_SAMPLES, mean_hpa, &median_hpa, &used);

    Serial.print(F("  ")); Serial.print(label);
    Serial.println(F(" readings (hPa, * = outlier, dropped):"));
    print_baro_samples(samples_hpa, ALT_GROUND_SAMPLES, median_hpa);
    Serial.print(F("    used ")); Serial.print(used);
    Serial.print(F(" of "));      Serial.print(ALT_GROUND_SAMPLES);
    Serial.print(F(" ("));        Serial.print(ALT_GROUND_SAMPLES - used);
    Serial.println(F(" outliers dropped)"));

    if (!ok) {
        fail("Too many outliers — sensor glitching or board moving; try again");
        return false;
    }
    return true;
}

static void test_baro_lift_height() {
    print_banner("TEST H: Barometer Lift-Height (LPS22HBTR)");

    if (!lps22hb_init()) {
        fail("lps22hb_init() returned false — sensor not responding");
        return;
    }

    info("Capturing baseline height — keep the barometer still...");
    float baseline_hpa;
    if (!baro_lift_capture(F("Baseline"), &baseline_hpa)) return;
    Serial.print(F("  Baseline pressure: ")); Serial.print(baseline_hpa, 3); Serial.println(F(" hPa"));
    Serial.println(F("  Baseline height set to 0.0 m."));

    Serial.println();
    Serial.println(F("  >>> Lift the barometer now. <<<"));
    Serial.print(F("  Reading again in "));
    Serial.print(BARO_LIFT_WAIT_MS / 1000.0f, 1);
    Serial.println(F(" s..."));

    delay(BARO_LIFT_WAIT_MS);

    Serial.println(F("  Hold it there — averaging..."));
    float new_hpa;
    if (!baro_lift_capture(F("Post-lift"), &new_hpa)) return;
    float delta_alt_m = 44330.0f * (1.0f - powf(new_hpa / baseline_hpa, 0.1902949f));

    Serial.print(F("  New pressure: ")); Serial.print(new_hpa, 3); Serial.println(F(" hPa"));
    Serial.print(F("  Height change: "));
    Serial.print(delta_alt_m, 2);
    Serial.println(F(" m"));

    if (delta_alt_m > 0.05f) {
        pass("Barometer detected a lift — height increased");
    } else if (delta_alt_m < -0.05f) {
        info("Height decreased — barometer was lowered, not lifted");
    } else {
        info("No significant height change detected (< 5 cm) — try lifting higher/faster");
    }
}

// ============================================================
//  TEST B — Buzzer HAL (PIN_BUZZER = 3, hardware PWM via FlexPWM)
// ============================================================
static void run_pattern(buzzer_t *b, buzzer_pattern_t p,
                        const char *name, uint32_t listen_ms) {
    Serial.print(F("  ")); Serial.print(name); Serial.print(F(" ... "));
    buzzer_set(b, p);
    uint32_t t0 = millis();
    while (millis() - t0 < listen_ms) buzzer_update(b);
    buzzer_off(b);
    delay(300);
    Serial.println(F("done"));
}

static void test_buzzer() {
    print_banner("TEST B: Buzzer HAL (hardware PWM, pin 3)");
    Serial.print(F("  Frequency: ")); Serial.print(BUZZER_FREQ_HZ);
    Serial.println(F(" Hz  (edit BUZZER_FREQ_HZ in board_pins.h after sweep)"));

    buzzer_t b;
    buzzer_init(&b, &BUZZER_HAL_TEENSY, PIN_BUZZER, BUZZER_FREQ_HZ);

    run_pattern(&b, BUZZ_BOOT,          "BOOT         (1 chirp, holds)", 500);
    run_pattern(&b, BUZZ_SELFTEST_PASS, "SELFTEST PASS (2 chirps, holds)", 700);
    run_pattern(&b, BUZZ_SELFTEST_FAIL, "SELFTEST FAIL (3 long tones)",   2000);
    run_pattern(&b, BUZZ_IDLE,          "IDLE          (chirp/3s, 4s)",   4000);
    run_pattern(&b, BUZZ_ARMED,         "ARMED         (dbl chirp/1s, 3s)", 3000);
    run_pattern(&b, BUZZ_LOCATOR,       "LOCATOR       (1 Hz, 3s)",       3000);

    Serial.println(F("  Frequency sweep tip: change BUZZER_FREQ_HZ in 100 Hz steps"));
    Serial.println(F("  (1500-4500 Hz) and re-run 'B' — pick the loudest frequency."));
    pass("Buzzer PASSED — confirm tone was audible on each pattern");
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
    Serial.println(F("║  5 - SD Card (detect → stage → CSV)      ║"));
    Serial.println(F("║  6 - Logger round-trip (RAM → SD CSV)    ║"));
    Serial.println(F("║  7 - Run ALL tests in sequence            ║"));
    Serial.println(F("║  S - Servo sweep (X and Y axes)          ║"));
    Serial.println(F("║  L - LED test (GREEN/WHITE/RED)           ║"));
    Serial.println(F("║  B - Buzzer patterns                      ║"));
    Serial.println(F("║  H - Barometer lift-height test          ║"));
    Serial.println(F("║  M - Magnetometer calibration (~50 s)    ║"));
    Serial.println(F("║  C - Pyro continuity (LED visual aid)    ║"));
    Serial.println(F("║  A - ARM_SENSE readout (pin 24)          ║"));
    Serial.println(F("║  U - USB log dump (no SD needed)         ║"));
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
    SPI2.begin();
    Wire.begin();
    Wire1.begin();
    delay(100);

    // Release flash from power-down
    SPI2.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    digitalWriteFast(PIN_FLASH_CS, LOW);
    SPI2.transfer(FCMD_RELEASE_PD);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    SPI2.endTransaction();
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
        case 'H': case 'h': test_baro_lift_height(); break;
        case 'M': case 'm': test_mag_calibrate(); break;
        case 'C': case 'c': test_pyro_continuity(); break;
        case 'A': case 'a': test_arm_sense(); break;
        case 'U': case 'u': test_usb_dump(); break;
        case 'R': case 'r': print_menu(); break;
        default:
            Serial.print(F("Unknown command: "));
            Serial.println(c);
            print_menu();
            break;
    }
}