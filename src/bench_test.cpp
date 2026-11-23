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
//    8 — SD diagnostics (line/bridge checks, card init stage by stage)
//    H — Barometer lift-height test (averaged baseline, lift prompt, 2.5 s delay, averaged delta)
//    I — IMU tilt -> gimbal response (IMU init + noise removal; rock each axis while the gimbal answers as in flight, Y/N moves on)
//    M — Magnetometer calibration (rotate ~50 s, then 'Y' to save to EEPROM)
//    C — Pyro continuity test (LEDs as visual aid, pyro battery must be live)
//    F — Pyro fire test (bench load only; SW401 + continuity + typed FIRE + countdown)
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
#include "pid.h"
#include "buzzer.h"
#include "pyro.h"
#include "flash.h"

extern "C" const buzzer_hal_t BUZZER_HAL_TEENSY;

// The flight attitude filter (mahrs_integration.cpp, linked into this
// build too) — Test I steers by the same filter the flight loop does.
// Declared the same way main_control_loop.cpp does.
struct RocketAttitude { float tip_a; float tip_b; float spin; };
void mahrs_init();
void mahrs_seed_from_accel(float ax, float ay, float az);
void mahrs_tick(const LSM6DSOX_Data *imu, const mag_data *mag);
void mahrs_get_attitude(RocketAttitude *out);

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

// Macro value as a string literal, e.g. BT_STR(SERVO_PITCH_PIN) -> "6".
#define BT_STR2(x) #x
#define BT_STR(x)  BT_STR2(x)

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
//  TEST 8 — SD diagnostics
//  For when Test 5's mount fails. Works along the SD path from the
//  Teensy pins out to the card:
//   1. Line check (card out) — each SD line under the Teensy's internal
//      pull-up, then pull-down. A line that reads LOW against the pull-up
//      is held down: bridged to GND at the socket, something unpowered on
//      the net, or a damaged Teensy pin. One that reads HIGH against the
//      pull-down is held up — the board has no SD pull-ups, so that's a
//      bridge to 3.3 V.
//   2. Bridge check (card out) — drives each line in turn and watches the
//      others. The microSD socket's 0.8 mm pins bridge easily.
//   3. Meter check (card out, optional) — drives one line HIGH at a time
//      to be measured at J701. The only check that catches an OPEN line,
//      which reads like a healthy one in 1 and 2. MOSI (26) and SCK (27)
//      reach H4 through the Teensy's underside pads, so they're the
//      likeliest opens.
//   4. Card init (card in) — the SdFat handshake (always run at 400 kHz)
//      and the filesystem mount, at 16, then 4, then 1 MHz, printing the
//      step that failed and what that points to.
// ============================================================
#define SD_DIAG_WAIT_MS  60000

struct SdLine { const char *name; uint8_t pin; uint8_t j701; };
static const SdLine SD_LINES[4] = {    // CS first — it's skipped as a driver with a card in
    { "CS   (pin 0,  J701-2 DAT3)", PIN_SD_CS,   2 },
    { "MISO (pin 1,  J701-7 DAT0)", PIN_SD_MISO, 7 },
    { "MOSI (pin 26, J701-3 CMD) ", PIN_SD_MOSI, 3 },
    { "SCK  (pin 27, J701-5 CLK) ", PIN_SD_SCK,  5 },
};

// Next key, upper-cased, line endings ignored; 0 after SD_DIAG_WAIT_MS.
static int wait_key() {
    uint32_t t0 = millis();
    while (millis() - t0 < SD_DIAG_WAIT_MS) {
        if (!Serial.available()) continue;
        int c = Serial.read();
        if (c != '\r' && c != '\n') return toupper(c);
    }
    return 0;
}

// Card out only — with a card in, driving MISO would fight its DAT0.
static void sd_meter_check() {
    Serial.println(F("  Meter check — DC volts, black lead on a Teensy GND pin (H2 pin 02), not"));
    Serial.println(F("  J701-6, so a socket ground that isn't connected can't skew the readings."));
    Serial.println(F("    First: J701 pin 4 -> 3.3 V to Teensy GND AND to J701 pin 6. Lower to"));
    Serial.println(F("    pin 6 means VSS isn't grounded."));
    for (uint8_t i = 0; i < 4; i++) {
        for (const SdLine &l : SD_LINES) { pinMode(l.pin, OUTPUT); digitalWrite(l.pin, LOW); }
        digitalWrite(SD_LINES[i].pin, HIGH);
        Serial.print(F("    ")); Serial.print(SD_LINES[i].name);
        Serial.print(F(" HIGH: J701 pin ")); Serial.print(SD_LINES[i].j701);
        Serial.println(F(" -> 3.3 V, pins 2/3/5/7 otherwise ~0 V. Any key = next, Q = stop"));
        int c = wait_key();
        if (c == 'Q' || c == 0) break;
    }
    for (const SdLine &l : SD_LINES) pinMode(l.pin, INPUT);
    Serial.println(F("  A pin reading ~0 V while its line was HIGH is open between the Teensy chip"));
    Serial.println(F("  and J701 — for 26/27, suspect the joint on the Teensy's underside pad first."));
    Serial.println(F("  3.3 V on the wrong J701 pin means the lines are crossed."));
}

static bool sd_card_present_now() {
    pinMode(PIN_SD_CD, INPUT);
    return digitalRead(PIN_SD_CD) == SD_CD_PRESENT_LEVEL;
}

// Wait for card detect to show `present` for SD_CD_DEBOUNCE_MS, or for a
// key (line endings ignored). Returns true if card detect got there.
static bool wait_for_card(bool present) {
    while (Serial.available()) Serial.read();
    bool     holding = false;
    uint32_t since   = 0;
    uint32_t t0      = millis();
    while (millis() - t0 < SD_DIAG_WAIT_MS) {
        if (Serial.available()) {
            int c = Serial.read();
            if (c != '\r' && c != '\n') return false;
        }
        if (sd_card_present_now() != present) { holding = false; continue; }
        if (!holding) { holding = true; since = millis(); }
        if (millis() - since >= SD_CD_DEBOUNCE_MS) return true;
    }
    return false;
}

static int read_with_pull(uint8_t pin, uint8_t mode) {
    pinMode(pin, mode);
    delay(1);   // 100k against the socket's few pF settles in microseconds
    return digitalRead(pin);
}

// Counts lines held LOW and lines pulled HIGH. The board has no pull-ups
// on the SD lines, so with the card out a pulled-HIGH line is a fault too.
static void sd_line_check(uint8_t *held_low, uint8_t *pulled_high) {
    *held_low = *pulled_high = 0;
    pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);   // card (if any) deselected
    // CS (index 0) last, so it holds the card off while the others are read.
    for (uint8_t k = 0; k < 4; k++) {
        const SdLine &l = SD_LINES[(k + 1) % 4];
        int up   = read_with_pull(l.pin, INPUT_PULLUP);
        int down = read_with_pull(l.pin, INPUT_PULLDOWN);
        pinMode(l.pin, INPUT);
        Serial.print(F("    ")); Serial.print(l.name);
        Serial.print(F("  pull-up: "));   Serial.print(up   ? F("HIGH") : F("LOW "));
        Serial.print(F("  pull-down: ")); Serial.print(down ? F("HIGH") : F("LOW "));
        if (!up) {
            (*held_low)++;
            Serial.println(F("  HELD LOW"));
        } else if (down) {
            (*pulled_high)++;
            Serial.println(F("  PULLED HIGH"));
        } else {
            Serial.println(F("  OK"));
        }
    }
    pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
}

// Drive one line HIGH then LOW with the rest on the opposite pull; a line
// that follows both ways is bridged to it. Returns the number of bridges.
static uint8_t sd_bridge_check(bool card_in) {
    uint8_t bridges = 0;
    const uint8_t first = card_in ? 1 : 0;   // with a card in, CS stays HIGH — driving it LOW selects the card
    pinMode(PIN_SD_CS, OUTPUT); digitalWrite(PIN_SD_CS, HIGH);
    for (uint8_t d = first; d < 4; d++) {
        bool hi[4] = {}, lo[4] = {};
        pinMode(SD_LINES[d].pin, OUTPUT);
        digitalWrite(SD_LINES[d].pin, HIGH);
        for (uint8_t o = first; o < 4; o++) if (o != d) hi[o] = read_with_pull(SD_LINES[o].pin, INPUT_PULLDOWN);
        digitalWrite(SD_LINES[d].pin, LOW);
        for (uint8_t o = first; o < 4; o++) if (o != d) lo[o] = read_with_pull(SD_LINES[o].pin, INPUT_PULLUP);
        pinMode(SD_LINES[d].pin, INPUT);
        for (uint8_t o = d + 1; o < 4; o++) {   // each pair once
            if (hi[o] && !lo[o]) {
                bridges++;
                Serial.print(F("    BRIDGE: ")); Serial.print(SD_LINES[d].name);
                Serial.print(F(" <-> "));       Serial.println(SD_LINES[o].name);
            }
        }
        for (uint8_t o = first; o < 4; o++) pinMode(SD_LINES[o].pin, INPUT);
    }
    if (!bridges) Serial.println(F("    No bridges between the SD lines"));
    return bridges;
}

// What a failed begin() points to, from SdFat's error code and the last
// byte the card sent.
static void explain_sd_error(uint8_t code, uint8_t data) {
    Serial.print(F("    SdError 0x")); Serial.print(code, HEX);
    Serial.print(F(" ("));            printSdErrorSymbol(&Serial, code);
    Serial.print(F("), data 0x"));    Serial.println(data, HEX);
    switch (code) {
    case SD_CARD_ERROR_NONE:
        Serial.println(F("    -> The card works; there's no FAT32/exFAT filesystem on it."));
        Serial.println(F("       Format it with the SD Association's SD Card Formatter."));
        break;
    case SD_CARD_ERROR_CMD0:
        if (data == 0xFF) {
            Serial.println(F("    -> The card never answered (MISO stayed HIGH). The card has no"));
            Serial.println(F("       power, or CS/SCK/MOSI don't reach it, or its DAT0 doesn't reach"));
            Serial.println(F("       pin 1. J701-4 should read 3.3 V to the Teensy's GND AND to"));
            Serial.println(F("       J701-6 (less to pin 6 = VSS not grounded). Then re-run 8 with"));
            Serial.println(F("       the meter check (M) to find an open line."));
        } else if (data == 0x00) {
            Serial.println(F("    -> MISO is stuck LOW: a bridge to GND, a damaged pin 1, or a card"));
            Serial.println(F("       with no power dragging DAT0 down through its protection diodes."));
        } else {
            Serial.println(F("    -> The card answered with garbage: noisy or marginal signals."));
        }
        break;
    case SD_CARD_ERROR_CMD8:
    case SD_CARD_ERROR_CMD58:
        Serial.println(F("    -> The card answered the reset but garbled the next reply:"));
        Serial.println(F("       marginal supply or signal quality at the socket."));
        break;
    case SD_CARD_ERROR_ACMD41:
        Serial.println(F("    -> The card talks but never finished powering up (2 s timeout)."));
        Serial.println(F("       Almost always the supply: 3.3 V at the socket sagging when the"));
        Serial.println(F("       card draws its ~100 mA bursts. J701 has no decoupling on the"));
        Serial.println(F("       schematic — add >= 10 uF + 100 nF across J701 pins 4 and 6."));
        break;
    default:
        Serial.println(F("    -> The handshake worked but data transfers failed at this clock:"));
        Serial.println(F("       signal quality. Trying slower."));
        break;
    }
}

// Handshake-stage failures happen at the fixed 400 kHz init clock, so a
// slower data clock can't fix them.
static bool sd_error_is_handshake(uint8_t code) {
    return code == SD_CARD_ERROR_NONE || code == SD_CARD_ERROR_CMD0 || code == SD_CARD_ERROR_CMD8 ||
           code == SD_CARD_ERROR_CMD58 || code == SD_CARD_ERROR_ACMD41;
}

static void test_sd_diag() {
    print_banner("TEST 8: SD Diagnostics (SPI1 lines, socket, card init)");
    _bench_sd.end();   // release SPI1 so the pins can be tested as GPIO
    SPI1.end();

    // --- Line + bridge checks, card out ---
    bool card_in = sd_card_present_now();
    if (card_in) {
        Serial.println(F("  Remove the card for the line checks (or send any key to test with it in)..."));
        wait_for_card(false);
        card_in = sd_card_present_now();
    }
    Serial.print(F("  Line check (card "));  Serial.print(card_in ? F("IN") : F("out"));
    Serial.println(F(", internal ~22k pull-up / ~100k pull-down):"));
    uint8_t held_low, pulled_high;
    sd_line_check(&held_low, &pulled_high);
    if (card_in) pulled_high = 0;   // the card's own DAT3 pull-up can lift CS

    Serial.println(F("  Bridge check:"));
    if (card_in) info("Card is in — CS skipped (driving it LOW would select the card)");
    uint8_t bridges = sd_bridge_check(card_in);

    if (held_low) {
        fail("A line is HELD LOW — bridged to GND at the socket (DAT0, J701-7, sits next to VSS), "
             "something unpowered on the net, or a damaged Teensy pin");
    }
    if (pulled_high) {
        fail("A line is PULLED HIGH with no card in, and the board has no SD pull-ups — bridged to "
             "3.3 V (J701's VDD pin 4 sits between CMD and CLK)");
    }
    if (bridges) fail("SD lines are bridged — inspect the socket pins under magnification");
    if (!held_low && !pulled_high && !bridges) pass("No shorts: no line held LOW or HIGH, no bridges between them");

    if (!card_in) {
        Serial.println(F("  Those can't see an OPEN line. Send M for the multimeter check (each"));
        Serial.println(F("  line driven HIGH in turn, measured at J701), any other key to skip."));
        while (Serial.available()) Serial.read();
        if (wait_key() == 'M') sd_meter_check();
    }

    // --- Card init, card in ---
    if (!card_in) {
        Serial.println(F("  Insert the card (or send any key to try without card detect)..."));
        if (!wait_for_card(true)) {
            info("Card detect never showed a card — trying init anyway (the CD switch is separate "
                 "from the contacts)");
        }
    }
    delay(250);   // let the card's supply settle after insertion

    SPI1.setMISO(PIN_SD_MISO);
    SPI1.setMOSI(PIN_SD_MOSI);
    SPI1.setSCK(PIN_SD_SCK);

    const uint8_t clocks_mhz[] = { 16, 4, 1 };
    for (uint8_t mhz : clocks_mhz) {
        Serial.print(F("  Card init, data clock ")); Serial.print(mhz); Serial.print(F(" MHz ... "));
        uint32_t t0 = millis();
        bool ok = _bench_sd.begin(SdSpiConfig(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(mhz), &SPI1));
        uint32_t ms = millis() - t0;
        if (ok) {
            Serial.print(F("OK in ")); Serial.print(ms); Serial.println(F(" ms"));
            Serial.print(F("    Card: "));
            uint8_t type = _bench_sd.card()->type();
            Serial.print(type == SD_CARD_TYPE_SDHC ? F("SDHC/SDXC") : type == SD_CARD_TYPE_SD2 ? F("SD v2") : F("SD v1"));
            Serial.print(F(", ")); Serial.print(_bench_sd.card()->sectorCount() / 2048UL);
            Serial.print(F(" MB, ")); _bench_sd.printFatType(&Serial); Serial.println();
            if (mhz == 16) {
                pass("SD card initialises and mounts at the firmware's 16 MHz — Test 5 should pass now");
            } else {
                char msg[140];
                snprintf(msg, sizeof(msg), "Card only works at %u MHz — set SD_CLOCK_MHZ (logger.cpp) and "
                         "BENCH_SD_CLOCK_MHZ to %u, and check signal quality", (unsigned)mhz, (unsigned)mhz);
                fail(msg);
            }
            return;
        }
        Serial.print(F("FAILED after ")); Serial.print(ms); Serial.println(F(" ms"));
        uint8_t code = _bench_sd.sdErrorCode();
        explain_sd_error(code, _bench_sd.sdErrorData());
        if (sd_error_is_handshake(code)) break;
    }
    fail("SD card init failed — see the diagnosis above");
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
//  TEST S — Servos (SERVO_PITCH_PIN, SERVO_YAW_PIN)
//  Watch the TVC mount physically while this runs.
//  Sweeps each axis to its full travel (SERVO_*_MIN_US / MAX_US).
// ============================================================
static void test_servos() {
    print_banner("TEST S: TVC Servos (pitch = pin " BT_STR(SERVO_PITCH_PIN)
                 ", yaw = pin " BT_STR(SERVO_YAW_PIN) ")");
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

    sweep("PITCH (pin " BT_STR(SERVO_PITCH_PIN) ")", servo_set_pitch, SERVO_PITCH_MAX_ANGLE_DEG, SERVO_PITCH_CENTER_US);
    sweep("YAW   (pin " BT_STR(SERVO_YAW_PIN) ")",   servo_set_yaw,   SERVO_YAW_MAX_ANGLE_DEG,   SERVO_YAW_CENTER_US);

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
//  TEST I — IMU tilt -> gimbal response
//  Checks that the IMU sees the rocket being tipped off its central axis,
//  and that the gimbal answers the way it would in flight.
//   1. IMU init (WHO_AM_I), then a fresh gyro-offset calibration, held
//      still. The offset is what the attitude filter would otherwise
//      integrate into a creeping tilt. Used for this run only — not saved.
//   2. Noise check at rest: residual gyro offset and noise, accel noise,
//      and how far the filtered attitude wanders.
//   3. Z axis: rock the nose toward and away from you for as long as you
//      like while the gimbal swings along Z in answer, then send Y or N for
//      whether it went the right way in both directions. Nothing moves on
//      until you answer. Then the X axis the same way.
//  The gimbal runs exactly as in flight throughout — same rate, gains and
//  limits: the flight attitude filter (mahrs_tick, 125 Hz) -> tip_a / tip_b
//  -> pid_update with the flight gains and limits (each axis's full servo
//  travel) -> servo_set_pitch / servo_set_yaw. Nothing is scaled up for
//  the bench, so the swing is flight-sized.
//  Directions: facing the IMU side, nose up, +Z is toward you and +X to
//  your right. tip_a > 0 is the nose leaning toward +Z, tip_b > 0 toward
//  -X (mahrs_get_attitude, after lsm6dsox_to_body). The PID commands
//  -gain * tip, and the servos must turn that into a nozzle swing AWAY
//  from the lean:
//    +pitch -> nozzle +Z    -pitch -> nozzle -Z    (SERVO_PITCH_PIN)
//    -yaw   -> nozzle +X    +yaw   -> nozzle -X    (SERVO_YAW_PIN)
//  "Nozzle" is the exhaust end of the motor.
// ============================================================
#define TILT_HZ               125         // flight loop rate — mahrs_tick() assumes it
#define TILT_DT               (1.0f / TILT_HZ)
#define TILT_CAL_TRIES        3
#define TILT_SEED_SAMPLES     50
#define TILT_NOISE_TICKS      250         // 2 s at rest
#define TILT_LPF_TAU_S        0.1f        // smooths the readout only — the PID gets the
                                          // filter output unsmoothed, as in flight
#define TILT_SEEN_DEG         5.0f        // lean the IMU must see each way, on each axis
#define TILT_PRINT_MS         250
#define TILT_MAX_ATT_PP_DEG   0.5f        // attitude wander allowed at rest
#define TILT_MAX_GYRO_OFF_DPS 0.3f        // residual offset after calibration
#define TILT_MAX_GYRO_SD_DPS  0.5f
#define TILT_MAX_ACCEL_SD_G   0.02f

struct TiltRun {
    GyroBias       bias;
    uint32_t       next_us;
    LSM6DSOX_Data  imu;              // last sample, body frame
    RocketAttitude att;              // as the flight PID sees it
    float          lean_z, lean_x;   // smoothed lean: nose toward IMU +Z / +X [deg]
    bool           quit;             // Q was sent
};

struct TiltAxis {
    float peak_pos, peak_neg;        // furthest lean seen toward +axis / -axis [deg]
    char  answer;                    // 'Y' / 'N' as entered
};

static void print_signed(float v, uint8_t digits) {
    if (v >= 0.0f) Serial.print('+');
    Serial.print(v, digits);
}

static const char *tilt_where(char sign, char axis) {
    if (axis == 'Z') return sign == '+' ? "toward you" : "away from you";
    return sign == '+' ? "to your right" : "to your left";
}

// Which way a controller output sends the nozzle along `axis`: '+', '-',
// or 0 for (near) centre. +pitch -> +Z, -yaw -> +X.
static char tilt_nozzle_sign(char axis, float pitch, float yaw) {
    const float v = axis == 'Z' ? pitch : -yaw;
    if (fabsf(v) < 0.25f) return 0;
    return v > 0.0f ? '+' : '-';
}

// Next key sent, upper-cased, skipping spaces and line endings; 0 if none.
static int tilt_key() {
    while (Serial.available()) {
        int c = Serial.read();
        if (c != ' ' && c != '\r' && c != '\n') return toupper(c);
    }
    return 0;
}

// One 125 Hz tick of the flight attitude path: read, body frame, filter.
// Returns false if the IMU stopped reading.
static bool tilt_tick(TiltRun *t) {
    const uint32_t tick_us = 1000000UL / TILT_HZ;
    while ((int32_t)(micros() - t->next_us) < 0) {}
    const uint32_t now = micros();
    t->next_us += tick_us;
    if ((int32_t)(now - t->next_us) > 0) t->next_us = now + tick_us;   // fell behind — don't burst

    lsm6dsox_read(&t->imu, &t->bias);
    if (!t->imu.valid) {
        fail("IMU stopped returning valid data — run Test 2");
        return false;
    }
    lsm6dsox_to_body(&t->imu);
    mag_data no_mag = {};            // valid = false: tilt only, no heading needed
    mahrs_tick(&t->imu, &no_mag);
    mahrs_get_attitude(&t->att);
    const float a = TILT_DT / (TILT_LPF_TAU_S + TILT_DT);
    t->lean_z += a * ( t->att.tip_a - t->lean_z);
    t->lean_x += a * (-t->att.tip_b - t->lean_x);
    return true;
}

// The flight loop's control step (main_control_loop.cpp, section 5).
static void tilt_steer(const TiltRun *t, PIDController *pp, PIDController *py,
                       float *pitch, float *yaw) {
    *pitch = pid_update(pp, 0.0f, t->att.tip_a, TILT_DT);
    *yaw   = pid_update(py, 0.0f, t->att.tip_b, TILT_DT);
    servo_set_pitch(*pitch);
    servo_set_yaw(*yaw);
}

// Tick, and stop on Q. False on Q (t->quit) or IMU loss.
static bool tilt_tick_or_quit(TiltRun *t) {
    if (tilt_key() == 'Q') { t->quit = true; return false; }
    return tilt_tick(t);
}

// One live line: the lean on `axis` (and on the other axis, which should
// stay small), the controller's output to that axis's servo, and where that
// puts the nozzle.
static void tilt_print_live(const TiltRun *t, char axis, float pitch, float yaw) {
    const bool  z   = axis == 'Z';
    const float cmd = z ? pitch : yaw;
    const int   us  = z ? (int)servo_get_pitch_us() - SERVO_PITCH_CENTER_US
                        : (int)servo_get_yaw_us()   - SERVO_YAW_CENTER_US;
    Serial.print(F("        nose ")); Serial.print(axis); Serial.print(' ');
    print_signed(z ? t->lean_z : t->lean_x, 1);
    Serial.print(F(" (")); Serial.print(z ? 'X' : 'Z'); Serial.print(' ');
    print_signed(z ? t->lean_x : t->lean_z, 1);
    Serial.print(F(") deg  ->  ")); Serial.print(z ? F("pitch ") : F("yaw "));
    print_signed(cmd, 1); Serial.print(F(" deg = "));
    if (us >= 0) Serial.print('+');
    Serial.print(us); Serial.print(F(" us"));
    const float lim = z ? SERVO_PITCH_MAX_ANGLE_DEG : SERVO_YAW_MAX_ANGLE_DEG;
    if (fabsf(cmd) >= lim - 0.05f) Serial.print(F(" (limit)"));
    Serial.print(F("  ->  nozzle "));
    const char n = tilt_nozzle_sign(axis, pitch, yaw);
    if (!n) { Serial.println(F("centred")); return; }
    Serial.print(n); Serial.print(axis); Serial.print(F(", ")); Serial.println(tilt_where(n, axis));
}

// One axis, live until you answer: rock the nose toward +axis and -axis as
// often as you like while the gimbal answers as in flight, then send Y / N.
// Nothing moves on until you do. False on Q or IMU loss.
static bool tilt_axis(TiltRun *t, PIDController *pp, PIDController *py,
                      char axis, bool last, TiltAxis *out) {
    *out = { 0.0f, 0.0f, 0 };
    Serial.println();
    Serial.print(F("  ---- ")); Serial.print(axis); Serial.print(F(" axis ("));
    Serial.print(axis == 'Z' ? F("pitch servo, pin " BT_STR(SERVO_PITCH_PIN))
                             : F("yaw servo, pin " BT_STR(SERVO_YAW_PIN)));
    Serial.println(F(") ----"));
    Serial.print(F("  Rock the nose ")); Serial.print(tilt_where('+', axis));
    Serial.print(F(" (+")); Serial.print(axis); Serial.print(F(") and "));
    Serial.print(tilt_where('-', axis));
    Serial.print(F(" (-")); Serial.print(axis); Serial.println(F("), about 10-15 deg,"));
    Serial.println(F("  back and forth as long as you like. The nozzle should swing the"));
    Serial.println(F("  opposite way each time:"));
    for (uint8_t i = 0; i < 2; i++) {
        const char  lean = i == 0 ? '+' : '-', noz = i == 0 ? '-' : '+';
        const char *w    = tilt_where(lean, axis);
        Serial.print(F("    nose ")); Serial.print(lean); Serial.print(axis);
        Serial.print(F(" (")); Serial.print(w); Serial.print(')');
        for (size_t k = strlen(w); k < 14; k++) Serial.print(' ');
        Serial.print(F("->  nozzle ")); Serial.print(noz); Serial.print(axis);
        Serial.print(F(" (")); Serial.print(tilt_where(noz, axis)); Serial.println(')');
    }
    Serial.println(F("  Send Y if it went that way in BOTH directions, N if not. Nothing moves"));
    Serial.print(F("  on until you answer — then "));
    Serial.println(last ? F("the test finishes.") : F("it's the X axis."));

    pid_reset(pp);   // a fresh start, as on the flight loop's TVC-enable edge
    pid_reset(py);
    float    pitch = 0.0f, yaw = 0.0f;
    float    pr_lean = NAN, pr_off = NAN, pr_cmd = NAN;   // NaN: nothing printed yet
    uint32_t last_print = 0;

    while (true) {
        const int c = tilt_key();
        if (c == 'Y' || c == 'N') { out->answer = (char)c; break; }
        if (c == 'Q') { t->quit = true; return false; }
        if (c) Serial.println(F("        ? Send Y, N or Q"));

        if (!tilt_tick(t)) return false;
        tilt_steer(t, pp, py, &pitch, &yaw);

        const float lean = axis == 'Z' ? t->lean_z : t->lean_x;
        const float off  = axis == 'Z' ? t->lean_x : t->lean_z;
        const float cmd  = axis == 'Z' ? pitch : yaw;
        out->peak_pos = fmaxf(out->peak_pos,  lean);
        out->peak_neg = fmaxf(out->peak_neg, -lean);

        // Live readout — whenever something has moved, at most every TILT_PRINT_MS.
        const uint32_t now = millis();
        if (now - last_print >= TILT_PRINT_MS &&
            !(fabsf(lean - pr_lean) < 0.5f && fabsf(off - pr_off) < 0.5f && fabsf(cmd - pr_cmd) < 0.2f)) {
            tilt_print_live(t, axis, pitch, yaw);
            pr_lean = lean; pr_off = off; pr_cmd = cmd; last_print = now;
        }
    }

    Serial.print(F("        > ")); Serial.println(out->answer);
    Serial.print(F("        IMU saw the nose lean up to +")); Serial.print(axis); Serial.print(' ');
    Serial.print(out->peak_pos, 1); Serial.print(F(" deg and -")); Serial.print(axis); Serial.print(' ');
    Serial.print(out->peak_neg, 1); Serial.println(F(" deg"));
    return true;
}

static void test_tilt_response() {
    print_banner("TEST I: IMU Tilt -> Gimbal Response (flight controller)");
    Serial.println(F("  Stand the rocket nose up on its central axis and hold it still."));
    Serial.println(F("  Starting in 3 s..."));
    delay(3000);

    // --- 1. IMU init ---
    if (!lsm6dsox_init()) {
        fail("lsm6dsox_init() returned false — IMU not responding (run Test 2)");
        return;
    }
    pass("lsm6dsox_init() OK — WHO_AM_I 0x6C confirmed");
    servo_init();   // centred

    // --- 2. Remove the noise: measure the gyro's offset at rest ---
    TiltRun t = {};
    bool cal_ok = false;
    for (uint8_t i = 1; i <= TILT_CAL_TRIES && !cal_ok; i++) {
        cal_ok = lsm6dsox_calibrate_gyro(&t.bias);
        if (!cal_ok && i < TILT_CAL_TRIES) {
            Serial.println(F("  Keep it still — retrying..."));
            delay(1000);
        }
    }
    if (cal_ok) {
        pass("Gyro offset measured and removed (this run only — not saved to EEPROM)");
    } else if (lsm6dsox_load_bias(&t.bias)) {
        Serial.println(F("  [WARN] Couldn't hold still long enough — using the gyro offset saved in EEPROM"));
    } else {
        fail("Gyro calibration kept detecting motion and no offset is saved — rest the rocket "
             "against something and re-run I");
        return;
    }

    // Start the attitude filter at the measured tilt, as the flight firmware does at boot.
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    int   n  = 0;
    for (int i = 0; i < TILT_SEED_SAMPLES; i++) {
        LSM6DSOX_Data s;
        lsm6dsox_read(&s, &t.bias);
        lsm6dsox_to_body(&s);
        if (s.valid) { sx += s.ax; sy += s.ay; sz += s.az; n++; }
        delay(1000 / TILT_HZ);
    }
    if (n < TILT_SEED_SAMPLES / 2) {
        fail("Too many invalid IMU reads — SPI issue (run Test 2)");
        return;
    }
    mahrs_init();
    mahrs_seed_from_accel(sx / n, sy / n, sz / n);
    mahrs_get_attitude(&t.att);
    t.lean_z  =  t.att.tip_a;
    t.lean_x  = -t.att.tip_b;
    t.next_us = micros();
    while (Serial.available()) Serial.read();   // drop the 'I' line ending

    // Noise check: what's left at rest, through the same filter the flight loop uses.
    Serial.println(F("  Measuring noise at rest (2 s)..."));
    float g_sum[3] = {}, g_sq[3] = {}, a_sum = 0.0f, a_sq = 0.0f;
    float ta_min = INFINITY, ta_max = -INFINITY, tb_min = INFINITY, tb_max = -INFINITY;
    for (int i = 0; i < TILT_NOISE_TICKS; i++) {
        if (!tilt_tick_or_quit(&t)) {
            servo_center();
            if (t.quit) info("Stopped");
            return;
        }
        const float g[3] = { t.imu.gx, t.imu.gy, t.imu.gz };
        for (int k = 0; k < 3; k++) { g_sum[k] += g[k]; g_sq[k] += g[k] * g[k]; }
        const float am = sqrtf(t.imu.ax * t.imu.ax + t.imu.ay * t.imu.ay + t.imu.az * t.imu.az);
        a_sum += am; a_sq += am * am;
        ta_min = fminf(ta_min, t.att.tip_a); ta_max = fmaxf(ta_max, t.att.tip_a);
        tb_min = fminf(tb_min, t.att.tip_b); tb_max = fmaxf(tb_max, t.att.tip_b);
    }
    const float N = (float)TILT_NOISE_TICKS;
    float g_off_max = 0.0f, g_sd_max = 0.0f;
    Serial.print(F("  Gyro residual offset (body x/y/z):"));
    for (int k = 0; k < 3; k++) {
        const float mean = g_sum[k] / N;
        g_off_max = fmaxf(g_off_max, fabsf(mean));
        g_sd_max  = fmaxf(g_sd_max, sqrtf(fmaxf(g_sq[k] / N - mean * mean, 0.0f)));
        Serial.print(' '); print_signed(mean, 3);
    }
    Serial.print(F(" dps, noise ")); Serial.print(g_sd_max, 3); Serial.println(F(" dps (1 sd)"));
    const float a_mean = a_sum / N;
    const float a_sd   = sqrtf(fmaxf(a_sq / N - a_mean * a_mean, 0.0f));
    Serial.print(F("  Accel ")); Serial.print(a_mean, 3);
    Serial.print(F(" g, noise ")); Serial.print(a_sd, 4); Serial.println(F(" g (1 sd)"));
    const float att_pp = fmaxf(ta_max - ta_min, tb_max - tb_min);
    Serial.print(F("  Attitude: ")); Serial.print(hypotf(t.att.tip_a, t.att.tip_b), 1);
    Serial.print(F(" deg from vertical, wandering ")); Serial.print(att_pp, 2); Serial.println(F(" deg p-p"));

    if (a_mean < 0.9f || a_mean > 1.1f) {
        servo_center();
        fail("Accel doesn't read 1 g at rest — the IMU isn't measuring gravity correctly (run Test 2)");
        return;
    }
    bool quiet = true;
    if (g_off_max > TILT_MAX_GYRO_OFF_DPS) {
        quiet = false;
        Serial.println(F("  [WARN] Gyro offset left over after calibration — it moved, or the offset is drifting"));
    }
    if (g_sd_max > TILT_MAX_GYRO_SD_DPS || a_sd > TILT_MAX_ACCEL_SD_G) {
        quiet = false;
        Serial.println(F("  [WARN] Noisy at rest — vibration, a loose board, or it moved"));
    }
    if (att_pp > TILT_MAX_ATT_PP_DEG) {
        quiet = false;
        Serial.println(F("  [WARN] Attitude wandered at rest — it moved, or the readings are noisy"));
    }
    if (quiet) pass("Noise removed — attitude holds steady at rest");

    // --- 3. Rock each axis while the gimbal answers as in flight ---
    Serial.println();
    Serial.println(F("  Face the IMU side of the board, nose up: +Z is toward you, +X to your"));
    Serial.println(F("  right. From here the gimbal runs exactly as in flight: 125 Hz, flight"));
    Serial.print(F("  gains, limits of +/-")); Serial.print(SERVO_PITCH_MAX_ANGLE_DEG, 1);
    Serial.print(F(" deg (+/-")); Serial.print(SERVO_PITCH_SPAN_US);
    Serial.print(F(" us) on pitch and +/-")); Serial.print(SERVO_YAW_MAX_ANGLE_DEG, 1);
    Serial.print(F(" deg (+/-")); Serial.print(SERVO_YAW_SPAN_US); Serial.println(F(" us) on yaw:"));
    Serial.println(F("  the full servo travel. Nothing is scaled up, so the swing grows with the"));
    Serial.println(F("  lean and with how long you hold it. After a long hold the swing back"));
    Serial.println(F("  takes a few seconds (the integral term unwinding). The readout shows the"));
    Serial.println(F("  command, with (limit) when it's at full travel."));
    Serial.println(F("  Send Q to stop."));

    PIDController pid_p, pid_y;
    pid_init(&pid_p, SERVO_PITCH_MAX_ANGLE_DEG);   // flight gains and limits
    pid_init(&pid_y, SERVO_YAW_MAX_ANGLE_DEG);

    TiltAxis z, x;
    if (!tilt_axis(&t, &pid_p, &pid_y, 'Z', false, &z) ||
        !tilt_axis(&t, &pid_p, &pid_y, 'X', true,  &x)) {
        servo_center();
        if (t.quit) info("Stopped — gimbal centred, no result");
        return;
    }
    servo_center();

    // --- Summary ---
    Serial.println();
    const TiltAxis *axes[2]  = { &z, &x };
    const char      names[2] = { 'Z', 'X' };
    bool imu_ok = true;
    for (uint8_t i = 0; i < 2; i++) {
        const TiltAxis *a    = axes[i];
        const bool      seen = a->peak_pos >= TILT_SEEN_DEG && a->peak_neg >= TILT_SEEN_DEG;
        if (!seen) imu_ok = false;
        Serial.print(F("    ")); Serial.print(names[i]);
        Serial.print(a->answer == 'Y' ? F(" axis   you: correct       IMU saw it lean +")
                                      : F(" axis   you: NOT correct   IMU saw it lean +"));
        Serial.print(a->peak_pos, 1); Serial.print(F(" / -")); Serial.print(a->peak_neg, 1);
        Serial.print(F(" deg"));
        if (!seen) { Serial.print(F("  <- under ")); Serial.print(TILT_SEEN_DEG, 0); Serial.print(F(" deg")); }
        Serial.println();
    }

    if (imu_ok) {
        pass("IMU saw the rocket tip off its central axis both ways, on both axes");
    } else {
        fail("IMU didn't see a 5+ deg lean both ways on every axis — tilt further, or the IMU is "
             "reading the tilt on the other axis (watch the 'nose' readout; lsm6dsox_to_body)");
    }

    if (z.answer == 'Y') {
        pass("Z axis (pitch servo, pin " BT_STR(SERVO_PITCH_PIN) "): nozzle swings away from the lean both ways");
    } else {
        fail("Z axis (pitch servo, pin " BT_STR(SERVO_PITCH_PIN) ") moved wrong. Readout's nose sign "
             "matched your tilt but the nozzle went toward the lean: flip SERVO_PITCH_INVERT in "
             "include/servo_driver.h. Readout's sign was wrong: the IMU axes are mapped wrong "
             "(lsm6dsox_to_body). Moved along X instead: swap SERVO_PITCH_PIN / SERVO_YAW_PIN and "
             "their MIN/MAX/CENTER numbers in servo_driver.h (not the plugs). Didn't move: servo "
             "power, pin " BT_STR(SERVO_PITCH_PIN) " lead");
    }
    if (x.answer == 'Y') {
        pass("X axis (yaw servo, pin " BT_STR(SERVO_YAW_PIN) "): nozzle swings away from the lean both ways");
    } else {
        fail("X axis (yaw servo, pin " BT_STR(SERVO_YAW_PIN) ") moved wrong. Readout's nose sign "
             "matched your tilt but the nozzle went toward the lean: flip SERVO_YAW_INVERT in "
             "include/servo_driver.h. Readout's sign was wrong: the IMU axes are mapped wrong "
             "(lsm6dsox_to_body). Moved along Z instead: swap SERVO_PITCH_PIN / SERVO_YAW_PIN and "
             "their MIN/MAX/CENTER numbers in servo_driver.h (not the plugs). Didn't move: servo "
             "power, pin " BT_STR(SERVO_YAW_PIN) " lead");
    }

    if (imu_ok && z.answer == 'Y' && x.answer == 'Y') {
        pass("Tilt response PASSED — the gimbal corrects a lean on both axes as in flight");
    } else {
        fail("Tilt response FAILED — do NOT fly until the IMU and both axes are right");
    }
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
//  TEST F — Pyro fire
//  Fires one channel through the flight driver (pyro_fire_main /
//  pyro_fire_drogue, PYRO_FIRE_DURATION_MS pulse) into a bench load: a
//  loose e-match NOT in a charge, or a resistor/bulb load. Never with a
//  charge loaded.
//
//  Gates, in order: pyro power at the load, "FIRE" typed, then a
//  countdown any key aborts. "Power at the load" is the channel's
//  continuity sense node, which only reads when the pack is connected,
//  SW401 is closed AND a load completes the circuit — the direct test of
//  whether a fire can do anything. ARM_SENSE is reported alongside (it
//  tells "no power" from "no load") but doesn't gate: it reads the same
//  pack one step removed, and a misreading pin 24 shouldn't block a test.
//
//  Evidence, from the continuity sense node (low-side switch: PYRO PWR ->
//  match -> sense node -> MOSFET -> GND, see pyro.h):
//    during the pulse — the node drops to ~0 V: the MOSFET switched on
//    after the pulse  — continuity gone: the e-match burned open
//  plus the pack sag on ARM_SENSE while current flowed.
//
//  pyro_fire_*() saves a "fired" flag to EEPROM, and the flight firmware
//  only clears it on an arming edge seen after boot. A bench fire left in
//  EEPROM would stop the main firing in flight if the board then boots
//  with SW401 already closed — so the flags are cleared again afterwards.
// ============================================================
#define PYRO_TEST_CONFIRM_MS   30000
#define PYRO_TEST_COUNTDOWN_S  5
#define PYRO_TEST_SETTLE_MS    500     // after the pulse, before re-checking continuity
#define PYRO_TEST_MIN_SENSE_V  0.5f    // ~2 V at the load; a 2S pack gives 1.5-2.1 V here

static float bench_pack_v() {
    return arm_counts_to_v(analogRead(PIN_ARM_SENSE)) / BENCH_ARM_SENSE_DIVIDER_RATIO;
}

// 16 samples, as pyro_check_continuity() averages them.
static float avg_pin_v(uint8_t pin) {
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) { acc += analogRead(pin); delayMicroseconds(200); }
    return arm_counts_to_v(acc / 16);
}

// True only if the next four characters (line endings skipped) are "FIRE".
static bool read_fire_word() {
    static const char WORD[] = "FIRE";
    uint8_t  n  = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < PYRO_TEST_CONFIRM_MS) {
        if (!Serial.available()) continue;
        int c = Serial.read();
        if (c == '\r' || c == '\n') continue;
        if (c != WORD[n]) return false;
        if (++n == 4) return true;
    }
    return false;
}

// One chirp + red LED flash per second; any key (line endings ignored)
// aborts. Returns false if aborted.
static bool pyro_countdown() {
    pinMode(PIN_LED_RED, OUTPUT);
    for (int s = PYRO_TEST_COUNTDOWN_S; s > 0; s--) {
        Serial.print(F("    ")); Serial.print(s); Serial.println(F("..."));
        digitalWrite(PIN_LED_RED, HIGH);
        BUZZER_HAL_TEENSY.tone_on(PIN_BUZZER, BUZZER_FREQ_HZ);
        bool     beeping = true;
        uint32_t t0      = millis();
        while (millis() - t0 < 1000) {
            if (beeping && millis() - t0 >= 150) {
                BUZZER_HAL_TEENSY.tone_off(PIN_BUZZER);
                digitalWrite(PIN_LED_RED, LOW);
                beeping = false;
            }
            if (!Serial.available()) continue;
            int c = Serial.read();
            if (c == '\r' || c == '\n') continue;   // the Enter after "FIRE"
            BUZZER_HAL_TEENSY.tone_off(PIN_BUZZER);
            digitalWrite(PIN_LED_RED, LOW);
            return false;
        }
    }
    return true;
}

static void test_pyro_fire() {
    print_banner("TEST F: Pyro Fire (flight driver, bench load only)");
    Serial.println(F("  Load: a loose e-match NOT in a charge, or a resistor/bulb test load."));
    Serial.println(F("  NEVER with a charge loaded. Keep hands and face clear of the match."));

    analogReadResolution(12);
    pinMode(PIN_ARM_SENSE,   INPUT_PULLDOWN);   // as flight
    pinMode(PIN_PYRO1_SENSE, INPUT);
    pinMode(PIN_PYRO2_SENSE, INPUT);

    PyroState pyro;
    pyro_init(&pyro);   // fire pins LOW before their drivers are enabled

    Serial.println(F("  Channel? 1 = PYRO1 (main, pin 32)  2 = PYRO2 (pin 31)  anything else cancels"));
    while (Serial.available()) Serial.read();
    int ch = wait_key();
    if (ch != '1' && ch != '2') { info("Cancelled — nothing fired"); return; }
    const bool    is_main   = (ch == '1');
    const uint8_t fire_pin  = is_main ? PIN_PYRO1_FIRE  : PIN_PYRO2_FIRE;
    const uint8_t sense_pin = is_main ? PIN_PYRO1_SENSE : PIN_PYRO2_SENSE;

    // --- Gate 1: pyro power at the load ---
    float arm_v        = avg_pin_v(PIN_ARM_SENSE);
    bool  arm_reads    = arm_v > BENCH_ARM_SENSE_ARMED_V;
    float pack_before  = arm_v / BENCH_ARM_SENSE_DIVIDER_RATIO;
    float sense_before = avg_pin_v(sense_pin);
    Serial.print(F("  ARM_SENSE:        ")); Serial.print(arm_v, 2);
    Serial.print(F(" V -> pack "));         Serial.print(pack_before, 2); Serial.println(F(" V"));
    Serial.print(F("  Continuity sense: ")); Serial.print(sense_before, 2);
    Serial.print(F(" V -> pack at the load ")); Serial.print(sense_before / PYRO_CONT_DIVIDER_RATIO, 2);
    Serial.println(F(" V"));
    if (sense_before < PYRO_TEST_MIN_SENSE_V) {
        if (arm_reads) fail("Pyro power is on but nothing is connected — connect the e-match/test load, then re-run F");
        else           fail("No pyro power at the load — connect the pyro battery, close SW401, connect the "
                            "load, then re-run F");
        return;
    }
    if (!arm_reads) {
        info("ARM_SENSE reads low, but the load sees pyro power — pin 24 (an H4 underside pad) or its "
             "divider isn't reaching the Teensy. Going by the sense node (run A to look at ARM_SENSE)");
    }

    // --- Gate 2: typed confirmation, Gate 3: countdown ---
    Serial.print(F("  Type FIRE (capitals) to fire channel ")); Serial.print((char)ch);
    Serial.println(F(" — anything else cancels (30 s)."));
    while (Serial.available()) Serial.read();
    if (!read_fire_word()) { info("Cancelled — nothing fired"); return; }
    Serial.print(F("  Firing in ")); Serial.print(PYRO_TEST_COUNTDOWN_S);
    Serial.println(F(" s — send any key to ABORT"));
    if (!pyro_countdown()) { pyro_safe_all(&pyro); info("ABORTED — nothing fired"); return; }

    // --- Fire through the flight driver ---
    pyro_arm(&pyro);
    pyro_clear_fired(&pyro);   // a flag left by an earlier flight or test would block the fire
    float    sense_min = sense_before, pack_min = pack_before;
    uint32_t t0 = millis();
    if (is_main) pyro_fire_main(&pyro, PYRO_MAIN_MIN_ALT_M);   // the altitude floor is a flight gate, not a bench one
    else         pyro_fire_drogue(&pyro);
    while ((is_main ? pyro.main_firing : pyro.drogue_firing) &&
           millis() - t0 < PYRO_FIRE_DURATION_MS + 100) {      // bound in case the pulse never ends
        float s = arm_counts_to_v(analogRead(sense_pin));
        float p = bench_pack_v();
        if (s < sense_min) sense_min = s;
        if (p < pack_min)  pack_min  = p;
        pyro_update(&pyro);
    }
    uint32_t pulse_ms = millis() - t0;
    pyro_safe_all(&pyro);

    delay(PYRO_TEST_SETTLE_MS);
    float sense_after = avg_pin_v(sense_pin);
    bool  cont_after  = sense_after > 0.5f * sense_before;   // still near its pre-fire level = load intact
    pyro_clear_fired(&pyro);   // leave EEPROM as an unflown board — see the header

    // --- Report ---
    Serial.print(F("  Pulse: ")); Serial.print(pulse_ms);
    Serial.print(F(" ms on pin ")); Serial.println(fire_pin);
    Serial.print(F("  Sense node: ")); Serial.print(sense_before, 2);
    Serial.print(F(" V before -> "));  Serial.print(sense_min, 2);
    Serial.print(F(" V min during -> ")); Serial.print(sense_after, 2); Serial.println(F(" V after"));
    if (arm_reads) {
        Serial.print(F("  Pyro pack:  ")); Serial.print(pack_before, 2);
        Serial.print(F(" V before -> "));  Serial.print(pack_min, 2); Serial.println(F(" V min during (sag = current flowing)"));
    } else {
        Serial.println(F("  Pyro pack sag: not available — ARM_SENSE isn't reading"));
    }

    if (sense_min > 0.5f * sense_before) {
        char msg[160];
        snprintf(msg, sizeof(msg), "Sense node never dropped — the MOSFET didn't switch on. Check the gate drive "
                 "from pin %d (a Teensy underside pad), the MOSFET, and PYRO PWR", fire_pin);
        fail(msg);
    } else if (!cont_after) {
        pass(is_main ? "PYRO1 FIRED — MOSFET switched and the e-match burned open"
                     : "PYRO2 FIRED — MOSFET switched and the e-match burned open");
    } else {
        info("MOSFET switched, but the load is still intact. Expected for a bulb/resistor load — did "
             "it light or heat? An e-match still intact didn't ignite: check its all-fire current "
             "against the pack and wiring");
    }
    info("Fired flags cleared — EEPROM left as an unflown board");
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
    Serial.println(F("║  8 - SD diagnostics (when 5 fails)       ║"));
    Serial.println(F("║  S - Servo sweep (X and Y axes)          ║"));
    Serial.println(F("║  I - IMU tilt -> gimbal response         ║"));
    Serial.println(F("║  L - LED test (GREEN/WHITE/RED)           ║"));
    Serial.println(F("║  B - Buzzer patterns                      ║"));
    Serial.println(F("║  H - Barometer lift-height test          ║"));
    Serial.println(F("║  M - Magnetometer calibration (~50 s)    ║"));
    Serial.println(F("║  C - Pyro continuity (LED visual aid)    ║"));
    Serial.println(F("║  F - Pyro FIRE test (bench load only)    ║"));
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
        case '8': test_sd_diag();         break;
        case 'S': case 's': test_servos(); break;
        case 'I': case 'i': test_tilt_response(); break;
        case 'L': case 'l': test_leds();   break;
        case 'B': case 'b': test_buzzer(); break;
        case 'H': case 'h': test_baro_lift_height(); break;
        case 'M': case 'm': test_mag_calibrate(); break;
        case 'C': case 'c': test_pyro_continuity(); break;
        case 'F': case 'f': test_pyro_fire(); break;
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