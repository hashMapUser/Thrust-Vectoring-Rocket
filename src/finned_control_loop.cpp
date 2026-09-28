// ============================================================
//  finned_control_loop.cpp
//
//  Minimal flight firmware for a finned (no-TVC) recovery-only flight.
//  Uses simple_fsm.h instead of flight_sm.h — no ARMED flight-state, no
//  pad-rest latch, no PID/servo control. The arming switch (SW401) is
//  still read here (see the "ARMING SWITCH" block below) to clear the
//  EEPROM fired-flags and check continuity for each new flight, but it
//  does not gate launch detection the way flight_sm.h's does.
//
//  SAFETY: the pyro channel is armed once at boot, unconditionally.
//  See include/simple_fsm.h for the full safety note before powering
//  this up with a live e-match connected.
//
//  Build with: pio run -e finned -t upload
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Watchdog_t4.h"

#include "board_pins.h"
#include "lsm6dsox.h"
#include "lps22hb.h"
#include "alt_estimator.h"
#include "simple_fsm.h"
#include "pyro.h"
#include "buzzer.h"
#include "logger.h"

extern "C" const buzzer_hal_t BUZZER_HAL_TEENSY;

// --- WATCHDOG ---
static WDT_T4<WDT3> wdt;

// --- GLOBAL MODULE CONTEXTS ---
static GyroBias     gyro_bias;
static AltEstimator alt_est;
static SimpleFSM    fsm;
static PyroState    pyros;
static buzzer_t     buzz;

// --- TIMING ---
const uint32_t LOOP_INTERVAL_US = 8000;  // 125 Hz
uint32_t last_loop_time = 0;

// --- ARMING SWITCH (SW401) ---
// simple_fsm.h is deliberately minimal and has no ARMED state — pyro_arm()
// is called unconditionally at boot, same as before. What SW401 DOES still
// need to drive here: clearing the EEPROM "fired" flags for a new flight
// (see pyro_clear_fired()) and giving the pad crew an audible continuity
// check, same as main_control_loop.cpp. See that file's arming-switch
// block for the full policy this is a lighter version of.
#define ARM_SENSE_DIVIDER_RATIO 0.3197f
#define ARM_SENSE_ARMED_V       1.2f
#define ARM_DEBOUNCE_MS         100

static inline float read_arm_sense_v() {
    return analogRead(PIN_ARM_SENSE) * 3.30f / 4095.0f;
}
static inline float read_pack_voltage() {
    return read_arm_sense_v() / ARM_SENSE_DIVIDER_RATIO;
}

static bool     arm_raw_prev  = false;
static uint32_t arm_edge_ms   = 0;
static bool     arm_now       = false;
static bool     arm_prev      = false;
static bool     seen_disarmed = false;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // ARM_SENSE / PYRO1_SENSE — see the arming-switch block in loop()
    analogReadResolution(12);
    pinMode(PIN_ARM_SENSE, INPUT);
    pinMode(PIN_PYRO1_SENSE, INPUT);

    // IMU CS must be HIGH before SPI.begin()
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);

    SPI.begin();
    Wire.begin();
    delay(100);

    buzzer_init(&buzz, &BUZZER_HAL_TEENSY, PIN_BUZZER, BUZZER_FREQ_HZ);
    buzzer_set(&buzz, BUZZ_BOOT);
    pyro_init(&pyros);

    // Sensor init before anything that could compete for power — see the
    // matching note in main_control_loop.cpp for why this ordering matters.
    if (!lps22hb_init()) {
        Serial.println("[WARN] LPS22HB not found — baro disabled");
    }

    if (!lsm6dsox_init()) {
        Serial.println("[FAULT] LSM6DSOX init failed — check SPI wiring");
        buzzer_set(&buzz, BUZZ_SELFTEST_FAIL);
        while (true) { buzzer_update(&buzz); delay(10); }
    }
    lsm6dsox_load_bias(&gyro_bias);

    logger_init();

    float ground_hpa = 1013.25f;
    {
        LPS22HB_Data baro_ground;
        lps22hb_read(&baro_ground);
        if (baro_ground.valid) ground_hpa = baro_ground.pressure_pa / 100.0f;
    }
    alt_init(&alt_est, ground_hpa);

    // Pre-flight accel bias lock — hold still on the pad while this runs.
    Serial.println("[INIT] Hold still — calibrating accel bias (2 s)...");
    uint32_t cal_start = millis();
    while (millis() - cal_start < 2000) {
        LSM6DSOX_Data d;
        lsm6dsox_read(&d, &gyro_bias);
        alt_calibrate_sample(&alt_est, -d.ax);
        delay(8);
    }
    alt_calibrate_finish(&alt_est);

    simple_fsm_init(&fsm, alt_est.altitude_m);

    // SAFETY: arms the pyro channel unconditionally — see simple_fsm.h.
    // Do not power this up with a live e-match connected unless you are
    // at the pad, ready to fly.
    pyro_arm(&pyros);
    {
        bool arm_raw_boot = read_arm_sense_v() > ARM_SENSE_ARMED_V;
        Serial.print("[PYRO] Armed (software). SW401 reads: ");
        Serial.println(arm_raw_boot ? "CLOSED (armed)" : "OPEN (safe)");
        if (arm_raw_boot) {
            Serial.println("[ARM] SW401 already closed at boot — likely a mid-flight reset. "
                            "Fired flags kept.");
        }
    }

    {
        WDT_timings_t wdt_cfg;
        wdt_cfg.timeout = 0.5f;   // 500 ms
        wdt.begin(wdt_cfg);
    }

    buzzer_set(&buzz, BUZZ_SELFTEST_PASS);
    Serial.println("FINNED RECOVERY FIRMWARE READY. Pyro ARMED. Waiting for launch.");
}

void loop() {
    uint32_t now_us = micros();
    if (now_us - last_loop_time < LOOP_INTERVAL_US) return;

    float dt = (now_us - last_loop_time) / 1000000.0f;
    last_loop_time = now_us;

    wdt.feed();
    uint32_t now_ms = millis();

    // ── ARMING SWITCH (SW401) ─────────────────────────────────
    // Lighter version of main_control_loop.cpp's block: this build has
    // no ARMED state, so all it does is clear the fired flags and give
    // an audible continuity check on a genuine new-flight arming edge.
    {
        bool arm_raw = read_arm_sense_v() > ARM_SENSE_ARMED_V;
        if (arm_raw != arm_raw_prev) { arm_edge_ms = now_ms; arm_raw_prev = arm_raw; }
        if ((now_ms - arm_edge_ms) >= ARM_DEBOUNCE_MS) arm_now = arm_raw;

        if (!arm_now) seen_disarmed = true;

        bool on_pad = (fsm.state == SIMPLE_STATE_IDLE);

        if (on_pad && arm_now && !arm_prev && seen_disarmed) {
            pyro_clear_fired(&pyros);
            float pack_v  = read_pack_voltage();
            bool  cont_ok = pyro_check_continuity(PIN_PYRO1_SENSE, pack_v);
            buzzer_set(&buzz, cont_ok ? BUZZ_ARMED : BUZZ_CONT_OPEN);
            Serial.print("[ARM] SW401 CLOSED — new flight armed. Continuity: ");
            Serial.println(cont_ok ? "OK" : "OPEN");
        } else if (on_pad && !arm_now && arm_prev) {
            buzzer_set(&buzz, BUZZ_IDLE);
            Serial.println("[ARM] SW401 OPEN — disarmed.");
        }
        arm_prev = arm_now;
    }

    // Serial commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'X') {
            pyro_safe_all(&pyros);
            Serial.println("[SAFE] Pyro safed via serial.");
        } else if (c == 'R') {
            logger_finalize();
        } else if (c == 'U') {
            logger_usb_dump();
        }
    }

    // ── SENSOR INGESTION ──────────────────────────────────────
    LSM6DSOX_Data imu_data;
    lsm6dsox_read(&imu_data, &gyro_bias);

    LPS22HB_Data baro_data;
    lps22hb_read(&baro_data);

    float accel_up_g    = -imu_data.ax;   // body +X toward tail; negate for "up"
    float accel_mag_g   = sqrtf(imu_data.ax*imu_data.ax +
                                 imu_data.ay*imu_data.ay +
                                 imu_data.az*imu_data.az);
    float gyro_rate_dps = sqrtf(imu_data.gx*imu_data.gx +
                                 imu_data.gy*imu_data.gy +
                                 imu_data.gz*imu_data.gz);

    float pressure_for_est = baro_data.valid ? (baro_data.pressure_pa / 100.0f) : NAN;
    alt_update(&alt_est, pressure_for_est, accel_up_g, dt);

    // ── STATE MACHINE ─────────────────────────────────────────
    simple_fsm_update(&fsm, &pyros, accel_up_g, accel_mag_g, gyro_rate_dps,
                      alt_est.altitude_m, now_ms);

    if (simple_fsm_state_changed(&fsm)) {
        switch (fsm.state) {
            case SIMPLE_STATE_LANDED:
                buzzer_set(&buzz, BUZZ_LOCATOR);
                logger_finalize();
                Serial.println("[INFO] Landed. Flight log written.");
                break;
            default:
                break;
        }
    }

    // ── HOUSEKEEPING ──────────────────────────────────────────
    pyro_update(&pyros);
    buzzer_update(&buzz);

    // ── LOGGING ───────────────────────────────────────────────
    // Stop once landed — see the matching note in main_control_loop.cpp:
    // otherwise the ring buffer keeps recording ground noise and
    // overwrites the actual flight data within ~32 s if there's no SD
    // card and nobody's retrieved it yet.
    if (fsm.state == SIMPLE_STATE_LANDED) return;

    LogRecord rec = {};
    rec.timestamp_ms = now_ms;

    rec.gx = imu_data.gx; rec.gy = imu_data.gy; rec.gz = imu_data.gz;
    rec.ax = imu_data.ax; rec.ay = imu_data.ay; rec.az = imu_data.az;

    rec.temperature_c = baro_data.valid ? baro_data.temperature_c : NAN;
    rec.pressure_hpa  = baro_data.valid ? (baro_data.pressure_pa / 100.0f) : NAN;
    rec.altitude_m    = alt_est.altitude_m;
    rec.velocity_ms   = alt_est.velocity_ms;

    rec.flight_state = (uint8_t)fsm.state;
    rec.imu_valid    = imu_data.valid;
    rec.baro_valid   = baro_data.valid;
    rec.mag_valid    = false;

    logger_write(&rec);
}
