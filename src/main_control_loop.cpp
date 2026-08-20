#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Watchdog_t4.h"

#include "board_pins.h"
#include "lsm6dsox.h"
#include "lps22hb.h"
#include "mag.h"
#include "alt_estimator.h"
#include "flight_sm.h"
#include "pid.h"
#include "servo_driver.h"
#include "pyro.h"
#include "indicator.h"
#include "buzzer.h"

extern "C" const buzzer_hal_t BUZZER_HAL_TEENSY;
#include "logger.h"

// Forward declarations for mahrs_integration.cpp
struct RocketAttitude { float tip_a; float tip_b; float spin; };
void mahrs_init();
void mahrs_set_phase(FlightState phase);
void mahrs_tick(const LSM6DSOX_Data *imu, const mag_data *mag);
void mahrs_get_attitude(RocketAttitude *out);
void mahrs_get_quaternion(float *q0, float *q1, float *q2, float *q3);

// --- WATCHDOG ---
static WDT_T4<WDT3> wdt;

// --- GLOBAL MODULE CONTEXTS ---
static GyroBias       gyro_bias;
static AltEstimator   alt_est;
static FlightSM       fsm;
static PIDController  pid_pitch;
static PIDController  pid_yaw;
static PyroState      pyros;
static IndicatorState indicator;
static buzzer_t       buzz;

// --- TIMING ---
const uint32_t LOOP_INTERVAL_US = 8000;  // 125 Hz
uint32_t last_loop_time = 0;

// --- ARM SENSE DEBOUNCE ---
// Read PIN_ARM_SENSE via 12-bit analogRead.
// Threshold 1614 ≈ 1.3 V on a 3.3 V reference.
static bool arm_sense_stable  = false;
static bool arm_sense_raw     = false;
static uint32_t arm_edge_ms   = 0;

static inline bool read_arm_sense() {
    return analogRead(PIN_ARM_SENSE) >= ARM_SENSE_THRESHOLD;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // 1. ANALOG + PIN SETUP
    analogReadResolution(12);   // 12-bit ADC for ARM_SENSE
    pinMode(PIN_ARM_SENSE, INPUT);

    // IMU CS must be HIGH before SPI.begin() (keeps CS deasserted during bus init)
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);

    SPI.begin();
    Wire.begin();
    delay(100);

    // 2. HARDWARE OUTPUTS
    indicator_init(&indicator);
    buzzer_init(&buzz, &BUZZER_HAL_TEENSY, PIN_BUZZER, BUZZER_FREQ_HZ);
    buzzer_set(&buzz, BUZZ_BOOT);
    pyro_init(&pyros);
    servo_init();

    // 3. LOGGER
    logger_init();

    // 4. SENSOR INIT
    if (!lps22hb_init()) {
        Serial.println("[WARN] LPS22HB not found — baro disabled");
    }

    if (!lsm6dsox_init()) {
        Serial.println("[FAULT] LSM6DSOX init failed — check SPI wiring");
        buzzer_set(&buzz, BUZZ_SELFTEST_FAIL);
        while (true) { buzzer_update(&buzz); delay(10); }
    }
    lsm6dsox_load_bias(&gyro_bias);

    // 5. ALTITUDE ESTIMATOR INIT
    // Take a ground pressure snapshot (baro must be initialised first).
    // If baro is absent, init with sea-level; in-flight bias refinement will
    // still work but altitude will be inaccurate.
    {
        LPS22HB_Data baro_ground;
        lps22hb_read(&baro_ground);
        float ground_hpa = baro_ground.valid ? (baro_ground.pressure_pa / 100.0f)
                                              : 1013.25f;
        alt_init(&alt_est, ground_hpa);
    }

    // 6. FILTER & FSM INIT
    mahrs_init();
    fsm_init(&fsm);
    pid_init(&pid_pitch);
    pid_init(&pid_yaw);

    // 7. WATCHDOG — 500 ms timeout; fed every loop iteration.
    // On watchdog reset, setup() runs again: pyro pins go LOW first via
    // pyro_init(), servos centre via servo_init(), FSM starts in IDLE.
    {
        WDT_timings_t wdt_cfg;
        wdt_cfg.timeout = 0.5f;   // 500 ms
        wdt.begin(wdt_cfg);
    }

    buzzer_set(&buzz, BUZZ_SELFTEST_PASS);
    Serial.println("FLIGHT COMPUTER READY. WAITING FOR ARM SWITCH.");
}

void loop() {
    // ── 0. TIMING ──────────────────────────────────────────────
    uint32_t now_us = micros();
    if (now_us - last_loop_time < LOOP_INTERVAL_US) return;

    float dt = (now_us - last_loop_time) / 1000000.0f;
    last_loop_time = now_us;

    wdt.feed();   // T11: pet the watchdog every loop

    uint32_t now_ms = millis();

    // ── 1. ARM SENSE (T3) ─────────────────────────────────────
    {
        bool raw = read_arm_sense();
        if (raw != arm_sense_raw) { arm_sense_raw = raw; arm_edge_ms = now_ms; }
        if ((now_ms - arm_edge_ms) >= 50 && raw != arm_sense_stable) {
            arm_sense_stable = raw;
            if (raw) {
                // Rising edge: attempt to arm
                alt_calibrate_finish(&alt_est);   // T8: lock in accel bias before flight
                if (fsm_arm(&fsm, &pyros, true)) {
                    logger_checkpoint(STATE_ARMED, alt_est.altitude_m);
                    buzzer_set(&buzz, BUZZ_ARMED);
                    Serial.println("[ARM] Armed.");
                } else {
                    Serial.println("[ARM] Arm rejected — not in IDLE.");
                }
            } else {
                // Falling edge: disarm
                fsm_disarm(&fsm, &pyros);
                logger_checkpoint(STATE_IDLE, alt_est.altitude_m);
                buzzer_set(&buzz, BUZZ_IDLE);
                Serial.println("[ARM] Disarmed.");
            }
        }
    }

    // Serial commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'D') {
            fsm_disarm(&fsm, &pyros);
            Serial.println("[ARM] Disarmed via serial.");
        } else if (c == 'X') {
            fsm_abort(&fsm);
            pyro_safe_all(&pyros);
            Serial.println("[ARM] Abort via serial.");
        } else if (c == 'G') {
            if (fsm.state != STATE_IDLE) {
                Serial.println("[CAL] Gyro cal only allowed in IDLE.");
            } else {
                if (lsm6dsox_calibrate_gyro(&gyro_bias)) {
                    lsm6dsox_save_bias(&gyro_bias);
                    Serial.println("[CAL] Gyro bias saved to EEPROM.");
                }
            }
        } else if (c == 'R') {
            // Manual dump trigger — normally logger_finalize() runs
            // automatically on STATE_LANDED; this covers bench testing
            // or forcing a dump before landing is detected.
            logger_finalize();
        }
    }

    // ── 2. SENSOR INGESTION ───────────────────────────────────
    LSM6DSOX_Data imu_data;
    lsm6dsox_read(&imu_data, &gyro_bias);

    // Baro: gated by P_DA in lps22hb_read(); returns NaN when no new sample
    LPS22HB_Data baro_data;
    lps22hb_read(&baro_data);

    mag_data no_mag = {};   // mag not fitted this flight

    // ── 3. STATE ESTIMATION ───────────────────────────────────
    float accel_up_g = -imu_data.ax;   // body +X toward tail; negate for "up"

    float accel_mag_g   = sqrtf(imu_data.ax*imu_data.ax +
                                 imu_data.ay*imu_data.ay +
                                 imu_data.az*imu_data.az);
    float gyro_rate_dps = sqrtf(imu_data.gx*imu_data.gx +
                                 imu_data.gy*imu_data.gy +
                                 imu_data.gz*imu_data.gz);

    // T8: accumulate accel calibration samples during IDLE
    if (fsm.state == STATE_IDLE) {
        alt_calibrate_sample(&alt_est, accel_up_g);
    }

    // T8: update altitude estimator every tick (NaN pressure = accel-only update)
    float pressure_for_est = baro_data.valid ? (baro_data.pressure_pa / 100.0f) : NAN;
    alt_update(&alt_est, pressure_for_est, accel_up_g, dt);

    mahrs_tick(&imu_data, &no_mag);

    RocketAttitude attitude;
    mahrs_get_attitude(&attitude);

    float q0, q1, q2, q3;
    mahrs_get_quaternion(&q0, &q1, &q2, &q3);

    // ── 4. FLIGHT STATE MACHINE ───────────────────────────────
    fsm_update(&fsm, accel_up_g, alt_est.velocity_ms,
               accel_mag_g, gyro_rate_dps, alt_est.altitude_m, imu_data.valid);

    if (fsm_state_changed(&fsm)) {
        mahrs_set_phase(fsm.state);
        logger_checkpoint(fsm.state, alt_est.altitude_m);

        switch (fsm.state) {
            case STATE_POWERED:
                pid_reset(&pid_pitch);
                pid_reset(&pid_yaw);
                break;
            case STATE_MAIN:
                // G5: pass real altitude from estimator (T8)
                pyro_fire_main(&pyros, alt_est.altitude_m);
                break;
            case STATE_LANDED:
                servo_disable();
                buzzer_set(&buzz, BUZZ_LOCATOR);
                logger_finalize();
                Serial.println("[INFO] Landed. Flight log written to SD.");
                break;
            case STATE_ABORT:
                pyro_safe_all(&pyros);
                servo_center();
                buzzer_off(&buzz);
                break;
            default:
                break;
        }
    }

    // ── 5. CONTROL & ACTUATION ────────────────────────────────
    float pitch_cmd = 0.0f;
    float yaw_cmd   = 0.0f;

    if (fsm.state == STATE_POWERED && fsm.tvc_enabled) {
        pitch_cmd = pid_update(&pid_pitch, 0.0f, attitude.tip_a, dt);
        yaw_cmd   = pid_update(&pid_yaw,   0.0f, attitude.tip_b, dt);
        servo_set_pitch(pitch_cmd);
        servo_set_yaw(yaw_cmd);
    } else if (fsm.state >= STATE_COAST) {
        servo_center();
    }

    // ── 6. HOUSEKEEPING ───────────────────────────────────────
    pyro_update(&pyros);
    indicator_update(&indicator, fsm.state);
    buzzer_update(&buzz);

    // ── 7. LOGGING ────────────────────────────────────────────
    LogRecord rec;

    rec.timestamp_ms = now_ms;

    rec.roll  = attitude.tip_b;
    rec.pitch = attitude.tip_a;
    rec.yaw   = attitude.spin;
    rec.q0    = q0; rec.q1 = q1; rec.q2 = q2; rec.q3 = q3;

    rec.gx = imu_data.gx; rec.gy = imu_data.gy; rec.gz = imu_data.gz;
    rec.ax = imu_data.ax; rec.ay = imu_data.ay; rec.az = imu_data.az;

    rec.mx = 0.0f; rec.my = 0.0f; rec.mz = 0.0f;  // mag not fitted

    rec.temperature_c = baro_data.valid ? baro_data.temperature_c : NAN;
    rec.pressure_hpa  = baro_data.valid ? (baro_data.pressure_pa / 100.0f) : NAN;
    rec.altitude_m    = alt_est.altitude_m;      // T8: real altitude
    rec.velocity_ms   = alt_est.velocity_ms;     // T8: estimator velocity

    rec.servo_pitch_us = servo_get_pitch_us();
    rec.servo_yaw_us   = servo_get_yaw_us();
    rec.pid_pitch_out  = pitch_cmd;
    rec.pid_yaw_out    = yaw_cmd;

    rec.flight_state = (uint8_t)fsm.state;
    rec.imu_valid    = imu_data.valid;
    rec.baro_valid   = baro_data.valid;   // T9: set correctly now that T7 is landed
    rec.mag_valid    = false;

    logger_write(&rec);
}
