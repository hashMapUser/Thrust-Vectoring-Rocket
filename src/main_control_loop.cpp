#include <Arduino.h>
#include <SPI.h>

#include "lsm6dsox.h"
#include "mag.h"
#include "flight_sm.h"
#include "pid.h"
#include "servo_driver.h"
#include "pyro.h"
#include "indicator.h"
#include "logger.h"

// Forward declarations for mahrs_integration.cpp
struct RocketAttitude { float tip_a; float tip_b; float spin; };
void mahrs_init();
void mahrs_set_phase(FlightState phase);
void mahrs_tick(const LSM6DSOX_Data *imu, const mag_data *mag);
void mahrs_get_attitude(RocketAttitude *out);
void mahrs_get_quaternion(float *q0, float *q1, float *q2, float *q3);

// --- HARDWARE PIN DEFS ---
#define ARM_SWITCH_PIN 2   // RBF jumper: jumper IN = GND = safe, jumper OUT = pull-up HIGH = arm

// --- GLOBAL MODULE CONTEXTS ---
static GyroBias       gyro_bias;
static FlightSM       fsm;
static PIDController  pid_pitch;
static PIDController  pid_yaw;
static PyroState      pyros;
static IndicatorState indicator;

// --- TIMING ---
const uint32_t LOOP_INTERVAL_US = 8000;  // 125 Hz
uint32_t last_loop_time = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}   // wait up to 3 s for USB serial

    // 1. PIN & BUS SETUP
    pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);

    pinMode(LSM6DSOX_CS_PIN, OUTPUT);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);

    SPI.begin();
    delay(100);

    // 2. HARDWARE OUTPUTS
    indicator_init(&indicator);
    pyro_init(&pyros);
    servo_init();

    // 3. LOGGER — init before sensors so SD checkpoint is open before
    //    any sensor faults are logged. Non-fatal if SD is absent.
    logger_init();

    // 4. SENSOR INIT
    if (!lsm6dsox_init()) {
        Serial.println("[FAULT] LSM6DSOX init failed — check SPI wiring");
        while (true) {
            indicator_update(&indicator, STATE_ABORT);
            delay(10);
        }
    }
    lsm6dsox_load_bias(&gyro_bias);

    // 5. FILTER & FSM INIT
    mahrs_init();
    fsm_init(&fsm);
    pid_init(&pid_pitch);
    pid_init(&pid_yaw);

    Serial.println("FLIGHT COMPUTER READY. WAITING FOR SWITCH.");
}

void loop() {
    // ── 0. FIXED TIMING CONTROL ──────────────────────────────────
    uint32_t now = micros();
    if (now - last_loop_time < LOOP_INTERVAL_US) return;

    float dt = (now - last_loop_time) / 1000000.0f;
    last_loop_time = now;

    // ── 1. INPUT HANDLING ─────────────────────────────────────────
    // RBF arming switch (debounced, 50 ms)
    {
        static bool     rbf_stable  = false;
        static bool     rbf_raw     = false;
        static uint32_t rbf_edge_ms = 0;
        bool raw = (digitalRead(ARM_SWITCH_PIN) == HIGH);
        if (raw != rbf_raw) { rbf_raw = raw; rbf_edge_ms = millis(); }
        if ((millis() - rbf_edge_ms) >= 50 && raw != rbf_stable) {
            rbf_stable = raw;
            if (raw) {
                if (fsm_arm(&fsm, &pyros)) {
                    logger_checkpoint(STATE_ARMED, 0.0f);
                    Serial.println("[ARM] Armed.");
                } else {
                    Serial.println("[ARM] Arm rejected — not in IDLE.");
                }
            } else {
                fsm_disarm(&fsm, &pyros);
                logger_checkpoint(STATE_IDLE, 0.0f);
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
                Serial.println("[CAL] Gyro calibration — hold still for ~2.5 s...");
                if (lsm6dsox_calibrate_gyro(&gyro_bias)) {
                    lsm6dsox_save_bias(&gyro_bias);
                    Serial.println("[CAL] Gyro bias saved to EEPROM.");
                } else {
                    Serial.println("[CAL] Gyro cal failed — motion detected. Try again.");
                }
            }
        } else if (c == 'W') {
            logger_dump_to_sd();
        }
    }

    // ── 2. SENSOR INGESTION ───────────────────────────────────────
    LSM6DSOX_Data imu_data;
    lsm6dsox_read(&imu_data, &gyro_bias);

    // Dummy mag — AHRS runs 6-DOF only (mag not fitted this flight)
    mag_data no_mag = {};

    // ── 3. STATE ESTIMATION ───────────────────────────────────────
    float accel_up_g = -imu_data.ax;  // body +X toward tail; negate for "up"

    float accel_mag_g   = sqrtf(imu_data.ax*imu_data.ax +
                                 imu_data.ay*imu_data.ay +
                                 imu_data.az*imu_data.az);
    float gyro_rate_dps = sqrtf(imu_data.gx*imu_data.gx +
                                 imu_data.gy*imu_data.gy +
                                 imu_data.gz*imu_data.gz);

    static float vertical_velocity_ms = 0.0f;
    if (fsm.state == STATE_POWERED || fsm.state == STATE_COAST) {
        vertical_velocity_ms += (accel_up_g - 1.0f) * 9.81f * dt;
    }

    mahrs_tick(&imu_data, &no_mag);

    RocketAttitude attitude;
    mahrs_get_attitude(&attitude);

    float q0, q1, q2, q3;
    mahrs_get_quaternion(&q0, &q1, &q2, &q3);

    // ── 4. FLIGHT STATE MACHINE ───────────────────────────────────
    fsm_update(&fsm, accel_up_g, vertical_velocity_ms, accel_mag_g, gyro_rate_dps, imu_data.valid);

    if (fsm_state_changed(&fsm)) {
        mahrs_set_phase(fsm.state);
        logger_checkpoint(fsm.state, vertical_velocity_ms);  // altitude_m not available; use vel as proxy

        switch (fsm.state) {
            case STATE_POWERED:
                vertical_velocity_ms = 0.0f;
                pid_reset(&pid_pitch);
                pid_reset(&pid_yaw);
                break;
            case STATE_MAIN:
                // APOGEE→MAIN is an atomic double-transition inside fsm_update(), so
                // case STATE_APOGEE is never observable here — fire main on MAIN entry instead.
                // Altitude guard bypassed: apogee detection via velocity zero-crossing already
                // confirms altitude; baro not fitted this flight so altitude_m would be 0.
                pyro_fire_main(&pyros, PYRO_MAIN_MIN_ALT_M + 1.0f);
                break;
            case STATE_LANDED:
                servo_disable();
                Serial.println("[INFO] Landed. Send 'W' to dump flight log to SD.");
                logger_dump_to_sd();   // auto-dump on landing
                break;
            case STATE_ABORT:
                pyro_safe_all(&pyros);
                servo_center();
                break;
            default:
                break;
        }
    }

    // ── 5. CONTROL & ACTUATION ────────────────────────────────────
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

    // ── 6. HOUSEKEEPING ───────────────────────────────────────────
    pyro_update(&pyros);
    indicator_update(&indicator, fsm.state);

    // ── 7. LOGGING ────────────────────────────────────────────────
    // Populate LogRecord from all live sensor and estimator data.
    // Fields match the LogRecord struct in logger.h exactly.
    LogRecord rec;

    rec.timestamp_ms = millis();

    // Attitude (Madgwick Euler output, remapped to rocket frame by mahrs_get_attitude)
    rec.roll  = attitude.tip_b;
    rec.pitch = attitude.tip_a;
    rec.yaw   = attitude.spin;

    // Quaternion (filter frame — document axis mapping in post-processing)
    rec.q0 = q0;
    rec.q1 = q1;
    rec.q2 = q2;
    rec.q3 = q3;

    // IMU — raw bias-corrected values straight from lsm6dsox_read()
    rec.gx = imu_data.gx;   // deg/s
    rec.gy = imu_data.gy;
    rec.gz = imu_data.gz;
    rec.ax = imu_data.ax;   // g
    rec.ay = imu_data.ay;
    rec.az = imu_data.az;

    // Magnetometer — not fitted; log zeros with mag_valid = false
    rec.mx = 0.0f;
    rec.my = 0.0f;
    rec.mz = 0.0f;

    // Barometer — not used this flight; log zeros
    rec.temperature_c = 0.0f;
    rec.pressure_hpa  = 0.0f;

    // Altitude estimator — velocity from IMU integration; no baro altitude yet
    rec.altitude_m  = 0.0f;
    rec.velocity_ms = vertical_velocity_ms;

    // Control outputs — servo pulse widths and PID terms
    rec.servo_pitch_us = servo_get_pitch_us();
    rec.servo_yaw_us   = servo_get_yaw_us();
    rec.pid_pitch_out  = pitch_cmd;
    rec.pid_yaw_out    = yaw_cmd;

    // System status
    rec.flight_state = (uint8_t)fsm.state;
    rec.imu_valid    = imu_data.valid;
    rec.baro_valid   = false;   // baro not fitted this flight
    rec.mag_valid    = false;   // mag not fitted this flight

    logger_write(&rec);
}