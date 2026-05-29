#include <Arduino.h>
#include <SPI.h>

#include "lsm6dsox.h"
#include "mag.h"          // mag_data type — needed for mahrs_tick signature
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
const uint32_t LOOP_INTERVAL_US = 8000; // 125 Hz
uint32_t last_loop_time = 0;

void setup() {
    Serial.begin(115200);

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

    // 3. SENSOR INIT
    // IMU is required — halt with error pattern if it fails.
    if (!lsm6dsox_init()) {
        Serial.println("[FAULT] LSM6DSOX init failed — check SPI wiring (SCK hand-wired to pin 13)");
        while (true) {
            indicator_beep_error(&indicator);
            delay(2000);
        }
    }
    lsm6dsox_load_bias(&gyro_bias);

    // 4. FILTER & FSM INIT
    mahrs_init();
    fsm_init(&fsm);
    pid_init(&pid_pitch);
    pid_init(&pid_yaw);

    Serial.println("FLIGHT COMPUTER READY. WAITING FOR SWITCH.");
}

void loop() {
    // 0. FIXED TIMING CONTROL
    uint32_t now = micros();
    if (now - last_loop_time < LOOP_INTERVAL_US) return;

    float dt = (now - last_loop_time) / 1000000.0f;
    last_loop_time = now;

    // 1. INPUT HANDLING
    // --- RBF arming switch (debounced, 50 ms) ---
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
                    Serial.println("[ARM] Armed.");
                } else {
                    Serial.println("[ARM] Arm rejected — not in IDLE.");
                    indicator_beep_error(&indicator);
                }
            } else {
                fsm_disarm(&fsm, &pyros);
                Serial.println("[ARM] Disarmed.");
            }
        }
    }

    // --- Serial commands ---
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
                Serial.println("[CAL] Gyro calibration — hold perfectly still for ~2.5 s...");
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

    // 2. SENSOR INGESTION — IMU only (baro, mag not used this flight)
    LSM6DSOX_Data imu_data;
    lsm6dsox_read(&imu_data, &gyro_bias);

    // Dummy mag with valid=false — AHRS runs in 6-DOF mode
    mag_data no_mag = {};

    // 3. STATE ESTIMATION
    float accel_up_g = -imu_data.ax;  // body X points toward tail; negate for "up"

    // Accel vector magnitude and gyro rate magnitude for landed detection
    float accel_mag_g   = sqrtf(imu_data.ax*imu_data.ax +
                                 imu_data.ay*imu_data.ay +
                                 imu_data.az*imu_data.az);
    float gyro_rate_dps = sqrtf(imu_data.gx*imu_data.gx +
                                 imu_data.gy*imu_data.gy +
                                 imu_data.gz*imu_data.gz);

    // Vertical velocity integration — only during POWERED and COAST.
    // Gravity-corrected: subtract 1 g before converting to m/s².
    static float vertical_velocity_ms = 0.0f;
    if (fsm.state == STATE_POWERED || fsm.state == STATE_COAST) {
        vertical_velocity_ms += (accel_up_g - 1.0f) * 9.81f * dt;
    }

    mahrs_tick(&imu_data, &no_mag);

    RocketAttitude attitude;
    mahrs_get_attitude(&attitude);

    // 4. FLIGHT STATE MACHINE
    fsm_update(&fsm, accel_up_g, vertical_velocity_ms, accel_mag_g, gyro_rate_dps, imu_data.valid);

    if (fsm_state_changed(&fsm)) {
        mahrs_set_phase(fsm.state);

        switch (fsm.state) {
            case STATE_POWERED:
                // Reset integrator at motor ignition
                vertical_velocity_ms = 0.0f;
                pid_reset(&pid_pitch);
                pid_reset(&pid_yaw);
                break;
            case STATE_APOGEE:
                // Single-chute flight — fire main at apogee, no drogue.
                pyro_fire_main(&pyros);
                break;
            case STATE_LANDED:
                servo_disable();   // de-energise servos — stops buzzing and saves battery
                Serial.println("[INFO] Landed. Send 'W' to dump flight log to SD.");
                break;
            case STATE_ABORT:
                pyro_safe_all(&pyros);
                servo_center();
                break;
            default:
                break;
        }
    }

    // 5. CONTROL & ACTUATION
    if (fsm.state == STATE_POWERED && fsm.tvc_enabled) {
        float pitch_cmd = pid_update(&pid_pitch, 0.0f, attitude.tip_a, dt);
        float yaw_cmd   = pid_update(&pid_yaw,   0.0f, attitude.tip_b, dt);

        servo_set_pitch(pitch_cmd);
        servo_set_yaw(yaw_cmd);
    } else if (fsm.state >= STATE_COAST) {
        servo_center();
    }

    // 6. HOUSEKEEPING
    pyro_update(&pyros);
    indicator_update(&indicator, fsm.state);
}
