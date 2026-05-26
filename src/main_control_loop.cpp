#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// ==========================================
// 1. HARDWARE & SENSOR HEADERS
// ==========================================
#include "bmp390.h"      // Barometer
#include "lsm6dsox.h"    // IMU
#include "mag.h"         // Magnetometer (MMC5603NJ)
#include "mag_calib.h"   // Hard/Soft iron mag calibration

// ==========================================
// 2. ESTIMATION & STATE HEADERS
// ==========================================
#include "alt_estimator.h" // 1D Baro/Accel Complementary Filter
#include "flight_sm.h"     // State Machine

// Forward declarations for mahrs_integration.cpp 
struct RocketAttitude { float tip_a; float tip_b; float spin; };
void mahrs_init();
void mahrs_set_phase(FlightState phase);
void mahrs_tick(const LSM6DSOX_Data *imu, const mag_data *mag);
void mahrs_get_attitude(RocketAttitude *out);

// ==========================================
// 3. CONTROL & ACTUATION HEADERS
// ==========================================
#include "pid.h"           // PID Controllers
#include "servo_driver.h"  // TVC Servos
#include "pyro.h"          // Ejection Charges

// ==========================================
// 4. HOUSEKEEPING HEADERS
// ==========================================
#include "indicator.h"     // Buzzer & LED
#include "logger.h"        // SD Card & RAM Ring Buffer

// ==========================================
// GLOBAL STATE INSTANTIATIONS
// ==========================================

// These hold the memory state for modules across loop iterations.
BMP390_Calib   bmp_cal;
GyroBias       gyro_bias;
MagCalib       mag_cal;

AltEstimator   alt;
FlightSM       fsm;

PIDController  pid_pitch;
PIDController  pid_yaw;

PyroState      pyros;
IndicatorState indicator;


// --- Global Variables (Instantiated in setup) ---
const uint32_t LOOP_INTERVAL_US = 8000; // 125 Hz = 8000 microseconds
uint32_t last_loop_time = 0;

void setup() {
    // ==========================================
    // 1. SERIAL & BUS SETUP
    // ==========================================
    Serial.begin(115200);
    // Note: Do not put a `while(!Serial);` loop here for flight code!
    // If you fly without a USB cable connected, the rocket will freeze forever.

    // CRITICAL: The CS pin must be driven HIGH before SPI.begin() is called.
    // If it floats, the LSM6DSOX receives garbage and won't initialize properly. [cite: 563, 1121]
    pinMode(LSM6DSOX_CS_PIN, OUTPUT);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);
    SPI.begin();

    // ==========================================
    // 2. HARDWARE OUTPUTS & HOUSEKEEPING
    // ==========================================
    // Initialize these early so they are in a safe, known state (e.g., pyros LOW)
    indicator_init(&indicator);
    logger_init();
    pyro_init(&pyros);
    servo_init();

    // ==========================================
    // 3. SENSOR INITIALIZATION & CALIBRATION
    // ==========================================
    // The bmp390_init and mag_init functions handle their own Wire/Wire1 bus setups[cite: 760, 761, 1060].
    if (!bmp390_init(&bmp_cal)) {
        Serial.println("[ERROR] Barometer Init Failed!");
    }

    if (!lsm6dsox_init()) {
        Serial.println("[ERROR] IMU Init Failed!");
    }
    // Load Gyro bias from EEPROM to prevent attitude drift
    if (!lsm6dsox_load_bias(&gyro_bias)) {
        Serial.println("[WARN] No Gyro Bias in EEPROM. Using default (0.0).");
    }

    if (!mag_init()) {
        Serial.println("[ERROR] Magnetometer Init Failed!");
    }
    // Load Hard/Soft iron corrections from EEPROM
    if (!mag_load_calib(&mag_cal)) {
        Serial.println("[WARN] No Mag Calibration in EEPROM. Using defaults.");
    }

    // ==========================================
    // 4. BASELINE ENVIRONMENT CALIBRATION
    // ==========================================
    // We must poll the barometer to get the local atmospheric pressure on the launch pad.
    // This allows the altitude estimator to calculate Altitude Above Ground Level (AGL). [cite: 441]
    float ground_pressure_hpa = 1013.25f; // Fallback to standard sea level
    BMP390_Data baro_data;
    
    // Read a few times to flush out stale initial reads
    for (int i = 0; i < 10; i++) {
        bmp390_read(&bmp_cal, &baro_data);
        delay(20);
    }
    if (baro_data.valid) {
        ground_pressure_hpa = baro_data.pressure_pa / 100.0f;
        Serial.print("[INIT] Baseline Pad Pressure: ");
        Serial.print(ground_pressure_hpa);
        Serial.println(" hPa");
    }

    // ==========================================
    // 5. STATE, ESTIMATION, & CONTROL SETUP
    // ==========================================
    alt_init(&alt, ground_pressure_hpa);
    mahrs_init();
    fsm_init(&fsm);

    pid_init(&pid_pitch);
    pid_init(&pid_yaw);

    // Give time to clear the pad or send the arming command
    Serial.println("====================================");
    Serial.println(" FLIGHT COMPUTER READY. SEND 'A' TO ARM.");
    Serial.println("====================================");
}

void loop() {
    // ==========================================
    // 0. TIMING CONTROL (FIXED RATE)
    // ==========================================
    uint32_t now = micros();
    if (now - last_loop_time < LOOP_INTERVAL_US) {
        return; // Non-blocking wait for the next tick
    }
    // Calculate actual dt (should be ~0.008s)
    float dt = (now - last_loop_time) / 1000000.0f; 
    last_loop_time = now;

    // ==========================================
    // 1. SENSOR INGESTION
    // ==========================================
    BMP390_Data baro_data;
    bmp390_read(&bmp_cal, &baro_data);

    LSM6DSOX_Data imu_data;
    lsm6dsox_read(&imu_data, &gyro_bias);

    mag_data raw_mag, cal_mag;
    mag_read(&raw_mag);
    if (raw_mag.valid) {
        // Apply hard/soft iron calibration
        mag_apply_calib(&mag_cal, raw_mag.mag_x, raw_mag.mag_y, raw_mag.mag_z,
                        &cal_mag.mag_x, &cal_mag.mag_y, &cal_mag.mag_z);
        cal_mag.valid = true;
    } else {
        cal_mag.valid = false;
    }

    // ==========================================
    // 2. STATE ESTIMATION
    // ==========================================
    // Map the IMU's physical vertical axis to the "Positive = Up" convention.
    // (Assuming physical +X points toward the nozzle, making -X point up)
    float accel_up_g = -imu_data.ax; 
    
    // Altitude & Velocity estimation (convert Pa to hPa for the estimator)
    alt_update(&alt, baro_data.pressure_pa / 100.0f, accel_up_g, dt);

    // Orientation estimation (Madgwick Filter)
    mahrs_tick(&imu_data, &cal_mag);
    RocketAttitude attitude;
    mahrs_get_attitude(&attitude);

    // ==========================================
    // 3. FLIGHT STATE MACHINE (FSM)
    // ==========================================
    fsm_update(&fsm, accel_up_g, alt.altitude_m, alt.velocity_ms, 
               imu_data.valid, baro_data.valid);

    // Handle one-shot events on state transitions
    if (fsm_state_changed(&fsm)) {
        // Adjust filter gains based on flight phase (from mahrs_integration)
        mahrs_set_phase(fsm.state);
        
        // Log the state transition to the SD card checkpoint file
        logger_checkpoint(fsm.state, alt.altitude_m);

        // State-specific transition logic
        switch (fsm.state) {
            case STATE_POWERED:
                // Zero out the PID integrators right as thrust begins
                pid_reset(&pid_pitch);
                pid_reset(&pid_yaw);
                break;
            case STATE_APOGEE:
                pyro_fire_drogue(&pyros);
                break;
            case STATE_MAIN:
                pyro_fire_main(&pyros, alt.altitude_m);
                break;
            case STATE_ABORT:
                pyro_safe_all(&pyros);
                servo_center();
                break;
            default:
                break;
        }
    }

    // ==========================================
    // 4. CONTROL & ACTUATION
    // ==========================================
    if (fsm.state == STATE_POWERED && fsm.tvc_enabled) {
        // Target setpoint is 0.0 degrees (straight up)
        float pitch_cmd = pid_update(&pid_pitch, 0.0f, attitude.tip_a, dt);
        float yaw_cmd   = pid_update(&pid_yaw,   0.0f, attitude.tip_b, dt);

        servo_set_pitch(pitch_cmd);
        servo_set_yaw(yaw_cmd);
    } else if (fsm.state >= STATE_COAST) {
        // Lock servos to center once motor burns out to reduce drag/wobble
        servo_center(); 
    }

    // ==========================================
    // 5. HOUSEKEEPING & LOGGING
    // ==========================================
    // Manage pyro firing pulse durations
    pyro_update(&pyros);
    
    // Update Buzzer/LED sequences
    indicator_update(&indicator, fsm.state);

    // Populate and push telemetry to the high-speed RAM buffer
    LogRecord record;
    record.timestamp_ms   = millis();
    record.altitude_m     = alt.altitude_m;
    record.velocity_ms    = alt.velocity_ms;
    record.flight_state   = fsm.state;
    // ... [Populate remaining record fields from attitude, imu_data, etc.] ...
    
    logger_write(&record);
}