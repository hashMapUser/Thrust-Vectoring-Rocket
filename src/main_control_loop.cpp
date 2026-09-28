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

// --- ARMING ---
// pyro_arm() is still called unconditionally in setup() — SW401 is a
// hardware interlock that physically cuts PYRO PWR, so software arming
// state can never be the thing that blocks a deployment. What SW401
// DOES drive in software: clearing the EEPROM "fired" flags for a new
// flight, the STATE_IDLE<->STATE_ARMED display state, and the ARMED /
// continuity-open buzzer feedback. See the arming-switch block in loop().
//
// This flag only tracks the pad-rest rising edge, to lock in the accel
// bias calibration at the same moment the old auto-arm step used to.
static bool pad_rest_prev = false;

// ARM_SENSE divider: 10K/4.7K, ratio 0.3197. The pyro pack voltage isn't
// sensed directly on this board, but the ARM_SENSE pin gives it indirectly.
#define ARM_SENSE_DIVIDER_RATIO 0.3197f

static inline float read_arm_sense_v() {
    return analogRead(PIN_ARM_SENSE) * 3.30f / 4095.0f;
}

static inline float read_pack_voltage() {
    return read_arm_sense_v() / ARM_SENSE_DIVIDER_RATIO;
}

// SW401 debounce + edge tracking. ~2.69 V at ARM_SENSE when armed (2S
// pack), 0 V when safe — 1.2 V sits comfortably between the two.
#define ARM_SENSE_ARMED_V   1.2f
#define ARM_DEBOUNCE_MS     100

static bool     arm_raw_prev  = false;
static uint32_t arm_edge_ms   = 0;
static bool     arm_now       = false;
static bool     arm_prev      = false;
static bool     seen_disarmed = false;   // latches once SW401 has read OFF since boot
static uint32_t boot_ms       = 0;

// Rising-edge tracker for fsm.tvc_enabled — see section 5 in loop().
static bool tvc_enabled_prev = false;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // 1. ANALOG + PIN SETUP
    analogReadResolution(12);   // 12-bit ADC for ARM_SENSE, PYRO1_SENSE
    pinMode(PIN_ARM_SENSE, INPUT);
    pinMode(PIN_PYRO1_SENSE, INPUT);   // main-chute continuity sense; channel 2 unused this flight

    // IMU CS must be HIGH before SPI.begin() (keeps CS deasserted during bus init)
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);

    SPI.begin();
    Wire.begin();
    delay(100);

    // 2. HARDWARE OUTPUTS — indicator/buzzer/pyro only. Pyro pins go LOW
    // here for safety (see pyro_init()'s own ordering requirement), and the
    // buzzer needs to be ready before the sensor-init fail path below can
    // use it. servo_init() is deferred past sensor init — see step 3.
    indicator_init(&indicator);
    buzzer_init(&buzz, &BUZZER_HAL_TEENSY, PIN_BUZZER, BUZZER_FREQ_HZ);
    buzzer_set(&buzz, BUZZ_BOOT);
    pyro_init(&pyros);

    // pyro_arm() sets the software armed flags unconditionally — SW401 is
    // the real interlock (it cuts PYRO PWR), not this. Continuity is only
    // meaningful once PYRO PWR is actually present, so it's checked at the
    // SW401 arming edge in loop(), not here — checking it at boot with the
    // switch open would always read OPEN and tell you nothing.
    pyro_arm(&pyros);
    {
        bool arm_raw_boot = read_arm_sense_v() > ARM_SENSE_ARMED_V;
        Serial.print("[PYRO] Armed (software). SW401 reads: ");
        Serial.println(arm_raw_boot ? "CLOSED (armed)" : "OPEN (safe)");
        if (arm_raw_boot) {
            Serial.println("[ARM] SW401 already closed at boot — likely a mid-flight reset. "
                            "Fired flags kept; no continuity check yet.");
        }
    }

    // Green LED — bench aid, flashed by flight_sm.cpp on pad-rest latch.
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_GREEN, LOW);

    // 3. SENSOR INIT — before servo_init() attaches and centers the servos.
    // That draws a current spike, and lsm6dsox_init() is a one-shot
    // WHO_AM_I check with no retry — if the spike sags the rail during
    // that window, boot fails hard with no recovery. Sense first, then
    // bring up the actuator that competes for power.
    if (!lps22hb_init()) {
        Serial.println("[WARN] LPS22HB not found — baro disabled");
    }

    if (!lsm6dsox_init()) {
        Serial.println("[FAULT] LSM6DSOX init failed — check SPI wiring");
        buzzer_set(&buzz, BUZZ_SELFTEST_FAIL);
        while (true) { buzzer_update(&buzz); delay(10); }
    }
    lsm6dsox_load_bias(&gyro_bias);

    servo_init();

    // 4. LOGGER
    logger_init();

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
    boot_ms = millis();
    Serial.println("FLIGHT COMPUTER READY. PYRO ARMED. WAITING FOR LAUNCH.");
}

void loop() {
    // ── 0. TIMING ──────────────────────────────────────────────
    uint32_t now_us = micros();
    if (now_us - last_loop_time < LOOP_INTERVAL_US) return;

    float dt = (now_us - last_loop_time) / 1000000.0f;
    last_loop_time = now_us;

    wdt.feed();   // T11: pet the watchdog every loop

    uint32_t now_ms = millis();

    // ── 1. ACCEL BIAS LOCK ─────────────────────────────────────
    // Locks in the accel calibration bias the moment pad-rest first
    // latches, independent of the arming switch — this just needs the
    // vehicle to have sat still long enough, whether or not it's armed.
    {
        bool pad_rest_now = (fsm.state == STATE_IDLE || fsm.state == STATE_ARMED) &&
                             fsm.pad_rest_satisfied;
        if (pad_rest_now && !pad_rest_prev) {
            alt_calibrate_finish(&alt_est);   // T8: lock in accel bias before flight
            Serial.println("[INIT] Pad-rest latched — accel bias locked in.");
        }
        pad_rest_prev = pad_rest_now;
    }

    // ── 1b. ARMING SWITCH (SW401) ──────────────────────────────
    // The switch is the real safety interlock (it cuts PYRO PWR); this
    // block only ever REPORTS its state and drives EEPROM/display/buzzer
    // side effects — it must never gate whether a pyro can fire. Debounced
    // ~100 ms; threshold sits between 0 V (safe) and ~2.69 V (armed, 2S pack).
    {
        bool arm_raw = read_arm_sense_v() > ARM_SENSE_ARMED_V;
        if (arm_raw != arm_raw_prev) { arm_edge_ms = now_ms; arm_raw_prev = arm_raw; }
        if ((now_ms - arm_edge_ms) >= ARM_DEBOUNCE_MS) arm_now = arm_raw;

        if (!arm_now) seen_disarmed = true;

        // "On the pad" = launch hasn't even started latching. Once
        // launch_detect_ms is running (or later), SW401 transitions are
        // logged implicitly (ignored) by simply not matching this gate.
        bool on_pad = (fsm.state == STATE_IDLE || fsm.state == STATE_ARMED) &&
                      fsm.launch_detect_ms == 0;

        if (on_pad && arm_now && !arm_prev && seen_disarmed) {
            // Genuine new-flight arming edge (not "switch already on at
            // boot", which is the mid-flight-reset case — that must NOT
            // clear the fired flags; see the seen_disarmed gate).
            pyro_clear_fired(&pyros);
            float pack_v  = read_pack_voltage();
            bool  cont_ok = pyro_check_continuity(PIN_PYRO1_SENSE, pack_v);
            fsm_set_armed(&fsm, true);
            buzzer_set(&buzz, cont_ok ? BUZZ_ARMED : BUZZ_CONT_OPEN);
            Serial.print("[ARM] SW401 CLOSED — new flight armed. Continuity: ");
            Serial.print(cont_ok ? "OK" : "OPEN");
            Serial.print("  pack=");
            Serial.print(pack_v, 2);
            Serial.println(" V");
        } else if (on_pad && !arm_now && arm_prev) {
            fsm_set_armed(&fsm, false);
            buzzer_set(&buzz, BUZZ_IDLE);
            Serial.println("[ARM] SW401 OPEN — disarmed.");
        } else if (fsm.state == STATE_IDLE && now_ms - boot_ms > 600 &&
                   buzz.pattern != BUZZ_IDLE && buzz.pattern != BUZZ_ARMED &&
                   buzz.pattern != BUZZ_CONT_OPEN) {
            // First settle after the boot self-test chirps finish: default
            // to the disarmed/alive pattern rather than staying silent.
            buzzer_set(&buzz, BUZZ_IDLE);
        }
        arm_prev = arm_now;
    }

    // Serial commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'X') {
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
        } else if (c == 'U') {
            // USB CSV dump, independent of logger_finalize()'s idempotence.
            // The auto-finalize at STATE_LANDED fires the moment it's
            // reached, whether or not a USB cable is plugged in at that
            // exact instant — if it wasn't, that dump went nowhere. This
            // re-streams the RAM buffer on demand after recovery, any
            // number of times, as long as power hasn't been lost.
            logger_usb_dump();
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

    // T8: accumulate accel calibration samples while on the pad (armed or not)
    if (fsm.state == STATE_IDLE || fsm.state == STATE_ARMED) {
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
            // No PID reset here — that now happens on fsm.tvc_enabled's
            // rising edge in section 5 below, which fires at the initial
            // launch-accel trigger (i.e. before STATE_POWERED is even
            // entered — see item 3 in the firmware review).
            case STATE_MAIN:
                // First attempt on entry; section 6 retries every loop
                // until it actually fires (pyro_fire_main() can decline —
                // e.g. not armed — and this is a one-shot switch block).
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
    // Gated on fsm.tvc_enabled, not on fsm.state == STATE_POWERED — the
    // FSM now sets tvc_enabled true at the initial launch-accel trigger,
    // before liftoff is even confirmed, so the airframe is steered through
    // the slowest / least aerodynamically damped part of the flight too
    // (see item 3 in the firmware review). A rising edge on tvc_enabled
    // resets both PID controllers so a stale integral from a prior false
    // trigger never carries into a real one.
    float pitch_cmd = 0.0f;
    float yaw_cmd   = 0.0f;

    if (fsm.tvc_enabled && !tvc_enabled_prev) {
        pid_reset(&pid_pitch);
        pid_reset(&pid_yaw);
    }
    tvc_enabled_prev = fsm.tvc_enabled;

    if (fsm.tvc_enabled) {
        pitch_cmd = pid_update(&pid_pitch, 0.0f, attitude.tip_a, dt);
        yaw_cmd   = pid_update(&pid_yaw,   0.0f, attitude.tip_b, dt);
        servo_set_pitch(pitch_cmd);
        servo_set_yaw(yaw_cmd);
    } else {
        servo_center();
    }

    // ── 6. HOUSEKEEPING ───────────────────────────────────────
    // Retry the main charge every loop until it actually fires — it's
    // only called once on state entry above, and pyro_fire_main() can
    // legitimately decline once (e.g. transient not-armed) with nothing
    // else that would ever ask again.
    if (fsm.state == STATE_MAIN && !pyros.main_fired) {
        pyro_fire_main(&pyros, alt_est.altitude_m);
    }
    pyro_update(&pyros);
    indicator_update(&indicator, fsm.state);
    buzzer_update(&buzz);

    // ── 7. LOGGING ────────────────────────────────────────────
    // Stop once landed. logger_write() runs unconditionally otherwise, so
    // the ring buffer would keep recording nothing but ground noise after
    // touchdown — with no SD card, that overwrites the actual flight data
    // within LOG_RAM_CAPACITY/125Hz (~32 s) if nobody's retrieved it yet.
    // STATE_ABORT keeps logging — that's when you want the most data, not
    // the least, and it doesn't wrap into STATE_LANDED on its own.
    if (fsm.state == STATE_LANDED) return;

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
