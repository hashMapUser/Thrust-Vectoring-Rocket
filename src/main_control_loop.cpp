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
#include "telemetry.h"
#include "flight_resume.h"

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
// continuity-open buzzer feedback. See the arming-switch block in loop(),
// and section 6 for the buzzer, which follows the ARM_SENSE level.
//
// These only track pad-rest edges for the accel bias calibration:
// pad_still_prev — the pad-rest timer running (upright and still), which
// opens a fresh calibration window; pad_rest_prev — the latch itself, which
// locks the calibration in, at the same moment the old auto-arm step used to.
static bool pad_still_prev = false;
static bool pad_rest_prev  = false;

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
static bool     cont_open     = false;   // e-match read OPEN at the last arming edge — picks the buzzer pattern

// White LED heartbeat — blinks for as long as the flight firmware is
// running, so the build on the board is identifiable without a laptop.
#define HEARTBEAT_HALF_PERIOD_MS 500

static inline void heartbeat_update(uint32_t now_ms) {
    digitalWriteFast(PIN_LED_WHITE, ((now_ms / HEARTBEAT_HALF_PERIOD_MS) & 1) ? HIGH : LOW);
}

// Rising-edge tracker for fsm.tvc_enabled — see section 5 in loop().
static bool tvc_enabled_prev = false;

// --- BENCH TELEMETRY ---
// Tick counter for TLM_DIVIDER, and the last valid baro sample —
// lps22hb_read() returns NaN on ticks with no new data, so without
// holding the last value most telemetry frames would show no pressure.
static uint8_t  tlm_tick      = 0;
static uint32_t baro_last_ms  = 0;
static float    baro_last_hpa = NAN;
static float    baro_last_c   = NAN;

// --- WATCHDOG RECOVERY ---
// Last time the flight record was refreshed — see flight_resume.h.
static uint32_t resume_saved_ms = 0;

void setup() {
    // A watchdog reset in the air must get back to the control loop fast,
    // so the slow boot steps (USB wait, SD init, 1.3 s baro average) are
    // skipped when there's a flight to resume.
    bool         wdt_boot = resume_boot_was_watchdog();
    ResumeRecord resume;
    bool         resuming = wdt_boot && resume_load(&resume);

    Serial.begin(115200);
    if (!wdt_boot) {
        while (!Serial && millis() < 3000) {}
    }
    if (wdt_boot) {
        Serial.println(resuming ? "[WDT] Watchdog reset mid-flight — fast boot, resuming flight."
                                : "[WDT] Watchdog reset on the ground — normal boot.");
    }

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
    // here for safety (see pyro_init()'s own ordering requirement). The
    // buzzer stays silent until SW401 arms — boot status is shown on the
    // LEDs instead. servo_init() is deferred past sensor init — see step 3.
    indicator_init(&indicator);
    buzzer_init(&buzz, &BUZZER_HAL_TEENSY, PIN_BUZZER, BUZZER_FREQ_HZ);
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
    // White LED — flight-firmware heartbeat, see heartbeat_update().
    // Red LED — fast blink on a sensor-init fault.
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_GREEN, LOW);
    pinMode(PIN_LED_WHITE, OUTPUT);
    digitalWrite(PIN_LED_WHITE, LOW);
    pinMode(PIN_LED_RED, OUTPUT);
    digitalWrite(PIN_LED_RED, LOW);

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
        // In the air, carry on without it: the FSM flags the IMU fault,
        // drops TVC and still runs the baro/timeout recovery path.
        if (!resuming) while (true) {
            digitalWriteFast(PIN_LED_RED, ((millis() / 100) & 1) ? HIGH : LOW);
            delay(10);
        }
    }
    lsm6dsox_load_bias(&gyro_bias);

    servo_init();

    // 4. LOGGER — skipped on a resume: SD init can block for seconds on a
    // bad card. Logging still goes to RAM, and logger_finalize() falls
    // back to a USB dump.
    if (!resuming) logger_init();

    // 5. ALTITUDE ESTIMATOR INIT
    // Boot ground reference, averaged over ALT_GROUND_SAMPLES fresh baro
    // readings with outliers dropped (~1.33 s; runs before the watchdog
    // starts). This only covers the time before arming — the launch
    // baseline is re-captured on the SW401 arming edge in loop(). If baro
    // is absent, init with sea-level; in-flight bias refinement will still
    // work but altitude will be inaccurate.
    if (resuming) {
        // Same ground reference as before the reset; altitude starts at
        // the current baro reading so the apogee baro-drop check compares
        // against a real altitude, not zero.
        float samples_hpa[8];
        float now_hpa = NAN;
        if (!lps22hb_read_samples(8, samples_hpa) ||
            !alt_ground_mean(samples_hpa, 8, &now_hpa, nullptr, nullptr)) {
            now_hpa = NAN;
            Serial.println("[WARN] Baro read failed on resume — altitude starts at 0");
        }
        alt_resume(&alt_est, resume.ground_hpa, now_hpa);
    } else {
        float samples_hpa[ALT_GROUND_SAMPLES];
        float ground_hpa;
        if (!lps22hb_read_samples(ALT_GROUND_SAMPLES, samples_hpa) ||
            !alt_ground_mean(samples_hpa, ALT_GROUND_SAMPLES, &ground_hpa, nullptr, nullptr)) {
            Serial.println("[WARN] Baro ground reference failed — using sea-level, altitude unreliable");
            ground_hpa = ALT_SEA_LEVEL_HPA;
        }
        alt_init(&alt_est, ground_hpa);
    }

    // 6. FILTER & FSM INIT
    mahrs_init();
    fsm_init(&fsm);
    if (resuming) {
        // If the baro seed failed, altitude starts at 0 — don't compare it
        // against the old peak, or the baro-drop check fires immediately.
        float peak = isnan(alt_est.baro_altitude_m) || alt_est.altitude_m == 0.0f
                         ? 0.0f : fmaxf(resume.peak_altitude_m, alt_est.altitude_m);
        fsm_resume(&fsm, resume.state, resume.ms_since_launch, peak, resume.flight_proven);
    }
    pid_init(&pid_pitch);
    pid_init(&pid_yaw);

    // 7. WATCHDOG — 500 ms timeout; fed every loop iteration.
    // On watchdog reset, setup() runs again: pyro pins go LOW first via
    // pyro_init(), servos centre via servo_init(), and the FSM resumes the
    // flight if one was in progress (see flight_resume.h).
    {
        WDT_timings_t wdt_cfg;
        wdt_cfg.timeout = 500;   // ms — WDT3 (RTWDOG) takes milliseconds, not seconds
        wdt.begin(wdt_cfg);
    }
    logger_set_keepalive([] { wdt.feed(); });

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

    // ── 1. ARMING SWITCH (SW401) ───────────────────────────────
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
            cont_open = !cont_ok;
            Serial.print("[ARM] SW401 CLOSED — new flight armed. Continuity: ");
            Serial.print(cont_ok ? "OK" : "OPEN");
            Serial.print("  pack=");
            Serial.print(pack_v, 2);
            Serial.println(" V");

            // Launch baseline: averaged over the next ALT_GROUND_SAMPLES
            // baro readings, fed in section 3 — never blocking, the
            // watchdog is live.
            alt_ground_capture_start(&alt_est);
            Serial.println("[ALT] Capturing launch ground reference — hold still...");
        } else if (on_pad && !arm_now && arm_prev) {
            fsm_set_armed(&fsm, false);
            alt_ground_capture_cancel(&alt_est);
            Serial.println("[ARM] SW401 OPEN — disarmed.");
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
                // Calibration blocks for 4+ s; the keepalive feeds the
                // 500 ms watchdog each sample so it doesn't reset mid-run.
                if (lsm6dsox_calibrate_gyro(&gyro_bias, [] { wdt.feed(); })) {
                    lsm6dsox_save_bias(&gyro_bias);
                    Serial.println("[CAL] Gyro bias saved to EEPROM.");
                }
            }
        } else if (c == 'R') {
            // Manual dump trigger — normally logger_finalize() runs
            // automatically on STATE_LANDED; this covers bench testing
            // or forcing a dump before landing is detected.
            logger_finalize();
        } else if (c == 'T') {
            telemetry_set_enabled(true);
            Serial.println("[TLM] Stream on.");
        } else if (c == 't') {
            telemetry_set_enabled(false);
            Serial.println("[TLM] Stream off.");
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

    // T8: accel calibration samples — only while the pad-rest timer is
    // running (upright and still, armed or not). Each new still window
    // starts from scratch, so time spent being carried or lying flat never
    // reaches the average. Locked in at the latch, after fsm_update().
    {
        bool pad_still = (fsm.state == STATE_IDLE || fsm.state == STATE_ARMED) &&
                         fsm.pad_rest_start_ms != 0;
        if (pad_still && !pad_still_prev) alt_calibrate_reset(&alt_est);
        if (pad_still) alt_calibrate_sample(&alt_est, accel_up_g);
        pad_still_prev = pad_still;
    }

    float pressure_for_est = baro_data.valid ? (baro_data.pressure_pa / 100.0f) : NAN;

    // Launch ground reference capture, started on the SW401 arming edge.
    // Runs before alt_update() so the tick that completes it already
    // measures against the new reference. If launch detection starts
    // first, abandon it — an average spanning liftoff would be wrong.
    if (alt_est.ground_capturing) {
        bool on_pad = (fsm.state == STATE_IDLE || fsm.state == STATE_ARMED) &&
                      fsm.launch_detect_ms == 0;
        if (!on_pad) {
            alt_ground_capture_cancel(&alt_est);
            Serial.println("[ALT] Launch began before ground reference finished — keeping previous baseline.");
        } else {
            switch (alt_ground_capture_sample(&alt_est, pressure_for_est)) {
                case ALT_CAPTURE_DONE:
                    fsm_set_launch_baseline(&fsm, alt_est.altitude_m);
                    Serial.print("[ALT] Launch ground reference set: ");
                    Serial.print(alt_est.ground_pressure, 3);
                    Serial.print(" hPa (");
                    Serial.print(alt_est.ground_cap_used);
                    Serial.print(" of ");
                    Serial.print(ALT_GROUND_SAMPLES);
                    Serial.println(" samples, outliers dropped). Altitude zeroed.");
                    break;
                case ALT_CAPTURE_RESTARTED:
                    Serial.print("[ALT] Ground reference rejected — only ");
                    Serial.print(alt_est.ground_cap_used);
                    Serial.print(" of ");
                    Serial.print(ALT_GROUND_SAMPLES);
                    Serial.println(" samples near the median. Recapturing — hold still...");
                    break;
                case ALT_CAPTURE_PENDING:
                    break;
            }
        }
    }

    // T8: update altitude estimator every tick (NaN pressure = accel-only update)
    alt_update(&alt_est, pressure_for_est, accel_up_g, dt);

    mahrs_tick(&imu_data, &no_mag);

    RocketAttitude attitude;
    mahrs_get_attitude(&attitude);

    float q0, q1, q2, q3;
    mahrs_get_quaternion(&q0, &q1, &q2, &q3);

    // ── 4. FLIGHT STATE MACHINE ───────────────────────────────
    fsm_update(&fsm, accel_up_g, alt_est.velocity_ms,
               accel_mag_g, gyro_rate_dps, alt_est.altitude_m, imu_data.valid);

    // Accel bias lock, on the pad-rest latch — independent of the arming
    // switch; the vehicle just has to have sat still long enough. Runs
    // right after fsm_update() so the FSM's baseline snapshot from this
    // same tick can be moved onto the re-synced altitude.
    {
        bool pad_rest_now = (fsm.state == STATE_IDLE || fsm.state == STATE_ARMED) &&
                             fsm.pad_rest_satisfied;
        if (pad_rest_now && !pad_rest_prev) {
            if (alt_calibrate_finish(&alt_est)) {   // T8: lock in accel bias before flight
                fsm_set_pad_rest_baseline(&fsm, alt_est.altitude_m);
                Serial.print("[INIT] Pad-rest latched — accel bias locked in: ");
                Serial.print(alt_est.accel_bias_ms2, 3);
                Serial.println(" m/s^2. Velocity zeroed, altitude re-synced to baro.");
            } else {
                Serial.println("[INIT] Pad-rest latched — accel calibration rejected "
                               "(not upright/still); bias will be learned from the baro.");
            }
        }
        pad_rest_prev = pad_rest_now;
    }

    bool state_changed = fsm_state_changed(&fsm);

    // Refresh the flight record so a watchdog reset can pick up from here.
    if (state_changed || now_ms - resume_saved_ms >= RESUME_SAVE_INTERVAL_MS) {
        resume_saved_ms = now_ms;
        ResumeRecord r;
        r.state           = fsm.state;
        r.flight_proven   = fsm.peak_velocity_ms > MIN_FLIGHT_VELOCITY_MS;
        r.ms_since_launch = fsm.powered_entry_ms ? now_ms - fsm.powered_entry_ms : 0;
        r.ground_hpa      = alt_est.ground_pressure;
        r.peak_altitude_m = fsm.peak_altitude_m;
        resume_save(&r);
    }

    if (state_changed) {
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
                logger_finalize();
                Serial.println("[INFO] Landed. Flight log written to SD.");
                break;
            case STATE_ABORT:
                pyro_safe_all(&pyros);
                servo_center();
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

    // Buzzer follows ARM_SENSE and nothing else: high (SW401 closed, pyro
    // battery connected) → armed pattern, in every state, including a
    // switch already closed at boot; low → silent. After landing the armed
    // pattern becomes the locator. buzzer_set() is idempotent, so calling
    // it every tick doesn't restart the pattern.
    {
        buzzer_pattern_t want = BUZZ_SILENT;
        if (arm_now) {
            if (fsm.state == STATE_LANDED) want = BUZZ_LOCATOR;
            else                           want = cont_open ? BUZZ_CONT_OPEN : BUZZ_ARMED;
        }
        buzzer_set(&buzz, want);
    }
    buzzer_update(&buzz);
    heartbeat_update(now_ms);

    // ── 6b. BENCH TELEMETRY ───────────────────────────────────
    // One $TLM frame every TLM_DIVIDER ticks while the stream is on.
    // Sits before section 7's STATE_LANDED early return so the
    // dashboard keeps updating after touchdown. telemetry_emit() drops
    // a frame rather than block when the USB buffer is full.
    if (baro_data.valid) {
        baro_last_ms  = now_ms;
        baro_last_hpa = baro_data.pressure_pa / 100.0f;
        baro_last_c   = baro_data.temperature_c;
    }

    if (++tlm_tick >= TLM_DIVIDER) {
        tlm_tick = 0;
        if (telemetry_enabled()) {
            TelemetryFrame t;
            t.t_ms  = now_ms;
            t.state = (uint8_t)fsm.state;

            uint8_t flags = 0;
            if (imu_data.valid)                     flags |= TLM_F_IMU_VALID;
            if (baro_last_ms != 0 &&
                now_ms - baro_last_ms < TLM_BARO_STALE_MS) flags |= TLM_F_BARO_OK;
            if (fsm.tvc_enabled)                    flags |= TLM_F_TVC_LIVE;
            if (arm_now)                            flags |= TLM_F_ARM_SWITCH;
            if (fsm.pad_rest_satisfied)             flags |= TLM_F_PAD_REST;
            if (fsm.launch_detect_ms != 0)          flags |= TLM_F_LAUNCH_DET;
            if (alt_est.ground_capturing)           flags |= TLM_F_GROUND_CAP;
            if (pyros.main_fired)                   flags |= TLM_F_MAIN_FIRED;
            t.flags = flags;

            t.q0 = q0; t.q1 = q1; t.q2 = q2; t.q3 = q3;
            t.tip_a = attitude.tip_a;
            t.tip_b = attitude.tip_b;
            t.spin  = attitude.spin;

            t.gx = imu_data.gx; t.gy = imu_data.gy; t.gz = imu_data.gz;
            t.ax = imu_data.ax; t.ay = imu_data.ay; t.az = imu_data.az;

            t.alt_m      = alt_est.altitude_m;
            t.vel_ms     = alt_est.velocity_ms;
            t.baro_alt_m = alt_est.baro_altitude_m;
            t.press_hpa  = baro_last_hpa;
            t.temp_c     = baro_last_c;

            t.pid_p      = pitch_cmd;
            t.pid_y      = yaw_cmd;
            t.servo_p_us = servo_get_pitch_us();
            t.servo_y_us = servo_get_yaw_us();

            t.pack_v  = read_pack_voltage();
            t.loop_us = micros() - now_us;

            telemetry_emit(&t);
        }
    }

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
