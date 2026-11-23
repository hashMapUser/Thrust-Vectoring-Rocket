// ============================================================
//  sim_hil.cpp — bench simulation of the flight firmware
//
//  Stands in for the LSM6DSOX, LPS22HB and MMC5603NJ drivers (and the
//  magnetometer calibration store) in env:flight_sim, feeding readings
//  from the simulated rocket in sim_model.cpp. Runs the scenario menu and
//  the launch countdown, opens the simulated chute when the firmware fires
//  the main, and grades each detected event in a report. See sim_hil.h.
//
//  Compiled only with -D SIM_MODE.
// ============================================================
#ifdef SIM_MODE

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include "sim_hil.h"
#include "sim_model.h"
#include "lsm6dsox.h"
#include "lps22hb.h"
#include "mag.h"
#include "mag_calib.h"
#include "flight_resume.h"

// Run record for the watchdog-reset scenario: written just before the
// simulated hang, read and cleared on the reboot. Its own EEPROM slot,
// apart from the flight firmware's (gyro 10, pyro 30, mag 40; sim pyro 230).
#define SIM_EEPROM_ADDR  240
#define SIM_REC_MAGIC    0x51A7u

typedef struct {
    uint16_t magic;
    uint8_t  scenario;
    uint8_t  armed;
    uint32_t reset_ms;      // ms after ignition when the firmware hung
    float    powered_s;     // when liftoff was confirmed, for the report
} SimResetRecord;

#define WDT_TIMEOUT_S    0.5     // main_control_loop.cpp's watchdog
#define BUMP_REPORT_S    12.0
#define COUNTDOWN_S      5

// ------------------------------------------------------------
// Session state
// ------------------------------------------------------------
static SimModel g_model;
static bool     g_begun        = false;
static uint8_t  g_sc           = 0;       // scenario index
static bool     g_launch_set   = false;   // L accepted; g_ign_us is ignition
static uint64_t g_ign_us       = 0;
static bool     g_continued    = false;   // this boot carries on after the simulated hang
static double   g_cont_off_s   = 0.0;
static bool     g_pad_rest     = false;
static int      g_countdown    = -1;
static bool     g_ignited_msg  = false;
static bool     g_was_armed    = false;
static bool     g_deployed     = false;
static bool     g_reported     = false;
static double   g_next_wait_msg = 40.0;

static struct {
    double t_state[9];          // first entry into each FlightState, s after ignition
    double t_main;              // main charge fired
    double t_imu_fault;
    int    triggers;            // launch-accel latches
    int    false_triggers;      // latches released without liftoff
    bool   trig_prev;
    bool   resumed;
    bool   tvc_after_fault;     // TVC still on 0.1 s after the IMU fault
    bool   landed_airborne;     // LANDED while the simulated rocket was still in the air
    float  apogee_alt;          // altitude estimate when apogee was called
    FlightState last_state;
} g_ev;

static uint64_t micros64() {
    static uint32_t last = 0;
    static uint64_t high = 0;
    uint32_t now = micros();
    if (now < last) high += 0x100000000ULL;
    last = now;
    return high | now;
}

// Seconds after ignition; far negative before L.
static double sim_t() {
    if (g_continued) return g_cont_off_s + (double)micros64() / 1e6;
    if (!g_launch_set) return -1000.0;
    return ((double)micros64() - (double)g_ign_us) / 1e6;
}

static void reset_events() {
    for (int i = 0; i < 9; i++) g_ev.t_state[i] = NAN;
    g_ev.t_main          = NAN;
    g_ev.t_imu_fault     = NAN;
    g_ev.triggers        = 0;
    g_ev.false_triggers  = 0;
    g_ev.trig_prev       = false;
    g_ev.resumed         = false;
    g_ev.tvc_after_fault = false;
    g_ev.landed_airborne = false;
    g_ev.apogee_alt      = NAN;
    g_ev.last_state      = STATE_IDLE;
}

static void select_scenario(uint8_t i) {
    g_sc = i;
    sim_model_init(&g_model, &SIM_SCENARIOS[g_sc], 1234u + g_sc);
}

// First call (from the first driver setup() touches): pick up a run that
// the simulated hang interrupted, or start fresh on scenario 1.
static void sim_begin() {
    if (g_begun) return;
    g_begun = true;
    Serial.println("[SIM] Bench simulation build of " __DATE__ " " __TIME__);
    reset_events();
    select_scenario(0);

    SimResetRecord r;
    EEPROM.get(SIM_EEPROM_ADDR, r);
    if (r.magic != SIM_REC_MAGIC) return;
    r.magic = 0;
    EEPROM.put(SIM_EEPROM_ADDR, r);        // one continuation only

    // Only a real watchdog reset leaves the flight's resume record in the
    // SNVS registers — a power cycle clears them.
    ResumeRecord flight;
    if (r.scenario >= SIM_SCENARIO_COUNT || !resume_load(&flight)) return;

    select_scenario(r.scenario);
    g_continued   = true;
    g_launch_set  = true;
    g_ignited_msg = true;
    g_was_armed   = r.armed;
    g_cont_off_s  = r.reset_ms / 1000.0 + WDT_TIMEOUT_S;
    g_ev.t_state[STATE_POWERED] = r.powered_s;
    g_ev.triggers = 1;
}

// ------------------------------------------------------------
// Menu and commands
// ------------------------------------------------------------
static void print_menu() {
    Serial.println();
    Serial.println("================ BENCH SIMULATION (env:flight_sim) ================");
    Serial.println(" Real flight code, simulated sensors. Pyro outputs are NEVER driven.");
    Serial.println(" Servos DO move — keep clear of the gimbal.");
    Serial.println(" Scenarios:");
    for (int i = 0; i < SIM_SCENARIO_COUNT; i++)
        Serial.printf("  %d  %-15s %s\n", i + 1, SIM_SCENARIOS[i].name, SIM_SCENARIOS[i].about);
    Serial.printf(" Selected: %d (%s)\n", g_sc + 1, SIM_SCENARIOS[g_sc].name);
    Serial.println(" Wait for \"[FSM] PAD REST SATISFIED\", then send L to launch (5 s countdown).");
    Serial.println(" Close SW401 first to fly it armed. T = telemetry for tools/flight_live.py,");
    Serial.println(" ? = this menu, Z = reboot for the next run.");
    Serial.println("====================================================================");
}

void sim_after_setup(PyroState *pyros, bool resuming) {
    sim_begin();
    if (g_continued) {
        Serial.printf("[SIM] Carrying on scenario %d (%s) after the simulated hang, at T+%.2f s.\n",
                      g_sc + 1, SIM_SCENARIOS[g_sc].name, sim_t());
        if (!resuming)
            Serial.println("[SIM] WARNING: the firmware did NOT resume the flight — expect FAILs.");
        return;
    }
    // A fresh simulated flight: clear the sim's own fired flags (its EEPROM
    // slot, not the flight firmware's) so a previous run can't block the chute.
    Serial.println("[SIM] Clearing the simulation build's pyro fired flags for a fresh run:");
    pyro_clear_fired(pyros);
    print_menu();
}

void sim_command(char c) {
    sim_begin();
    if (c >= '1' && c < '1' + SIM_SCENARIO_COUNT) {
        if (g_launch_set) {
            Serial.println("[SIM] A run is under way — send Z to reboot before picking another.");
            return;
        }
        select_scenario((uint8_t)(c - '1'));
        Serial.printf("[SIM] Scenario %d: %s — %s\n", g_sc + 1, SIM_SCENARIOS[g_sc].name, SIM_SCENARIOS[g_sc].about);
        Serial.println("[SIM] Send L to launch.");
        return;
    }
    switch (c) {
        case 'L': case 'l':
            if (g_launch_set) {
                Serial.println("[SIM] Already launched — send Z to reboot for another run.");
            } else if (!g_pad_rest) {
                Serial.println("[SIM] Not yet: wait for \"[FSM] PAD REST SATISFIED\" (2 s still after boot), then send L.");
            } else {
                g_ign_us     = micros64() + (uint64_t)COUNTDOWN_S * 1000000ULL;
                g_launch_set = true;
                Serial.printf("[SIM] Scenario %d (%s): ignition in %d s.\n", g_sc + 1, SIM_SCENARIOS[g_sc].name, COUNTDOWN_S);
            }
            break;
        case 'Z': case 'z': {
            SimResetRecord r = {};
            EEPROM.put(SIM_EEPROM_ADDR, r);
            Serial.println("[SIM] Rebooting...");
            Serial.flush();
            delay(50);
            SCB_AIRCR = 0x05FA0004;   // system reset
            break;
        }
        case '?': case 'h': case 'H':
            print_menu();
            break;
        case '\r': case '\n': case ' ':
            break;
        default:
            Serial.printf("[SIM] Unknown command '%c' — send ? for the menu.\n", c);
            break;
    }
}

// ------------------------------------------------------------
// Report
// ------------------------------------------------------------
static int g_pass = 0, g_fail = 0;

static void check_time(const char *what, double got, double lo, double hi) {
    bool ok = !isnan(got) && !isnan(lo) && !isnan(hi) && got >= lo && got <= hi;
    if (ok) g_pass++; else g_fail++;
    if (isnan(got)) Serial.printf("  %-36s   never ", what);
    else            Serial.printf("  %-36s T%+7.2f", what, got);
    if (isnan(lo) || isnan(hi)) Serial.printf("   expected, but the simulated event never happened");
    else                        Serial.printf("   expect T%+.2f to T%+.2f", lo, hi);
    Serial.println(ok ? "   PASS" : "   FAIL");
}

static void check_true(const char *what, bool ok) {
    if (ok) g_pass++; else g_fail++;
    Serial.printf("  %-36s %s\n", what, ok ? "  PASS" : "  FAIL");
}

static double or_nan(double t) { return t >= 0.0 ? t : NAN; }

static void report() {
    const SimScenario &sc = SIM_SCENARIOS[g_sc];
    const SimModel    &m  = g_model;
    const double *e = g_ev.t_state;
    g_pass = g_fail = 0;

    Serial.println();
    Serial.println("======================== SIMULATION REPORT ========================");
    Serial.printf(" Scenario %d: %s — %s\n", g_sc + 1, sc.name, sc.about);
    Serial.printf(" Arming switch at ignition: %s\n", g_was_armed ? "closed (ARMED)" : "open (flew disarmed)");

    if (sc.fault == SIM_FAULT_PAD_BUMP) {
        check_true("Launch trigger seen, then rejected", g_ev.triggers > 0 && g_ev.false_triggers > 0);
        check_true("No liftoff (never POWERED)", isnan(e[STATE_POWERED]));
        check_true("Main not fired", isnan(g_ev.t_main));
        check_true("Pad rest latched again after the bump", g_pad_rest);
    } else {
        Serial.print(" Simulated rocket: liftoff ");
        if (m.t_liftoff >= 0) Serial.printf("T+%.2f", m.t_liftoff);
        else                  Serial.print("never");
        Serial.printf(", burnout T+%.2f, apogee ", (double)SIM_BURN_S);
        if (m.t_apogee >= 0) Serial.printf("T+%.2f at %.1f m", m.t_apogee, m.apogee_m);
        else                 Serial.print("never");
        Serial.print(", touchdown ");
        if (m.t_touchdown >= 0) Serial.printf("T+%.2f\n", m.t_touchdown);
        else                    Serial.println("not yet");

        double lo   = or_nan(m.t_liftoff);
        double apo  = or_nan(m.t_apogee);
        double td   = or_nan(m.t_touchdown);
        bool burn_check = !((sc.fault == SIM_FAULT_IMU_DEAD || sc.fault == SIM_FAULT_WDT_RESET) &&
                            sc.fault_t < SIM_BURN_S);

        check_time("Launch confirmed (POWERED)", e[STATE_POWERED], lo, lo + 1.0);
        if (burn_check)
            check_time("Burnout (COAST)", e[STATE_COAST], SIM_BURN_S - 0.3, SIM_BURN_S + 0.3);
        check_time("Apogee detected (APOGEE)", e[STATE_APOGEE], apo - 0.5, apo + 2.5);
        check_time("Main chute fired", g_ev.t_main, e[STATE_APOGEE], e[STATE_APOGEE] + 0.5);
        check_time("Descending under the main (MAIN)", e[STATE_MAIN], e[STATE_APOGEE], e[STATE_APOGEE] + 0.5);
        check_true("No touchdown call while airborne", !g_ev.landed_airborne);

        // Fall-over, the speed estimate settling, then the LANDED_TIME_MS hold
        double land_hi = td + LANDED_TIME_MS / 1000.0 + 5.0;
        if (sc.fault == SIM_FAULT_BARO_DEAD || sc.fault == SIM_FAULT_IMU_DEAD) {
            double fb = e[STATE_POWERED] + (LANDED_FALLBACK_MS + LANDED_TIME_MS) / 1000.0 + 3.0;
            if (!isnan(fb) && fb > land_hi) land_hi = fb;
        }
        check_time("Touchdown (LANDED)", e[STATE_LANDED], td, land_hi);

        if (sc.fault == SIM_FAULT_IMU_DEAD) {
            check_time("IMU fault flagged", g_ev.t_imu_fault, sc.fault_t, sc.fault_t + 0.5);
            check_true("TVC off after the IMU fault", !g_ev.tvc_after_fault);
        }
        if (sc.fault == SIM_FAULT_WDT_RESET)
            check_true("Flight resumed after the watchdog reset", g_ev.resumed);
        if (!isnan(g_ev.apogee_alt))
            Serial.printf(" Altitude estimate when apogee was called: %.1f m\n", g_ev.apogee_alt);
    }

    Serial.printf(" RESULT: %s — %d passed, %d failed\n", g_fail ? "FAIL" : "PASS", g_pass, g_fail);
    Serial.println(" Flight log: FLIGHT_XXX.CSV on the SD card — tools/flight_plot.m plots it.");
    Serial.println(" Send Z to reboot for the next scenario.");
    Serial.println("====================================================================");
}

// ------------------------------------------------------------
// Per-tick observer
// ------------------------------------------------------------
void sim_observe(const FlightSM *fsm, bool main_fired, float altitude_m) {
    sim_begin();
    g_pad_rest = fsm->pad_rest_satisfied;
    if (!g_launch_set) return;

    double t = sim_t();
    if (t < 0.0) {
        int sec = (int)ceil(-t);
        if (sec != g_countdown) {
            g_countdown = sec;
            Serial.printf("[SIM] T-%d\n", sec);
        }
        return;
    }
    if (!g_ignited_msg) {
        g_ignited_msg = true;
        g_was_armed   = fsm->state == STATE_ARMED;
        Serial.printf("[SIM] IGNITION — scenario %d (%s)\n", g_sc + 1, SIM_SCENARIOS[g_sc].name);
    }
    if (g_reported) return;

    // The firmware fired the main: open the simulated chute.
    if (main_fired && !g_deployed) {
        g_deployed = true;
        g_ev.t_main = t;
        sim_model_deploy(&g_model);
        Serial.printf("[SIM] Main chute opening at T+%.2f s, %.1f m (true)\n", t, g_model.h);
    }

    FlightState s = fsm->state;
    if (s != g_ev.last_state) {
        // When the state was entered, by the FSM's own clock — the tick that
        // enters LANDED also writes the log to SD (seconds) before we get here.
        double t_entry = t - (double)(millis() - fsm->state_entry_ms) / 1000.0;
        if (isnan(g_ev.t_state[s])) g_ev.t_state[s] = t_entry;
        if (s == STATE_APOGEE && isnan(g_ev.apogee_alt)) g_ev.apogee_alt = altitude_m;
        if (s == STATE_LANDED && !(g_model.t_touchdown >= 0.0 && t_entry >= g_model.t_touchdown)) {
            g_ev.landed_airborne = true;
            Serial.printf("[SIM] !! Touchdown called at T+%.2f s while the rocket was still in the air\n", t_entry);
        }
        g_ev.last_state = s;
    }

    bool trig = fsm->launch_detect_ms != 0;
    if (trig && !g_ev.trig_prev) g_ev.triggers++;
    if (!trig && g_ev.trig_prev && (s == STATE_IDLE || s == STATE_ARMED)) g_ev.false_triggers++;
    g_ev.trig_prev = trig;

    if (fsm->imu_fault && isnan(g_ev.t_imu_fault)) g_ev.t_imu_fault = t;
    if (fsm->imu_fault && fsm->tvc_enabled && t > g_ev.t_imu_fault + 0.1) g_ev.tvc_after_fault = true;
    if (fsm->resumed) g_ev.resumed = true;

    // Report once landed, or at a time limit (past the landing fallback).
    const SimScenario &sc = SIM_SCENARIOS[g_sc];
    double powered = isnan(g_ev.t_state[STATE_POWERED]) ? 1.0 : g_ev.t_state[STATE_POWERED];
    double limit   = sc.fault == SIM_FAULT_PAD_BUMP
                         ? BUMP_REPORT_S
                         : powered + (LANDED_FALLBACK_MS + LANDED_TIME_MS) / 1000.0 + 8.0;
    bool landed = s == STATE_LANDED && t > g_ev.t_state[STATE_LANDED] + 0.2;
    if (landed || t > limit) {
        g_reported = true;
        report();
    } else if (t > g_next_wait_msg && sc.fault != SIM_FAULT_PAD_BUMP) {
        g_next_wait_msg += 20.0;
        Serial.printf("[SIM] T+%.0f s — waiting for touchdown detection (report by T+%.0f s at the latest)\n", t, limit);
    }
}

// ------------------------------------------------------------
// Simulated drivers — same signatures as lsm6dsox.cpp, lps22hb.cpp,
// mag.cpp and mag_calib.cpp, which this build leaves out.
// ------------------------------------------------------------
bool lsm6dsox_init() {
    sim_begin();
    Serial.println("[SIM] LSM6DSOX simulated");
    return true;
}

bool lsm6dsox_calibrate_gyro(GyroBias *bias, void (*keepalive)()) {
    (void)keepalive;
    bias->x = bias->y = bias->z = 0.0f;
    return true;
}

void lsm6dsox_save_bias(const GyroBias *bias) { (void)bias; }

bool lsm6dsox_load_bias(GyroBias *bias) {
    bias->x = bias->y = bias->z = 0.0f;
    return true;
}

void lsm6dsox_read(LSM6DSOX_Data *out, const GyroBias *bias) {
    sim_begin();
    double t = sim_t();
    const SimScenario &sc = SIM_SCENARIOS[g_sc];

    // Watchdog-reset scenario: hang here, unfed. The 500 ms watchdog reboots
    // the board, and the flight firmware's own resume logic takes over.
    if (sc.fault == SIM_FAULT_WDT_RESET && g_launch_set && !g_continued && t >= sc.fault_t) {
        SimResetRecord r;
        r.magic     = SIM_REC_MAGIC;
        r.scenario  = g_sc;
        r.armed     = g_was_armed;
        r.reset_ms  = (uint32_t)(t * 1000.0);
        r.powered_s = (float)g_ev.t_state[STATE_POWERED];
        EEPROM.put(SIM_EEPROM_ADDR, r);
        Serial.printf("[SIM] Simulating a firmware hang at T+%.2f s — the watchdog reboots the board in 0.5 s.\n", t);
        Serial.println("[SIM] USB drops for a moment; reconnect the serial monitor if it doesn't on its own.");
        Serial.flush();
        while (true) { __asm__ volatile("nop"); }   // volatile: the compiler may not drop the hang
    }

    sim_model_advance(&g_model, t);
    if (sc.fault == SIM_FAULT_IMU_DEAD && g_launch_set && t >= sc.fault_t) {
        *out = {};
        out->valid = false;
        return;
    }
    SimSensors s;
    sim_model_sensors(&g_model, &s);
    out->ax = s.ax;  out->ay = s.ay;  out->az = s.az;
    out->gx = s.gx - (bias ? bias->x : 0.0f);
    out->gy = s.gy - (bias ? bias->y : 0.0f);
    out->gz = s.gz - (bias ? bias->z : 0.0f);
    out->valid = true;
}

bool lps22hb_init(void) {
    sim_begin();
    Serial.println("[SIM] LPS22HB simulated");
    return true;
}

// New sample at 75 Hz, like the real part's ODR: NaN in between.
void lps22hb_read(LPS22HB_Data *out) {
    static uint32_t last_k = 0xFFFFFFFFu;
    sim_begin();
    double t = sim_t();
    const SimScenario &sc = SIM_SCENARIOS[g_sc];
    uint32_t k = (uint32_t)(micros64() * 75ULL / 1000000ULL);
    bool dead = sc.fault == SIM_FAULT_BARO_DEAD && g_launch_set && t >= sc.fault_t;
    if (dead || k == last_k) {
        out->valid         = false;
        out->pressure_pa   = NAN;
        out->temperature_c = NAN;
        return;
    }
    last_k = k;
    sim_model_advance(&g_model, t);
    SimSensors s;
    sim_model_sensors(&g_model, &s);
    out->pressure_pa   = s.pressure_hpa * 100.0f;
    out->temperature_c = s.temperature_c;
    out->valid         = true;
}

bool lps22hb_read_samples(uint16_t samples, float *samples_hpa) {
    for (uint16_t i = 0; i < samples; i++) {
        uint32_t t0 = millis();
        LPS22HB_Data d;
        for (;;) {
            lps22hb_read(&d);
            if (d.valid) break;
            if (millis() - t0 > LPS22HB_SAMPLE_TIMEOUT_MS) return false;
            delay(1);
        }
        samples_hpa[i] = d.pressure_pa / 100.0f;
    }
    return true;
}

static void mag_sample(mag_data *out) {
    sim_model_advance(&g_model, sim_t());
    SimSensors s;
    sim_model_sensors(&g_model, &s);
    out->mag_x = s.mx;
    out->mag_y = s.my;
    out->mag_z = s.mz;
    out->valid = true;
}

bool mag_init() {
    sim_begin();
    Serial.println("[SIM] MMC5603NJ simulated");
    return true;
}

void mag_read(mag_data *out) {
    sim_begin();
    mag_sample(out);
}

// ~40 Hz, like the real part in continuous mode
bool mag_poll(mag_data *out) {
    static uint32_t last_ms = 0;
    sim_begin();
    uint32_t now = millis();
    if (now - last_ms < 25) return false;
    last_ms = now;
    mag_sample(out);
    return true;
}

// The simulated field is already "calibrated": identity calibration.
bool mag_load_calib(MagCalib *cal) {
    cal->offset_x = cal->offset_y = cal->offset_z = 0.0f;
    cal->scale_x  = cal->scale_y  = cal->scale_z  = 1.0f;
    return true;
}

bool mag_calibrate(MagCalib *cal) { return mag_load_calib(cal); }

void mag_save_calib(const MagCalib *cal) { (void)cal; }

void mag_apply_calib(const MagCalib *cal, float mx, float my, float mz,
                     float *cx, float *cy, float *cz) {
    *cx = (mx - cal->offset_x) * cal->scale_x;
    *cy = (my - cal->offset_y) * cal->scale_y;
    *cz = (mz - cal->offset_z) * cal->scale_z;
}

#endif // SIM_MODE
