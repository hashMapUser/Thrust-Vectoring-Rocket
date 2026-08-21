#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "board_pins.h"
#include "lsm6dsox.h"
#include "lps22hb.h"
#include "mag.h"
#include "flight_sm.h"

// ============================================================
// SENSOR HEALTH MONITOR
// ============================================================
// Aggregates the state of every sensor / storage / safety channel
// into one structure, and renders it two ways:
//
//   health_print_page()   human-readable ASCII page (Serial, on demand)
//   health_emit_frame()   one-line CSV frame for tools/health_monitor.py
//
// DESIGN NOTE — why two different failure policies exist:
//
//   The IMU runs at 833 Hz and the control loop at 125 Hz, so every
//   lsm6dsox_read() must return fresh data. A false `valid` flag there
//   is a genuine SPI fault, so IMU health uses CONSECUTIVE-FAILURE
//   counting.
//
//   The barometer runs at 75 Hz. lps22hb_read() gates on P_DA and sets
//   valid = false when no new sample is ready yet — which happens on
//   roughly 40% of reads at a 125 Hz loop rate. Counting those as
//   failures would mark the baro degraded on every flight. Baro health
//   therefore uses TIME-SINCE-LAST-GOOD-SAMPLE (staleness) instead.
//
// The per-channel policy table in health.cpp encodes which rule applies.
// ============================================================

// --------------------------------------------------------
// CHANNELS
// --------------------------------------------------------

typedef enum {
    HC_IMU   = 0,   // LSM6DSOX      SPI0
    HC_BARO,        // LPS22HB       I2C0 (Wire)
    HC_MAG,         // MMC5603NJ     I2C1 (Wire1) — not fitted this flight
    HC_FLASH,       // GD25Q128      SPI2
    HC_SD,          // microSD       SPI1 — disabled (H6: VDD wired to 5 V)
    HC_ARM,         // ARM_SENSE     A10 analog
    HC_PYRO1,       // main chute    D32 — no continuity sense (H5)
    HC_PYRO2,       // unused        D31 — no continuity sense (H5)
    HC_BATT,        // pack voltage  — no ADC on pin 2 (documented in board_pins.h)
    HC_COUNT
} HealthChannel;

// --------------------------------------------------------
// STATES
// --------------------------------------------------------

typedef enum {
    HS_NOT_FITTED = 0,   // hardware absent or disabled by design — not a fault
    HS_OK         = 1,   // fresh, in range, no recent failures
    HS_DEGRADED   = 2,   // intermittent failures, stale, or implausible values
    HS_FAILED     = 3,   // sustained failure — treat as unavailable
    HS_UNKNOWN    = 4    // fitted, but has never produced a good sample
} HealthState;

// --------------------------------------------------------
// FLAGS (bitfield — reported in both outputs)
// --------------------------------------------------------

#define HF_NONE        0x00
#define HF_STALE       0x01   // no good sample within the channel's stale window
#define HF_RANGE       0x02   // value outside plausible engineering bounds
#define HF_UNSTABLE    0x04   // pad-rest plausibility failed (accel != 1 g / gyro moving)
#define HF_NEVER_GOOD  0x08   // never produced a valid sample since boot
#define HF_INIT_FAIL   0x10   // driver init returned false
#define HF_RECOVERING  0x20   // was failed; on probation until continuously good

// --------------------------------------------------------
// PLAUSIBILITY BOUNDS
// Mirrors the thresholds already used in src/test_imu.h and src/test_baro.h
// so bench tests and in-flight health agree on what "good" means.
// --------------------------------------------------------

// Applied only while on the pad (IDLE / ARMED) — meaningless under thrust.
#define HEALTH_PAD_ACCEL_MAG_MIN   0.90f    // g
#define HEALTH_PAD_ACCEL_MAG_MAX   1.10f    // g
#define HEALTH_PAD_GYRO_MAX_DPS    2.0f     // deg/s

// Applied in every flight state — these are sensor-range sanity checks,
// not attitude checks. ±16 g / ±2000 dps full scale; a rail is suspicious.
#define HEALTH_ACCEL_RAIL_G        15.5f
#define HEALTH_GYRO_RAIL_DPS       1980.0f

// Barometer plausibility (hPa / °C)
#define HEALTH_PRESS_MIN_HPA       850.0f
#define HEALTH_PRESS_MAX_HPA       1084.0f
#define HEALTH_TEMP_MIN_C         -20.0f
#define HEALTH_TEMP_MAX_C          70.0f

// Magnetometer plausibility — Earth's field is ~0.25–0.65 Gauss.
// Wide bounds because hard-iron offsets from the airframe shift this.
#define HEALTH_MAG_MIN_GAUSS       0.05f
#define HEALTH_MAG_MAX_GAUSS       2.00f

// --------------------------------------------------------
// TIMING
// --------------------------------------------------------

// Rate estimator smoothing. The EWMA runs over the sample PERIOD, not over
// 1/period: averaging instantaneous rates over irregular intervals is biased
// high (it is an arithmetic mean where the harmonic mean is wanted). Averaging
// the period and inverting once at the end gives the true mean rate.
#define HEALTH_RATE_ALPHA          0.05f

// After a channel reaches HS_FAILED, one good sample is not enough to call it
// healthy again — a flapping connection would read as OK. The channel is held
// at HS_DEGRADED until it has been continuously good for this long.
#define HEALTH_RECOVER_HOLD_MS     1000

// How often health_emit_frame() is allowed to transmit [ms].
#define HEALTH_FRAME_INTERVAL_MS   500

// --------------------------------------------------------
// PER-CHANNEL RECORD
// --------------------------------------------------------

typedef struct {
    HealthState state;
    uint8_t     flags;

    bool        init_ok;         // driver init() returned true
    bool        ever_good;       // at least one valid sample seen

    uint32_t    last_good_ms;    // millis() of most recent valid sample
    uint32_t    samples;         // total valid samples
    uint32_t    fails;           // total invalid / errored reads
    uint16_t    consec_fails;    // current consecutive-failure run
    uint16_t    consec_fail_max; // worst run since boot

    float       period_ms;       // EWMA of interval between good samples
    float       rate_hz;         // 1000 / period_ms, or 0 before first estimate
    uint32_t    recover_until_ms;// non-zero while on post-failure probation
} ChannelHealth;

// --------------------------------------------------------
// MONITOR CONTEXT
// --------------------------------------------------------

typedef struct {
    ChannelHealth ch[HC_COUNT];
    uint32_t      boot_ms;
    uint32_t      last_frame_ms;
    uint16_t      arm_counts;     // last ARM_SENSE ADC reading (12-bit)
} HealthMonitor;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Zero all counters and set each channel to its designed starting state.
 * Channels marked not-fitted in the policy table start at HS_NOT_FITTED;
 * everything else starts at HS_UNKNOWN until it produces a good sample.
 * Call once in setup(), before any driver init().
 */
void health_init(HealthMonitor *h);

/**
 * Record the result of a driver's init() call.
 * A false result latches HF_INIT_FAIL and drives the channel to HS_FAILED.
 * Call immediately after each *_init() in setup().
 */
void health_set_init(HealthMonitor *h, HealthChannel c, bool ok);

/**
 * Feed one IMU sample. Uses consecutive-failure counting (see header note).
 * @param on_pad  true in IDLE / ARMED — enables the 1 g / static-gyro
 *                plausibility check. Pass false once POWERED is entered.
 */
void health_update_imu(HealthMonitor *h, const LSM6DSOX_Data *d,
                       uint32_t now_ms, bool on_pad);

/**
 * Feed one barometer sample. Uses staleness, not consecutive failures —
 * an invalid read is normal when P_DA has not yet asserted.
 */
void health_update_baro(HealthMonitor *h, const LPS22HB_Data *d, uint32_t now_ms);

/**
 * Feed one magnetometer sample. No-op when the channel is not fitted.
 */
void health_update_mag(HealthMonitor *h, const mag_data *d, uint32_t now_ms);

/**
 * Report the outcome of a logger write attempt.
 * Feeds HC_SD, which is the flight log destination on this board revision.
 * @param ok  false if the record was dropped or the card reported not-ready.
 */
void health_update_log(HealthMonitor *h, bool ok, uint32_t now_ms);

/**
 * Feed the raw 12-bit ARM_SENSE reading. Stores the count for display and
 * marks the channel good — the ADC itself is the thing being health-checked,
 * not the armed/safe state (that belongs to the FSM).
 */
void health_update_arm(HealthMonitor *h, uint16_t adc_counts, uint32_t now_ms);

/**
 * Re-evaluate staleness for every fitted channel and recompute states.
 * Call once per loop iteration, after all health_update_* calls.
 */
void health_tick(HealthMonitor *h, uint32_t now_ms);

/**
 * @return true if the vehicle is safe to arm / fly on sensor grounds.
 * IMU must be HS_OK. Baro may be degraded or absent (alt_init falls back to
 * 1013.25 hPa), but a FAILED baro blocks flight because the apogee detector
 * and the G5 altitude lockout both depend on it.
 */
bool health_flight_critical_ok(const HealthMonitor *h);

/**
 * One-line reason string explaining a false health_flight_critical_ok(),
 * or "all critical sensors nominal" when GO. Points to static storage.
 */
const char *health_critical_reason(const HealthMonitor *h);

/**
 * Print the full human-readable status page to Serial.
 * Blocking (~2 ms at 115200 baud) — call from the serial command handler
 * in IDLE, never inside the flight control path.
 */
void health_print_page(const HealthMonitor *h, uint32_t now_ms, FlightState st);

/**
 * Emit one machine-parseable status frame, rate-limited to
 * HEALTH_FRAME_INTERVAL_MS. Safe to call every loop iteration.
 *
 * Format (NMEA-style XOR checksum over everything between $ and *):
 *   $HLTH,<t_ms>,<state_name>,NAME=st:flags:consec:fails:age_ms,...*XX
 */
void health_emit_frame(HealthMonitor *h, uint32_t now_ms, FlightState st);

/** Short display name for a channel, e.g. "IMU". */
const char *health_channel_name(HealthChannel c);

/** Short display name for a state, e.g. "OK". */
const char *health_state_name(HealthState s);