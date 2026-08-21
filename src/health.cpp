#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "health.h"

// ============================================================
// PER-CHANNEL POLICY TABLE
// ============================================================
// This table is the single place where "what counts as broken"
// is defined for each channel. Nothing below it is channel-specific.
//
//   fitted          false => hardware absent/disabled by design.
//                   Reported as N/FIT, never as a fault.
//   stale_ms        age of last good sample that means DEGRADED.
//   dead_ms         age that means FAILED. 0 disables staleness checks.
//   degrade_consec  consecutive failed reads that mean DEGRADED.
//   fail_consec     consecutive failed reads that mean FAILED.
//                   0 disables consecutive counting — correct for the
//                   barometer, where an invalid read just means "no new
//                   sample yet" (P_DA low), not a bus error.
// ============================================================

typedef struct {
    const char *name;
    const char *bus;
    bool        fitted;
    uint32_t    stale_ms;
    uint32_t    dead_ms;
    uint16_t    degrade_consec;
    uint16_t    fail_consec;
    const char *note;
} ChannelPolicy;

static const ChannelPolicy CH_POLICY[HC_COUNT] = {
    // name     bus     fitted stale dead  deg fail  note
    { "IMU",   "SPI0",  true,    40,  200,   3,  25, "" },
    { "BARO",  "I2C0",  true,    60,  500,   0,   0, "75 Hz ODR; invalid read = no new sample" },
    { "MAG",   "I2C1",  false,    0,    0,   0,   0, "not fitted this flight" },
    { "FLASH", "SPI2",  false,    0,    0,   0,   0, "GD25Q128 miswired - unusable, see logger.h" },
    { "SD",    "SPI1",  true,  2000,10000,   2,  10, "H6 fixed - primary flight log (FLIGHT_XXX.CSV)" },
    { "ARM",   "A10",   true,    100, 1000,  0,   0, "" },
    { "PYRO1", "D32",   false,    0,    0,   0,   0, "A8 sense fitted; no continuity telemetry - H5" },
    { "PYRO2", "D31",   false,    0,    0,   0,   0, "A9 sense fitted; no continuity telemetry - H5" },
    { "BATT",  "A0",    false,    0,    0,   0,   0, "pin 14 is analog-capable; no firmware use yet" },
};

// ============================================================
// INTERNAL HELPERS
// ============================================================

static void note_good(ChannelHealth *c, uint32_t now_ms) {
    if (c->ever_good && now_ms > c->last_good_ms) {
        // Average the PERIOD, then invert. Averaging 1/dt directly would bias
        // the reported rate high whenever intervals are uneven — which they
        // always are when a 75 Hz sensor is polled from a 125 Hz loop.
        const float dt = (float)(now_ms - c->last_good_ms);
        c->period_ms = (c->period_ms <= 0.0f)
                     ? dt
                     : c->period_ms + HEALTH_RATE_ALPHA * (dt - c->period_ms);
        c->rate_hz = (c->period_ms > 0.0f) ? (1000.0f / c->period_ms) : 0.0f;
    }
    c->ever_good    = true;
    c->last_good_ms = now_ms;
    c->samples++;
    c->consec_fails = 0;
    c->flags &= (uint8_t)~HF_NEVER_GOOD;
}

static void note_fail(ChannelHealth *c) {
    c->fails++;
    if (c->consec_fails < 0xFFFFu) c->consec_fails++;
    if (c->consec_fails > c->consec_fail_max) c->consec_fail_max = c->consec_fails;
}

static void set_flag(ChannelHealth *c, uint8_t flag, bool on) {
    if (on) c->flags |= flag;
    else    c->flags &= (uint8_t)~flag;
}

/**
 * Collapse counters + flags into a single HealthState.
 * Worst-case wins: any rule that says FAILED overrides any rule that
 * says DEGRADED.
 */
static void recompute(ChannelHealth *c, const ChannelPolicy *p, uint32_t now_ms) {
    if (!p->fitted) { c->state = HS_NOT_FITTED; return; }

    if (c->flags & HF_INIT_FAIL) { c->state = HS_FAILED; return; }

    if (!c->ever_good) {
        c->flags |= HF_NEVER_GOOD;
        c->state = HS_UNKNOWN;
        return;
    }

    HealthState worst = HS_OK;

    // --- staleness ---
    const uint32_t age = now_ms - c->last_good_ms;
    bool stale = false;
    if (p->dead_ms && age >= p->dead_ms) {
        worst = HS_FAILED;
        stale = true;
    } else if (p->stale_ms && age >= p->stale_ms) {
        if (worst < HS_DEGRADED) worst = HS_DEGRADED;
        stale = true;
    }
    set_flag(c, HF_STALE, stale);

    // --- consecutive failures (disabled where fail_consec == 0) ---
    if (p->fail_consec && c->consec_fails >= p->fail_consec) {
        worst = HS_FAILED;
    } else if (p->degrade_consec && c->consec_fails >= p->degrade_consec) {
        if (worst < HS_DEGRADED) worst = HS_DEGRADED;
    }

    // --- plausibility ---
    if (c->flags & (HF_RANGE | HF_UNSTABLE)) {
        if (worst < HS_DEGRADED) worst = HS_DEGRADED;
    }

    // --- post-failure probation ---
    // Entering HS_FAILED opens a recovery window. Until it closes the channel
    // cannot report better than HS_DEGRADED, so an intermittent bus does not
    // flicker back to OK on a single lucky read.
    if (worst == HS_FAILED) {
        c->recover_until_ms = now_ms + HEALTH_RECOVER_HOLD_MS;
    } else if (c->recover_until_ms != 0) {
        if ((int32_t)(now_ms - c->recover_until_ms) >= 0) {
            c->recover_until_ms = 0;
        } else if (worst < HS_DEGRADED) {
            worst = HS_DEGRADED;
        }
    }
    set_flag(c, HF_RECOVERING, c->recover_until_ms != 0);

    c->state = worst;
}

// ============================================================
// PUBLIC API
// ============================================================

void health_init(HealthMonitor *h) {
    memset(h, 0, sizeof(HealthMonitor));
    h->boot_ms       = millis();
    h->last_frame_ms = 0;

    for (int i = 0; i < HC_COUNT; i++) {
        ChannelHealth *c = &h->ch[i];
        c->rate_hz = 0.0f;
        c->flags   = CH_POLICY[i].fitted ? HF_NEVER_GOOD : HF_NONE;
        c->state   = CH_POLICY[i].fitted ? HS_UNKNOWN : HS_NOT_FITTED;
        c->init_ok = !CH_POLICY[i].fitted;   // unfitted channels are vacuously "fine"
    }
}

void health_set_init(HealthMonitor *h, HealthChannel ci, bool ok) {
    if (ci >= HC_COUNT) return;
    ChannelHealth *c = &h->ch[ci];
    c->init_ok = ok;
    set_flag(c, HF_INIT_FAIL, !ok);
    if (!ok && CH_POLICY[ci].fitted) c->state = HS_FAILED;
}

void health_update_imu(HealthMonitor *h, const LSM6DSOX_Data *d,
                       uint32_t now_ms, bool on_pad) {
    ChannelHealth *c = &h->ch[HC_IMU];

    if (!d || !d->valid) { note_fail(c); recompute(c, &CH_POLICY[HC_IMU], now_ms); return; }

    // Full-scale rail check — applies in every flight state.
    const float amag = sqrtf(d->ax * d->ax + d->ay * d->ay + d->az * d->az);
    const bool railed =
        fabsf(d->ax) >= HEALTH_ACCEL_RAIL_G || fabsf(d->ay) >= HEALTH_ACCEL_RAIL_G ||
        fabsf(d->az) >= HEALTH_ACCEL_RAIL_G ||
        fabsf(d->gx) >= HEALTH_GYRO_RAIL_DPS || fabsf(d->gy) >= HEALTH_GYRO_RAIL_DPS ||
        fabsf(d->gz) >= HEALTH_GYRO_RAIL_DPS ||
        isnan(amag);
    set_flag(c, HF_RANGE, railed);

    // Pad-rest plausibility — only meaningful before the motor lights.
    // Under thrust |a| is 5-15 g and this check would false-trip continuously.
    if (on_pad) {
        const float gmax = fmaxf(fmaxf(fabsf(d->gx), fabsf(d->gy)), fabsf(d->gz));
        const bool bad = (amag < HEALTH_PAD_ACCEL_MAG_MIN) ||
                         (amag > HEALTH_PAD_ACCEL_MAG_MAX) ||
                         (gmax > HEALTH_PAD_GYRO_MAX_DPS);
        set_flag(c, HF_UNSTABLE, bad);
    } else {
        set_flag(c, HF_UNSTABLE, false);
    }

    note_good(c, now_ms);
    recompute(c, &CH_POLICY[HC_IMU], now_ms);
}

void health_update_baro(HealthMonitor *h, const LPS22HB_Data *d, uint32_t now_ms) {
    ChannelHealth *c = &h->ch[HC_BARO];

    // An invalid read here is expected whenever P_DA has not asserted.
    // It is counted for visibility but does NOT drive the state machine
    // (fail_consec == 0 for this channel). Staleness does that instead.
    if (!d || !d->valid) { note_fail(c); recompute(c, &CH_POLICY[HC_BARO], now_ms); return; }

    const float hpa = d->pressure_pa / 100.0f;
    const bool bad = isnan(hpa) || isnan(d->temperature_c) ||
                     hpa < HEALTH_PRESS_MIN_HPA || hpa > HEALTH_PRESS_MAX_HPA ||
                     d->temperature_c < HEALTH_TEMP_MIN_C ||
                     d->temperature_c > HEALTH_TEMP_MAX_C;
    set_flag(c, HF_RANGE, bad);

    note_good(c, now_ms);
    recompute(c, &CH_POLICY[HC_BARO], now_ms);
}

void health_update_mag(HealthMonitor *h, const mag_data *d, uint32_t now_ms) {
    if (!CH_POLICY[HC_MAG].fitted) return;   // no-op until the mag is populated

    ChannelHealth *c = &h->ch[HC_MAG];
    if (!d || !d->valid) { note_fail(c); recompute(c, &CH_POLICY[HC_MAG], now_ms); return; }

    const float m = sqrtf(d->mag_x * d->mag_x + d->mag_y * d->mag_y + d->mag_z * d->mag_z);
    set_flag(c, HF_RANGE, isnan(m) || m < HEALTH_MAG_MIN_GAUSS || m > HEALTH_MAG_MAX_GAUSS);

    note_good(c, now_ms);
    recompute(c, &CH_POLICY[HC_MAG], now_ms);
}

void health_update_log(HealthMonitor *h, bool ok, uint32_t now_ms) {
    // Logging target is the SD card on this board revision (H6 fixed).
    // The NOR flash channel stays HS_NOT_FITTED until the V2.1 respin
    // corrects the MISO/MOSI swap.
    ChannelHealth *c = &h->ch[HC_SD];
    if (ok) note_good(c, now_ms);
    else    note_fail(c);
    recompute(c, &CH_POLICY[HC_SD], now_ms);
}

void health_update_arm(HealthMonitor *h, uint16_t adc_counts, uint32_t now_ms) {
    ChannelHealth *c = &h->ch[HC_ARM];
    h->arm_counts = adc_counts;
    note_good(c, now_ms);
    recompute(c, &CH_POLICY[HC_ARM], now_ms);
}

void health_tick(HealthMonitor *h, uint32_t now_ms) {
    for (int i = 0; i < HC_COUNT; i++) {
        recompute(&h->ch[i], &CH_POLICY[i], now_ms);
    }
}

bool health_flight_critical_ok(const HealthMonitor *h) {
    if (h->ch[HC_IMU].state != HS_OK)      return false;
    if (h->ch[HC_BARO].state == HS_FAILED) return false;
    if (h->ch[HC_BARO].state == HS_UNKNOWN)return false;
    return true;
}

const char *health_critical_reason(const HealthMonitor *h) {
    const ChannelHealth *imu  = &h->ch[HC_IMU];
    const ChannelHealth *baro = &h->ch[HC_BARO];

    if (imu->flags & HF_INIT_FAIL)  return "IMU init failed - check SPI0 wiring and CS";
    if (imu->state == HS_UNKNOWN)   return "IMU has produced no valid samples";
    if (imu->flags & HF_RANGE)      return "IMU at full-scale rail - suspect SPI corruption";
    if (imu->flags & HF_UNSTABLE)   return "IMU pad-rest check failing - vehicle moving or not vertical";
    if (imu->flags & HF_STALE)      return "IMU samples stale - read path stalled";
    if (imu->flags & HF_RECOVERING) return "IMU recovering from failure - hold for a clean window";
    if (imu->state != HS_OK)        return "IMU degraded - repeated read failures";

    if (baro->state == HS_UNKNOWN)  return "Barometer has produced no valid samples";
    if (baro->state == HS_FAILED)   return "Barometer dead - apogee detect and G5 lockout unavailable";

    return "all critical sensors nominal";
}

const char *health_channel_name(HealthChannel c) {
    return (c < HC_COUNT) ? CH_POLICY[c].name : "?";
}

const char *health_state_name(HealthState s) {
    switch (s) {
        case HS_NOT_FITTED: return "N/FIT";
        case HS_OK:         return "OK";
        case HS_DEGRADED:   return "DEGRADED";
        case HS_FAILED:     return "FAILED";
        default:            return "UNKNOWN";
    }
}

// --------------------------------------------------------
// RENDERING
// --------------------------------------------------------

/** Render flags as a fixed 6-char field: I N S R U C (dash = clear). */
static void flags_str(uint8_t f, char out[7]) {
    out[0] = (f & HF_INIT_FAIL)   ? 'I' : '-';
    out[1] = (f & HF_NEVER_GOOD)  ? 'N' : '-';
    out[2] = (f & HF_STALE)       ? 'S' : '-';
    out[3] = (f & HF_RANGE)       ? 'R' : '-';
    out[4] = (f & HF_UNSTABLE)    ? 'U' : '-';
    out[5] = (f & HF_RECOVERING)  ? 'C' : '-';
    out[6] = '\0';
}

static const char *state_str(FlightState st) {
    return (st >= STATE_IDLE && st <= STATE_ABORT) ? STATE_NAMES[st] : "?";
}

void health_print_page(const HealthMonitor *h, uint32_t now_ms, FlightState st) {
    const uint32_t up = now_ms - h->boot_ms;

    Serial.println();
    Serial.printf("======== SENSOR STATUS  T+%02lu:%02lu.%03lu  STATE: %s ========\n",
                  (unsigned long)(up / 60000UL),
                  (unsigned long)((up / 1000UL) % 60UL),
                  (unsigned long)(up % 1000UL),
                  state_str(st));

    const bool go = health_flight_critical_ok(h);
    Serial.printf(" FLIGHT-CRITICAL: %s  (%s)\n",
                  go ? "GO" : "NO-GO", health_critical_reason(h));
    Serial.println(" ---------------------------------------------------------------");
    Serial.println(" CHANNEL  BUS   STATUS     AGE      RATE     FAILS  FLAGS");

    uint8_t n_ok = 0, n_deg = 0, n_fail = 0, n_nf = 0;

    for (int i = 0; i < HC_COUNT; i++) {
        const ChannelHealth *c = &h->ch[i];
        const ChannelPolicy *p = &CH_POLICY[i];

        char fl[7];
        flags_str(c->flags, fl);

        char age[20]   = "     -";
        char rate[20]  = "      -";
        char fails[16] = "     -";

        if (p->fitted) {
            if (c->ever_good) {
                snprintf(age, sizeof(age), "%5lu ms",
                         (unsigned long)(now_ms - c->last_good_ms));
                if (c->rate_hz > 0.0f)
                    snprintf(rate, sizeof(rate), "%6.1f Hz", (double)c->rate_hz);
            }
            snprintf(fails, sizeof(fails), "%6lu", (unsigned long)c->fails);
        }

        Serial.printf(" %-8s %-5s %-9s %-8s %-8s %-6s %s\n",
                      p->name, p->bus, health_state_name(c->state),
                      age, rate, fails, p->fitted ? fl : "------");

        switch (c->state) {
            case HS_OK:         n_ok++;   break;
            case HS_DEGRADED:   n_deg++;  break;
            case HS_FAILED:     n_fail++; break;
            case HS_NOT_FITTED: n_nf++;   break;
            default:            n_fail++; break;   // UNKNOWN counts against you
        }
    }

    Serial.println(" ---------------------------------------------------------------");
    Serial.printf(" %u ok / %u degraded / %u failed-or-unknown / %u not fitted\n",
                  n_ok, n_deg, n_fail, n_nf);
    Serial.printf(" ARM_SENSE: %u counts (%.2f V)  threshold %u\n",
                  h->arm_counts, (double)h->arm_counts * 3.3 / 4095.0,
                  (unsigned)ARM_SENSE_THRESHOLD);

    Serial.println(" NOTES:");
    for (int i = 0; i < HC_COUNT; i++) {
        if (CH_POLICY[i].note[0] != '\0') {
            Serial.printf("   %-6s %s\n", CH_POLICY[i].name, CH_POLICY[i].note);
        }
    }
    Serial.println(" FLAGS: I=init-fail N=never-good S=stale R=out-of-range U=pad-unstable C=recovering");
    Serial.println("================================================================");
}

void health_emit_frame(HealthMonitor *h, uint32_t now_ms, FlightState st) {
    if (h->last_frame_ms != 0 &&
        (now_ms - h->last_frame_ms) < HEALTH_FRAME_INTERVAL_MS) return;
    h->last_frame_ms = now_ms;

    char body[400];
    int n = snprintf(body, sizeof(body), "HLTH,%lu,%s",
                     (unsigned long)now_ms, state_str(st));

    for (int i = 0; i < HC_COUNT && n > 0 && n < (int)sizeof(body); i++) {
        const ChannelHealth *c = &h->ch[i];
        const uint32_t age = c->ever_good ? (now_ms - c->last_good_ms) : 0UL;
        n += snprintf(body + n, sizeof(body) - n, ",%s=%u:%u:%u:%lu:%lu",
                      CH_POLICY[i].name,
                      (unsigned)c->state,
                      (unsigned)c->flags,
                      (unsigned)c->consec_fails,
                      (unsigned long)c->fails,
                      (unsigned long)age);
    }

    uint8_t cks = 0;
    for (int i = 0; body[i] != '\0'; i++) cks ^= (uint8_t)body[i];

    Serial.printf("$%s*%02X\n", body, cks);
}