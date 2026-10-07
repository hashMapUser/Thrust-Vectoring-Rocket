// ============================================================
//  telemetry.cpp — $TLM bench telemetry over USB serial
//  Frame format and field order: see include/telemetry.h
// ============================================================

#include <Arduino.h>
#include <stdio.h>
#include "telemetry.h"
#include "flight_sm.h"

static bool _enabled = false;

void telemetry_set_enabled(bool on) { _enabled = on; }
bool telemetry_enabled()            { return _enabled; }

bool telemetry_emit(const TelemetryFrame *f) {
    // !Serial: no host has the port open (DTR low), so nothing is
    // listening — skip the formatting work entirely.
    if (!_enabled || !f || !Serial) return false;

    const size_t n_states = sizeof(STATE_NAMES) / sizeof(STATE_NAMES[0]);
    const char *state = (f->state < n_states) ? STATE_NAMES[f->state] : "?";

    // 6 bytes held back for the "*XX\r\n" trailer and its NUL.
    char line[320];
    const int body_max = (int)sizeof(line) - 6;

    int n = snprintf(line, body_max,
        "$TLM,%lu,%s,%u,"
        "%.5f,%.5f,%.5f,%.5f,"
        "%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,"
        "%.3f,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,"
        "%.2f,%.2f,%.2f,%.2f,%.1f,"
        "%.3f,%.3f,%.0f,%.0f,"
        "%.2f,%lu",
        (unsigned long)f->t_ms, state, (unsigned)f->flags,
        f->q0, f->q1, f->q2, f->q3,
        f->tip_a, f->tip_b, f->spin,
        f->gx, f->gy, f->gz,
        f->ax, f->ay, f->az,
        f->mx, f->my, f->mz,
        f->alt_m, f->vel_ms, f->baro_alt_m, f->press_hpa, f->temp_c,
        f->pid_p, f->pid_y, f->servo_p_us, f->servo_y_us,
        f->pack_v, (unsigned long)f->loop_us);

    if (n <= 0 || n >= body_max) return false;   // truncated — never send a partial frame

    uint8_t ck = 0;
    for (int i = 1; i < n; i++) ck ^= (uint8_t)line[i];   // skip the '$'
    n += snprintf(line + n, 6, "*%02X\r\n", ck);

    // Whole line or nothing. A full buffer means the host has stopped
    // reading; writing anyway would block until the USB stack times out.
    if (Serial.availableForWrite() < n) return false;

    Serial.write((const uint8_t *)line, (size_t)n);
    return true;
}
