#include <Arduino.h>
#include <string.h>
#include "flight_resume.h"

// Register layout:
//   LPGPR0  [31:24] magic  [23:16] state  [15:8] flags  [7:0] checksum
//   LPGPR1  ms since launch
//   LPGPR2  ground pressure  (float bits)
//   LPGPR3  peak altitude    (float bits)
#define RESUME_MAGIC        0xA7u
#define RESUME_F_PROVEN     0x01u

static uint32_t float_bits(float f)  { uint32_t u; memcpy(&u, &f, 4); return u; }
static float    bits_float(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

static uint8_t checksum(uint32_t w0_hi, uint32_t w1, uint32_t w2, uint32_t w3) {
    uint32_t x = w0_hi ^ w1 ^ w2 ^ w3 ^ 0x5Au;
    x ^= x >> 16;
    x ^= x >> 8;
    return (uint8_t)x;
}

bool resume_boot_was_watchdog() {
    uint32_t srsr = SRC_SRSR;
    // The reset-status bits are sticky; write them back to clear them so
    // a later non-watchdog reset doesn't still read as one.
    SRC_SRSR = srsr;
    return (srsr & SRC_SRSR_WDOG3_RST_B) != 0;
}

bool resume_load(ResumeRecord *out) {
    uint32_t w0 = SNVS_LPGPR0, w1 = SNVS_LPGPR1, w2 = SNVS_LPGPR2, w3 = SNVS_LPGPR3;

    if ((w0 >> 24) != RESUME_MAGIC) return false;
    if ((uint8_t)w0 != checksum(w0 & 0xFFFFFF00u, w1, w2, w3)) return false;

    FlightState state = (FlightState)((w0 >> 16) & 0xFF);
    if (state < STATE_POWERED || state > STATE_MAIN) return false;

    out->state           = state;
    out->flight_proven   = ((w0 >> 8) & RESUME_F_PROVEN) != 0;
    out->ms_since_launch = w1;
    out->ground_hpa      = bits_float(w2);
    out->peak_altitude_m = bits_float(w3);
    return isfinite(out->ground_hpa) && isfinite(out->peak_altitude_m);
}

void resume_save(const ResumeRecord *rec) {
    uint32_t w0 = ((uint32_t)RESUME_MAGIC << 24) |
                  ((uint32_t)(rec->state & 0xFF) << 16) |
                  ((uint32_t)(rec->flight_proven ? RESUME_F_PROVEN : 0) << 8);
    uint32_t w1 = rec->ms_since_launch;
    uint32_t w2 = float_bits(rec->ground_hpa);
    uint32_t w3 = float_bits(rec->peak_altitude_m);
    w0 |= checksum(w0, w1, w2, w3);

    // Payload first, header last: a reset between the writes leaves a
    // header whose checksum doesn't match, which resume_load() rejects.
    SNVS_LPGPR1 = w1;
    SNVS_LPGPR2 = w2;
    SNVS_LPGPR3 = w3;
    SNVS_LPGPR0 = w0;
}
