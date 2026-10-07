#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flight_sm.h"

// ============================================================
// MID-FLIGHT WATCHDOG RECOVERY
// ============================================================
// A watchdog reset reboots the chip and wipes RAM, so a hang in the air
// would otherwise come back up in STATE_IDLE with no idea it was flying —
// and never deploy. This keeps a small flight record in the SNVS
// low-power general-purpose registers (SNVS_LPGPR0-3), which live in the
// always-on SNVS domain and survive any reset short of losing power.
//
// The record is only trusted after a watchdog-3 reset (SRC_SRSR), so a
// normal power-up never resumes from stale data.
// ============================================================

// How often the loop refreshes the record, on top of every state change.
#define RESUME_SAVE_INTERVAL_MS  100

typedef struct {
    FlightState state;
    bool        flight_proven;     // fsm.peak_velocity_ms > MIN_FLIGHT_VELOCITY_MS
    uint32_t    ms_since_launch;   // 0 if launch hasn't been confirmed
    float       ground_hpa;        // alt_est.ground_pressure
    float       peak_altitude_m;   // fsm.peak_altitude_m
} ResumeRecord;

/**
 * Read and clear the reset cause. Call once, first thing in setup().
 * @return true if this boot was caused by the RTWDOG (WDT3) timing out.
 */
bool resume_boot_was_watchdog();

/**
 * Load the saved record. Only meaningful after resume_boot_was_watchdog().
 * @return true if the record is intact and its state is mid-flight
 *         (POWERED through MAIN) — i.e. there is a flight to resume.
 */
bool resume_load(ResumeRecord *out);

/** Write the record. Cheap enough to call at RESUME_SAVE_INTERVAL_MS. */
void resume_save(const ResumeRecord *rec);
