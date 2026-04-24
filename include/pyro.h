#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// PYRO CONFIG
// --------------------------------------------------------

// Output pins — use Teensy 4.0 digital pins rated for the FET drive circuit
// These drive logic-level MOSFETs, NOT the e-match directly
#define PYRO_DROGUE_PIN       6    // drogue ejection charge (apogee)
#define PYRO_MAIN_PIN         8    // main ejection charge (low altitude)

// Continuity check pins — measure resistance across e-match
// Uses Teensy analog input with a voltage divider on the PCB
#define PYRO_DROGUE_CONT_PIN  A0
#define PYRO_MAIN_CONT_PIN    A1

// Continuity ADC thresholds — tune to your voltage divider values
// Above CONT_GOOD = continuity present (e-match connected)
// Below CONT_OPEN = open circuit (e-match missing or broken)
#define PYRO_CONT_GOOD        512   // ADC counts (0–1023)
#define PYRO_CONT_OPEN        100

// Fire pulse duration [ms] — long enough to ignite e-match
// Typical: 500 ms for commercial e-matches
#define PYRO_FIRE_DURATION_MS 500

// Minimum altitude to allow main deploy [m AGL]
// Prevents accidental ground firing if alt estimator glitches
#define PYRO_MAIN_MIN_ALT_M   50.0f

// --------------------------------------------------------
// PYRO STATE
// --------------------------------------------------------

typedef struct {
    bool drogue_continuity;
    bool main_continuity;

    bool drogue_armed;
    bool main_armed;

    bool drogue_fired;
    bool main_fired;

    uint32_t drogue_fire_start_ms;
    uint32_t main_fire_start_ms;
} PyroState;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise pyro output pins and check continuity.
 * Prints continuity status to Serial.
 * Call once in setup() — pyros are NOT armed at this point.
 */
void pyro_init(PyroState *pyro);

/**
 * Check continuity on both channels.
 * Updates pyro->drogue_continuity and pyro->main_continuity.
 */
void pyro_check_continuity(PyroState *pyro);

/**
 * Arm both pyro channels. Only valid when:
 *   - Both channels have continuity
 *   - System is transitioning to STATE_ARMED
 * @return true if arm accepted; false if continuity check fails.
 */
bool pyro_arm(PyroState *pyro);

/**
 * Disarm both channels. Sets outputs LOW.
 */
void pyro_disarm(PyroState *pyro);

/**
 * Fire the drogue charge. Only fires if:
 *   - drogue is armed and not already fired
 * Non-blocking — call pyro_update() every loop to manage pulse timing.
 */
void pyro_fire_drogue(PyroState *pyro);

/**
 * Fire the main charge. Only fires if:
 *   - main is armed and not already fired
 *   - altitude_m > PYRO_MAIN_MIN_ALT_M (safety lockout)
 * Non-blocking.
 */
void pyro_fire_main(PyroState *pyro, float altitude_m);

/**
 * Update pyro pulse timing — call every loop iteration.
 * Cuts the fire pin after PYRO_FIRE_DURATION_MS has elapsed.
 */
void pyro_update(PyroState *pyro);

/**
 * Force all pyro outputs LOW immediately.
 * Call on abort or any fault condition.
 */
void pyro_safe_all(PyroState *pyro);