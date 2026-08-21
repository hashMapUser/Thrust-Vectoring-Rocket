#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board_pins.h"

// --------------------------------------------------------
// PYRO CONFIG
// --------------------------------------------------------

// This flight: PIN_PYRO1_FIRE = main chute. PIN_PYRO2_FIRE unused.

// Fire pulse duration [ms].
// 150 ms is ample for ignition; shorter pulse limits contact energy if match fails open.
#define PYRO_FIRE_DURATION_MS 150

// Minimum altitude to allow main deploy [m AGL]
#define PYRO_MAIN_MIN_ALT_M   50.0f

// EEPROM — persist fired flags across brownouts so a reset cannot re-arm a spent channel.
#define PYRO_EEPROM_ADDR      30   // bytes 30-33 (after gyro bias at 10-23)
#define PYRO_EEPROM_MAGIC     0xF1A5u

// --------------------------------------------------------
// CONTINUITY SENSE
// --------------------------------------------------------

// Sense divider: 15K / 5.1K -> ratio 0.2537.
// Match present  -> sense ~= 0.2537 * V_pack
// Match absent   -> node pulled to GND through 20.1K -> ~0 V
//
// A fixed ADC threshold does not hold across a 2S pack's range:
//   8.4 V -> 2.13 V,  7.4 V -> 1.88 V,  6.0 V -> 1.52 V
// Scale against the measured pack voltage instead.
#define PYRO_CONT_DIVIDER_RATIO  0.2537f
#define PYRO_CONT_FRACTION       0.50f   // fraction of expected value = present

// --------------------------------------------------------
// PYRO STATE
// --------------------------------------------------------

typedef struct {
    bool drogue_armed;
    bool main_armed;

    bool drogue_fired;
    bool main_fired;

    bool drogue_firing;   // pulse currently active (replaces digitalRead on output pin)
    bool main_firing;

    uint32_t drogue_fire_start_ms;
    uint32_t main_fire_start_ms;
} PyroState;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise pyro output pins.
 * Sets outputs LOW BEFORE enabling the output driver (i.MX RT init-order requirement).
 * Loads fired flags from EEPROM — a previously fired channel stays marked fired.
 */
void pyro_init(PyroState *pyro);

/**
 * Arm both pyro channels unconditionally.
 * Continuity is not checked here — call pyro_check_continuity() upstream
 * (e.g. at the ARM_SENSE rising edge) and gate the call to this function
 * on the result, so a bad e-match blocks arming rather than firing.
 * @return always true (kept for API compat with fsm_arm caller).
 */
bool pyro_arm(PyroState *pyro);

/**
 * Check e-match continuity on one pyro channel.
 * Averages 16 ADC samples (the divider's ~3.8K Thevenin impedance is
 * borderline for the Teensy ADC's sampling time) and compares against the
 * expected divider voltage scaled by the measured pack voltage — a fixed
 * threshold does not hold across a 2S pack's charge range.
 *
 * Call this to decide whether to arm, not whether to fire — a sense line
 * that drops during boost must not prevent deployment.
 *
 * @param sense_pin  PIN_PYRO1_SENSE or PIN_PYRO2_SENSE.
 * @param pack_v     Measured pyro pack voltage [V].
 * @return true if continuity is present.
 */
bool pyro_check_continuity(uint8_t sense_pin, float pack_v);

/**
 * Disarm both channels and set outputs LOW.
 */
void pyro_disarm(PyroState *pyro);

/**
 * Fire the drogue charge. Non-blocking. Call pyro_update() every loop.
 */
void pyro_fire_drogue(PyroState *pyro);

/**
 * Fire the main charge if altitude_m > PYRO_MAIN_MIN_ALT_M.
 * Non-blocking. Call pyro_update() every loop.
 */
void pyro_fire_main(PyroState *pyro, float altitude_m);

/**
 * Cut fire pins after PYRO_FIRE_DURATION_MS. Call every loop iteration.
 */
void pyro_update(PyroState *pyro);

/**
 * Force all pyro outputs LOW immediately. Call on abort or fault.
 */
void pyro_safe_all(PyroState *pyro);
