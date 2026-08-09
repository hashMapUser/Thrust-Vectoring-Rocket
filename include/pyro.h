#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "board_pins.h"

// --------------------------------------------------------
// PYRO CONFIG
// --------------------------------------------------------

// PIN_PYRO1_FIRE (32) and PIN_PYRO2_FIRE (31) come from board_pins.h.
// This flight: PIN_PYRO1_FIRE = main chute. PIN_PYRO2_FIRE unused.
// H5: PYRO1_SENSE (30) and PYRO2_SENSE (33) have no ADC — continuity sensing disabled.

// Fire pulse duration [ms].
// 150 ms is ample for ignition; shorter pulse limits contact energy if match fails open.
#define PYRO_FIRE_DURATION_MS 150

// Minimum altitude to allow main deploy [m AGL]
#define PYRO_MAIN_MIN_ALT_M   50.0f

// EEPROM — persist fired flags across brownouts so a reset cannot re-arm a spent channel.
#define PYRO_EEPROM_ADDR      30   // bytes 30-33 (after gyro bias at 10-23)
#define PYRO_EEPROM_MAGIC     0xF1A5u

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
 * Continuity sensing is disabled (H5: sense pins have no ADC this flight).
 */
void pyro_init(PyroState *pyro);

/**
 * Arm both pyro channels unconditionally.
 * Continuity check removed — use bench meter before flight (H5).
 * @return always true (kept for API compat with fsm_arm caller).
 */
bool pyro_arm(PyroState *pyro);

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
