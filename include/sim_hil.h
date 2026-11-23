#pragma once

// ============================================================
// BENCH SIMULATION — env:flight_sim (-D SIM_MODE)
// ============================================================
// The real flight firmware — main_control_loop.cpp, the state machine,
// attitude filter, altitude estimator, PID, pyro logic, logger, flash, SD,
// watchdog, telemetry — runs unchanged on the Teensy. Only the sensor
// drivers are swapped out: src/sim_hil.cpp stands in for the LSM6DSOX,
// LPS22HB and MMC5603NJ and feeds readings from a simulated rocket
// (include/sim_model.h) flying the scenario picked over serial.
//
// Safety: pyro outputs are never driven in this build (src/pyro.cpp), and
// its fired flags and run record live in their own EEPROM slots, apart
// from the flight firmware's. Servos DO move.
//
// Serial commands (on top of the flight firmware's own):
//   1-8  pick a scenario      L  launch (5 s countdown)
//   ?    menu                 Z  reboot for the next run
// After landing (or a time limit), a PASS/FAIL report grades each event
// against what the simulated rocket really did.
// ============================================================

#include "flight_sm.h"
#include "pyro.h"

/** End of setup(): menu, and a clean slate for a new simulated flight. */
void sim_after_setup(PyroState *pyros, bool resuming);

/** A serial character the flight firmware doesn't use itself. */
void sim_command(char c);

/** Once per loop tick: open the chute when the main fires, record events, report. */
void sim_observe(const FlightSM *fsm, bool main_fired, float altitude_m);
