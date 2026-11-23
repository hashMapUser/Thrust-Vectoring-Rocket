#pragma once

// ============================================================
// SIMULATED ROCKET — physics and sensor readings for the bench
// simulation build (env:flight_sim). Plain C++, no Arduino: it can be
// built for a PC too.
//
// A 1-D vertical flight on the certified Estes F-15 curve, with drag, a
// main chute that opens when the firmware fires it, and a prescribed
// attitude (a launch wobble the TVC would see, a gravity turn in coast,
// swinging under the chute, falling over on landing). It produces what
// the IMU, baro and magnetometer chips would report — in each chip's own
// axes — so the flight code's own axis remapping is exercised too.
//
// The attitude is NOT affected by the gimbal: this checks detection,
// recovery and logging, not whether the TVC steers the right way (bench
// test I and a swing test do that).
// ============================================================

#include <stdint.h>
#include <stdbool.h>

typedef enum : uint8_t {
    SIM_FAULT_NONE = 0,
    SIM_FAULT_PAD_BUMP,     // a knock on the pad at T+0, no ignition
    SIM_FAULT_BARO_DEAD,    // barometer returns nothing from fault_t
    SIM_FAULT_IMU_DEAD,     // IMU returns invalid data from fault_t
    SIM_FAULT_WDT_RESET,    // firmware hangs at fault_t; the watchdog reboots it
} SimFault;

typedef struct {
    const char *name;
    const char *about;       // one line for the menu
    float   mass_kg;         // liftoff mass
    float   thrust_scale;    // 1.0 = certified F-15 curve
    float   swing_deg;       // swing amplitude under the main chute
    SimFault fault;
    float   fault_t;         // s after ignition
} SimScenario;

extern const SimScenario SIM_SCENARIOS[];
extern const int         SIM_SCENARIO_COUNT;

#define SIM_BURN_S  3.45f    // F-15 burn time

typedef enum : uint8_t { SIM_PAD, SIM_FLIGHT, SIM_CHUTE, SIM_GROUND } SimPhase;

typedef struct {
    const SimScenario *sc;
    double   t;              // s after ignition the state below is for
    float    h, v;           // m above the pad, m/s up
    float    mass;
    float    impulse;        // N*s delivered so far
    SimPhase phase;
    // True events, s after ignition (< 0 = hasn't happened)
    double   t_liftoff, t_apogee, t_deploy, t_touchdown;
    float    apogee_m;
    // Lean at deploy / touchdown, so the attitude stays continuous
    float    dep_lean_n, dep_lean_e, td_lean_n, td_lean_e;
    uint32_t rng;
} SimModel;

typedef struct {
    float ax, ay, az;        // accelerometer, LSM6DSOX chip axes [g]
    float gx, gy, gz;        // gyro, LSM6DSOX chip axes [deg/s]
    float mx, my, mz;        // magnetometer, MMC5603NJ chip axes [Gauss]
    float pressure_hpa;
    float temperature_c;
} SimSensors;

void  sim_model_init(SimModel *m, const SimScenario *sc, uint32_t seed);

/** Integrate to t (s after ignition; negative = still on the pad). Time only moves forward. */
void  sim_model_advance(SimModel *m, double t);

/** The firmware fired the main charge: open the chute now. */
void  sim_model_deploy(SimModel *m);

/** What the sensors read at the model's current time, with noise. */
void  sim_model_sensors(SimModel *m, SimSensors *out);
