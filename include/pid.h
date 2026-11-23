#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// DEFAULT GAINS
// --------------------------------------------------------
// These are starting points only — MUST be tuned on a thrust stand.
// Do not fly with untuned gains.
//
// Tuning order: P first (increase until oscillation, halve it),
// then D (add damping), then I last (remove steady-state error).

#define PID_DEFAULT_KP   0.34f   // proportional gain
#define PID_DEFAULT_KI   0.215f  // integral gain
#define PID_DEFAULT_KD   0.05f   // derivative gain

// The output limit is set per controller in pid_init() — each axis gets
// its full servo travel, SERVO_*_MAX_ANGLE_DEG from servo_driver.h.

// Integral term clamp [deg of nozzle], symmetric. pid_init() lowers it to
// the axis's output limit where that's smaller (pitch: 7), since integral
// past the output limit can't move the nozzle further — it only winds up.
#define PID_INTEGRAL_LIMIT  8.5f

// --------------------------------------------------------
// PID CONTEXT
// --------------------------------------------------------

typedef struct {
    float kp, ki, kd;

    float integral;
    float prev_error;
    float prev_output;

    float out_limit;        // output clamped to ±out_limit [deg]
    float integral_limit;   // integral clamped to ±integral_limit [deg]

    bool  first_run;  // skip derivative on first call (no prev_error yet)
} PIDController;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise a PID controller with default gains.
 * Call once for pitch and once for yaw.
 *
 * @param out_limit  Symmetric output limit [deg]: pass the axis's
 *                   SERVO_*_MAX_ANGLE_DEG so it can use the full travel.
 */
void pid_init(PIDController *pid, float out_limit);

/**
 * Initialise with custom gains.
 */
void pid_init_gains(PIDController *pid, float kp, float ki, float kd, float out_limit);

/**
 * Compute one PID output step.
 *
 * @param pid       Controller context.
 * @param setpoint  Desired value (0.0 for TVC — we want zero angle error).
 * @param measured  Current value from Madgwick (pitch or yaw in degrees).
 * @param dt        Time since last call [seconds].
 * @return          Nozzle deflection command [degrees].
 */
float pid_update(PIDController *pid, float setpoint, float measured, float dt);

/**
 * Reset integrator and derivative state.
 * Call when TVC switches on (the flight loop does it on the rising edge
 * of fsm.tvc_enabled).
 */
void pid_reset(PIDController *pid);

/**
 * Update gains at runtime (for in-flight gain scheduling if needed).
 */
void pid_set_gains(PIDController *pid, float kp, float ki, float kd);