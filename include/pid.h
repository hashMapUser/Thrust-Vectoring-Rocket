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

#define PID_DEFAULT_KP   2.0f   // proportional gain
#define PID_DEFAULT_KI   0.05f  // integral gain
#define PID_DEFAULT_KD   0.5f   // derivative gain

// Output limits [degrees of servo deflection]
#define PID_OUT_MAX      10.0f
#define PID_OUT_MIN     -10.0f

// Integral windup clamp — prevents integrator from building up
// during periods when the output is saturated (e.g. at max deflection)
#define PID_INTEGRAL_MAX  5.0f
#define PID_INTEGRAL_MIN -5.0f

// --------------------------------------------------------
// PID CONTEXT
// --------------------------------------------------------

typedef struct {
    float kp, ki, kd;

    float integral;
    float prev_error;
    float prev_output;

    float out_max, out_min;
    float integral_max, integral_min;

    bool  first_run;  // skip derivative on first call (no prev_error yet)
} PIDController;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise a PID controller with default gains.
 * Call once for pitch and once for yaw.
 */
void pid_init(PIDController *pid);

/**
 * Initialise with custom gains.
 */
void pid_init_gains(PIDController *pid, float kp, float ki, float kd);

/**
 * Compute one PID output step.
 *
 * @param pid       Controller context.
 * @param setpoint  Desired value (0.0 for TVC — we want zero angle error).
 * @param measured  Current value from Madgwick (pitch or yaw in degrees).
 * @param dt        Time since last call [seconds].
 * @return          Servo deflection command [degrees].
 */
float pid_update(PIDController *pid, float setpoint, float measured, float dt);

/**
 * Reset integrator and derivative state.
 * Call when transitioning into powered flight from armed state.
 */
void pid_reset(PIDController *pid);

/**
 * Update gains at runtime (for in-flight gain scheduling if needed).
 */
void pid_set_gains(PIDController *pid, float kp, float ki, float kd);