#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// DEFAULT GAINS
// --------------------------------------------------------
// One set for both axes, tuned in tvc_sim for the 775 g build: CAD MOI
// 0.0148 kg*m^2, pivot-to-CG 141 mm pitch / 183 mm yaw, CP 235 mm ahead of
// the CG (finless, nose cone only), 82.55 mm OD. The servo speed in
// tvc_sim is still an estimate — re-check once it's known.

#define PID_DEFAULT_KP   0.8f    // proportional gain
#define PID_DEFAULT_KI   0.215f  // integral gain
#define PID_DEFAULT_KD   0.10f   // derivative gain

// Derivative low-pass, applied once per update (every 8 ms):
// d = ALPHA * new + (1 - ALPHA) * d. 1 = no filter. 0.5 keeps attitude
// noise from making the slow pitch servo chase the D term.
#define PID_D_FILTER_ALPHA  0.5f

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
    float d_filt;           // low-passed error rate [deg/s]

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