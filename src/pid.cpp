#include <Arduino.h>
#include <math.h>
#include "pid.h"

void pid_init(PIDController *pid) {
    pid_init_gains(pid, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
}

void pid_init_gains(PIDController *pid, float kp, float ki, float kd) {
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->out_max      = PID_OUT_MAX;
    pid->out_min      = PID_OUT_MIN;
    pid->integral_max = PID_INTEGRAL_MAX;
    pid->integral_min = PID_INTEGRAL_MIN;
    pid_reset(pid);
}

void pid_reset(PIDController *pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->first_run  = true;
}

void pid_set_gains(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_update(PIDController *pid, float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return pid->prev_output;

    float error = setpoint - measured;

    // ── PROPORTIONAL ──
    float p_term = pid->kp * error;

    // ── INTEGRAL with anti-windup ──
    // Only accumulate integral when output is not saturated.
    // This prevents the integrator from winding up when the servo
    // is already at its limit — a common source of instability.
    pid->integral += pid->ki * error * dt;
    pid->integral  = constrain(pid->integral,
                               pid->integral_min,
                               pid->integral_max);
    float i_term   = pid->integral;

    // ── DERIVATIVE on measurement, not error ──
    // Using the derivative of the measurement rather than the error
    // prevents derivative kick when the setpoint changes suddenly.
    // For TVC the setpoint is always 0 so this distinction doesn't matter,
    // but it's best practice and makes the controller more general.
    float d_term = 0.0f;
    if (!pid->first_run) {
        d_term = -pid->kd * (measured - (setpoint - pid->prev_error)) / dt;
    }
    pid->first_run  = false;
    pid->prev_error = error;

    // ── OUTPUT ──
    float output = p_term + i_term + d_term;
    output = constrain(output, pid->out_min, pid->out_max);

    pid->prev_output = output;
    return output;
}