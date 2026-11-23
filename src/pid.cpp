#include <Arduino.h>
#include <math.h>
#include "pid.h"

void pid_init(PIDController *pid, float out_limit) {
    pid_init_gains(pid, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD, out_limit);
}

void pid_init_gains(PIDController *pid, float kp, float ki, float kd, float out_limit) {
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->out_limit      = fabsf(out_limit);
    pid->integral_limit = fminf(PID_INTEGRAL_LIMIT, pid->out_limit);
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

    // ── INTEGRAL, clamped to ±integral_limit so it can't wind up ──
    pid->integral += pid->ki * error * dt;
    pid->integral  = constrain(pid->integral, -pid->integral_limit, pid->integral_limit);
    float i_term   = pid->integral;

    // ── DERIVATIVE ──
    // Works out to kd * (change in error) / dt. The TVC setpoint is always
    // 0, so that's the same as -kd * (rate of the measured tilt).
    float d_term = 0.0f;
    if (!pid->first_run) {
        d_term = -pid->kd * (measured - (setpoint - pid->prev_error)) / dt;
    }
    pid->first_run  = false;
    pid->prev_error = error;

    // ── OUTPUT ──
    float output = p_term + i_term + d_term;
    output = constrain(output, -pid->out_limit, pid->out_limit);

    pid->prev_output = output;
    return output;
}