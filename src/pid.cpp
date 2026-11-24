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
    pid->d_filt     = 0.0f;
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
    // kd * (rate of change of the error), low-passed by PID_D_FILTER_ALPHA.
    // The TVC setpoint is always 0, so that's -kd * (rate of the measured tilt).
    if (!pid->first_run) {
        float rate = (error - pid->prev_error) / dt;
        pid->d_filt = PID_D_FILTER_ALPHA * rate + (1.0f - PID_D_FILTER_ALPHA) * pid->d_filt;
    }
    float d_term = pid->kd * pid->d_filt;
    pid->first_run  = false;
    pid->prev_error = error;

    // ── OUTPUT ──
    float output = p_term + i_term + d_term;
    output = constrain(output, -pid->out_limit, pid->out_limit);

    pid->prev_output = output;
    return output;
}