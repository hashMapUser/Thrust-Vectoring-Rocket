#include <math.h>
#include "madgwick.h"


// --------------------------------------------------------
// PRIVATE HELPERS
// --------------------------------------------------------

// Fast inverse square root — avoids sqrtf() + division in the normalize step.
// Uses the classic Quake III trick with one Newton-Raphson iteration.
static inline float inv_sqrt(float x) {
    float halfx = 0.5f * x;
    union { float f; uint32_t i; } conv = { .f = x };
    conv.i = 0x5F3759DFu - (conv.i >> 1);
    conv.f *= (1.5f - halfx * conv.f * conv.f);  // one N-R iteration
    conv.f *= (1.5f - halfx * conv.f * conv.f);  // second iteration — improves accuracy
    return conv.f;
}

// Degrees to radians
static inline float deg2rad(float deg) {
    return deg * 0.01745329251994f;   // deg * (π / 180)
}

// --------------------------------------------------------
// EULER EXTRACTION
// --------------------------------------------------------

// Derive ZYX Euler angles from the current quaternion and store in state.
// Called at the end of every update so state always has fresh angles.
//
// ZYX convention (yaw → pitch → roll) — aerospace standard:
//   roll  = atan2(2(q0*q1 + q2*q3),  1 - 2(q1² + q2²))
//   pitch = asin (2(q0*q2 - q3*q1))
//   yaw   = atan2(2(q0*q3 + q1*q2),  1 - 2(q2² + q3²))
//
// Pitch is clamped to ±90° (gimbal lock at the poles is unavoidable with Euler).
// Use the quaternion directly in the control loop to avoid this singularity.
static void update_euler(MadgwickState *s) {
    float q0 = s->q0, q1 = s->q1, q2 = s->q2, q3 = s->q3;

    // Roll (X axis rotation)
    float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
    s->roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

    // Pitch (Y axis rotation) — clamp argument to [-1, 1] to guard against NaN
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    s->pitch = asinf(sinp) * (180.0f / (float)M_PI);

    // Yaw (Z axis rotation)
    float siny_cosp = 2.0f * (q0*q3 + q1*q2);
    float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
    s->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void madgwick_init(MadgwickState *state, float beta, float zeta) {
    state->beta = beta;
    state->zeta = zeta;

    state->q0 = 1.0f;  // identity quaternion
    state->q1 = 0.0f;
    state->q2 = 0.0f;
    state->q3 = 0.0f;

    //gryo biases
    state->gyro_bias_x = 0.0f;
    state->gyro_bias_y = 0.0f;
    state->gyro_bias_z = 0.0f;

    //euler angles
    state->roll  = 0.0f;
    state->pitch = 0.0f;
    state->yaw   = 0.0f;