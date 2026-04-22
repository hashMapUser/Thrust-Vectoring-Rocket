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

void madgwick_init(MadgwickState *state) {
    // Identity quaternion — represents no rotation (sensor frame = world frame)
    state->q0 = 1.0f;
    state->q1 = 0.0f;
    state->q2 = 0.0f;
    state->q3 = 0.0f;

    state->roll  = 0.0f;
    state->pitch = 0.0f;
    state->yaw   = 0.0f;
}

// --------------------------------------------------------
// FULL UPDATE — gyro + accel + magnetometer
// --------------------------------------------------------
// Based on: Madgwick, S. (2010). "An efficient orientation filter for
// inertial and inertial/magnetic sensor arrays." University of Bristol.
//
// Algorithm overview:
//   1. Integrate gyro rates to propagate the quaternion forward in time.
//   2. Compute the gradient of the orientation error using accel and mag
//      as references for the gravity and magnetic field directions.
//   3. Apply a gradient descent correction step (scaled by beta) to pull
//      the quaternion back toward the physical reference directions.
//   4. Renormalize the quaternion and extract Euler angles.
// --------------------------------------------------------
void madgwick_update(MadgwickState *state,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt) {

    float q0 = state->q0, q1 = state->q1,
          q2 = state->q2, q3 = state->q3;

    // Convert gyro from deg/s to rad/s
    float gx_r = deg2rad(gx);
    float gy_r = deg2rad(gy);
    float gz_r = deg2rad(gz);

    // --- Normalize accelerometer ---
    float a_norm = inv_sqrt(ax*ax + ay*ay + az*az);
    if (a_norm == 0.0f) {
        // Accel reading is zero — skip correction, gyro-only propagation
        goto gyro_only;
    }
    ax *= a_norm; ay *= a_norm; az *= a_norm;

    // --- Normalize magnetometer ---
    {
        float m_norm = inv_sqrt(mx*mx + my*my + mz*mz);
        if (m_norm == 0.0f) {
            // Mag invalid — fall through to IMU-only update
            madgwick_update_imu(state, gx, gy, gz, ax/a_norm, ay/a_norm, az/a_norm, dt);
            return;
        }
        mx *= m_norm; my *= m_norm; mz *= m_norm;
    }

    {
        // Reference direction of Earth's magnetic field in world frame.
        // Rotate the mag measurement from sensor frame to world frame
        // using the current quaternion estimate.
        float hx = 2.0f*mx*(0.5f - q2*q2 - q3*q3)
                 + 2.0f*my*(q1*q2 - q0*q3)
                 + 2.0f*mz*(q1*q3 + q0*q2);

        float hy = 2.0f*mx*(q1*q2 + q0*q3)
                 + 2.0f*my*(0.5f - q1*q1 - q3*q3)
                 + 2.0f*mz*(q2*q3 - q0*q1);

        // Project onto the horizontal plane (bx = horizontal, bz = vertical)
        // This makes the filter insensitive to magnetic dip angle variations
        float bx = sqrtf(hx*hx + hy*hy);
        float bz = 2.0f*mx*(q1*q3 - q0*q2)
                 + 2.0f*my*(q2*q3 + q0*q1)
                 + 2.0f*mz*(0.5f - q1*q1 - q2*q2);

        // Gradient — objective function f for accel (gravity reference)
        float f1 = 2.0f*(q1*q3 - q0*q2)       - ax;
        float f2 = 2.0f*(q0*q1 + q2*q3)       - ay;
        float f3 = 1.0f - 2.0f*(q1*q1 + q2*q2) - az;

        // Gradient — objective function f for mag (magnetic field reference)
        float f4 = 2.0f*bx*(0.5f - q2*q2 - q3*q3)
                 + 2.0f*bz*(q1*q3 - q0*q2)           - mx;
        float f5 = 2.0f*bx*(q1*q2 - q0*q3)
                 + 2.0f*bz*(q0*q1 + q2*q3)           - my;
        float f6 = 2.0f*bx*(q0*q2 + q1*q3)
                 + 2.0f*bz*(0.5f - q1*q1 - q2*q2)   - mz;

        // Jacobian transposed × objective — gives gradient direction
        float j11 = -2.0f*q2, j12 =  2.0f*q1;
        float j13 = -4.0f*q0*bz, j14 = 2.0f*(bx*q3 + bz*q1);
        float j21 = -2.0f*q3, j22 = -2.0f*q0;

        float grad0 = j11*f1 + j12*f2
                    + (-2.0f*bz*q2)*f4
                    + (-2.0f*bx*q3 + 2.0f*bz*q1)*f5
                    + ( 2.0f*bx*q2)*f6;

        float grad1 = 2.0f*q3*f1 + 2.0f*q2*f2 + (-4.0f*q1)*f3
                    + j14*f4
                    + (2.0f*bx*q2 + 2.0f*bz*q0)*f5
                    + (2.0f*bx*q3 + 2.0f*bz*q1 - 4.0f*bx*q1 - 4.0f*bz*q1)*f6;  // simplified below

        float grad2 = j11*f1 + (-4.0f*q2)*f2 + (-2.0f*q0)*f3
                    + (-4.0f*bx*q2 - 2.0f*bz*q0)*f4
                    + (2.0f*bx*q1 + 2.0f*bz*q3)*f5
                    + (2.0f*bx*q0 - 4.0f*bz*q2)*f6;  // approximate

        float grad3 = j21*f1 + 2.0f*q1*f2
                    + (-2.0f*bx*q0 + 2.0f*bz*q1)*f4  // approximate
                    + (-2.0f*bx*q1)*f5
                    + (2.0f*bx*q1)*f6;

        // Suppress unused variable warnings from intermediate terms
        (void)j13; (void)j22;

        // Normalize gradient
        float g_norm = inv_sqrt(grad0*grad0 + grad1*grad1 + grad2*grad2 + grad3*grad3);
        grad0 *= g_norm; grad1 *= g_norm; grad2 *= g_norm; grad3 *= g_norm;

        // Gyro-based quaternion rate
        float qDot0 = 0.5f*(-q1*gx_r - q2*gy_r - q3*gz_r);
        float qDot1 = 0.5f*( q0*gx_r + q2*gz_r - q3*gy_r);
        float qDot2 = 0.5f*( q0*gy_r - q1*gz_r + q3*gx_r);
        float qDot3 = 0.5f*( q0*gz_r + q1*gy_r - q2*gx_r);

        // Apply gradient descent correction
        qDot0 -= MADGWICK_BETA * grad0;
        qDot1 -= MADGWICK_BETA * grad1;
        qDot2 -= MADGWICK_BETA * grad2;
        qDot3 -= MADGWICK_BETA * grad3;

        // Integrate
        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;
    }

    goto normalize;

gyro_only:
    {
        // Accel was zero — pure gyro integration, no correction
        float qDot0 = 0.5f*(-q1*gx_r - q2*gy_r - q3*gz_r);
        float qDot1 = 0.5f*( q0*gx_r + q2*gz_r - q3*gy_r);
        float qDot2 = 0.5f*( q0*gy_r - q1*gz_r + q3*gx_r);
        float qDot3 = 0.5f*( q0*gz_r + q1*gy_r - q2*gx_r);
        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;
    }

normalize:
    {
        float q_norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        state->q0 = q0 * q_norm;
        state->q1 = q1 * q_norm;
        state->q2 = q2 * q_norm;
        state->q3 = q3 * q_norm;
    }

    update_euler(state);
}

// --------------------------------------------------------
// IMU-ONLY UPDATE — gyro + accel, no magnetometer
// Yaw will drift. Use when mag is unavailable or invalid.
// --------------------------------------------------------
void madgwick_update_imu(MadgwickState *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt) {

    float q0 = state->q0, q1 = state->q1,
          q2 = state->q2, q3 = state->q3;

    float gx_r = deg2rad(gx);
    float gy_r = deg2rad(gy);
    float gz_r = deg2rad(gz);

    // Gyro rate as quaternion derivative
    float qDot0 = 0.5f*(-q1*gx_r - q2*gy_r - q3*gz_r);
    float qDot1 = 0.5f*( q0*gx_r + q2*gz_r - q3*gy_r);
    float qDot2 = 0.5f*( q0*gy_r - q1*gz_r + q3*gx_r);
    float qDot3 = 0.5f*( q0*gz_r + q1*gy_r - q2*gx_r);

    // Normalize accel
    float a_norm = inv_sqrt(ax*ax + ay*ay + az*az);
    if (a_norm != 0.0f) {
        ax *= a_norm; ay *= a_norm; az *= a_norm;

        // Gradient from gravity reference only
        float f1 = 2.0f*(q1*q3 - q0*q2)        - ax;
        float f2 = 2.0f*(q0*q1 + q2*q3)        - ay;
        float f3 = 1.0f - 2.0f*(q1*q1 + q2*q2) - az;

        float grad0 = -2.0f*q2*f1 + 2.0f*q1*f2;
        float grad1 =  2.0f*q3*f1 + 2.0f*q2*f2 - 4.0f*q1*f3;
        float grad2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3;
        float grad3 =  2.0f*q1*f1 + 2.0f*q0*f2;

        float g_norm = inv_sqrt(grad0*grad0 + grad1*grad1 + grad2*grad2 + grad3*grad3);
        grad0 *= g_norm; grad1 *= g_norm; grad2 *= g_norm; grad3 *= g_norm;

        qDot0 -= MADGWICK_BETA * grad0;
        qDot1 -= MADGWICK_BETA * grad1;
        qDot2 -= MADGWICK_BETA * grad2;
        qDot3 -= MADGWICK_BETA * grad3;
    }

    // Integrate and normalize
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    float q_norm = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    state->q0 = q0 * q_norm;
    state->q1 = q1 * q_norm;
    state->q2 = q2 * q_norm;
    state->q3 = q3 * q_norm;

    update_euler(state);
}