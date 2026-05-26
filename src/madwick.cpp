#include "madgwick.h"
#include <math.h>


/**
 * Imusre that you take into account how the sensor is mounted on your rocket when interpreting the quaternion output.
 * this frame assunes a NED (North-East-Down) coordinate system, which is common for aerospace applications.
 * 
 * For my case this was not true so fixing the output was handle outside of the filter
 * 
 */
// --------------------------------------------------------
// PRIVATE HELPERS
// --------------------------------------------------------

// Conversion factor from radians to degrees for the Euler getter
static const float RAD_TO_DEG = 57.29577951308232f;

// Reject zero-magnitude vectors to avoid divide-by-zero on sensor failure.
// Anything below this magnitude is treated as "no signal."
static const float NORM_EPSILON = 1e-6f;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void madgwick_init(MadgwickState *state, float beta, float zeta) {
    // Identity quaternion — body frame aligned with earth frame
    state->q0 = 1.0f;
    state->q1 = 0.0f;
    state->q2 = 0.0f;
    state->q3 = 0.0f;

    // Zero bias estimate
    state->gyro_bias_x = 0.0f;
    state->gyro_bias_y = 0.0f;
    state->gyro_bias_z = 0.0f;

    // Reference mag field — placeholder, overwritten by the first MARG
    // update with the actual local magnetic field direction.
    state->bx = 1.0f;
    state->bz = 0.0f;

    state->beta = beta;
    state->zeta = zeta;
}

void madgwick_marg_update(MadgwickState *state,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt)
{
    // Load quaternion and reference field into locals for readability
    float q0 = state->q0;
    float q1 = state->q1;
    float q2 = state->q2;
    float q3 = state->q3;
    float bx = state->bx;
    float bz = state->bz;

    // --- Normalize accelerometer ---
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < NORM_EPSILON) return;   // sensor failure; skip this tick
    float recip = 1.0f / norm;
    ax *= recip;
    ay *= recip;
    az *= recip;

    // --- Normalize magnetometer ---
    norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm < NORM_EPSILON) return;
    recip = 1.0f / norm;
    mx *= recip;
    my *= recip;
    mz *= recip;

    // --- Precompute repeated quaternion products ---
    float q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
    float q2q2 = q2*q2, q2q3 = q2*q3;
    float q3q3 = q3*q3;

    // --- Objective function f (predicted - measured) ---
    // Accel half: predicted gravity in body frame minus measured accel
    float f1 = 2.0f*(q1q3 - q0q2) - ax;
    float f2 = 2.0f*(q0q1 + q2q3) - ay;
    float f3 = 1.0f - 2.0f*(q1q1 + q2q2) - az;

    // Mag half: predicted [bx, 0, bz] rotated into body frame minus measured
    float f4 = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2) - mx;
    float f5 = 2.0f*bx*(q1q2 - q0q3)        + 2.0f*bz*(q0q1 + q2q3) - my;
    float f6 = 2.0f*bx*(q0q2 + q1q3)        + 2.0f*bz*(0.5f - q1q1 - q2q2) - mz;

    // --- Gradient deltaf = J^T * f
    float s0 = -2.0f*q2*f1 + 2.0f*q1*f2
               - 2.0f*bz*q2*f4
               + 2.0f*(-bx*q3 + bz*q1)*f5
               + 2.0f*bx*q2*f6;
    float s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 4.0f*q1*f3
               + 2.0f*bz*q3*f4
               + 2.0f*(bx*q2 + bz*q0)*f5
               + 2.0f*(bx*q3 - 2.0f*bz*q1)*f6;
    float s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3
               + 2.0f*(-2.0f*bx*q2 - bz*q0)*f4
               + 2.0f*(bx*q1 + bz*q3)*f5
               + 2.0f*(bx*q0 - 2.0f*bz*q2)*f6;
    float s3 =  2.0f*q1*f1 + 2.0f*q2*f2
               + 2.0f*(-2.0f*bx*q3 + bz*q1)*f4
               + 2.0f*(-bx*q0 + bz*q2)*f5
               + 2.0f*bx*q1*f6;

    // --- Normalize gradient ---
    norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm > NORM_EPSILON) {
        recip = 1.0f / norm;
        s0 *= recip;
        s1 *= recip;
        s2 *= recip;
        s3 *= recip;
    }

    // --- Gyro error in body frame (vector part of 2 * q* X deltaf) ---
    // zeta-based bias drift correction.
    float we_x = 2.0f*(q0*s1 - q1*s0 - q2*s3 + q3*s2);
    float we_y = 2.0f*(q0*s2 + q1*s3 - q2*s0 - q3*s1);
    float we_z = 2.0f*(q0*s3 - q1*s2 + q2*s1 - q3*s0);

    // --- Update gyro bias estimate ---
    state->gyro_bias_x += state->zeta * we_x * dt;
    state->gyro_bias_y += state->zeta * we_y * dt;
    state->gyro_bias_z += state->zeta * we_z * dt;

    // --- Subtract bias from measured gyro ---
    gx -= state->gyro_bias_x;
    gy -= state->gyro_bias_y;
    gz -= state->gyro_bias_z;

    // --- Quaternion derivative from gyro: q̇_ω = 0.5 * q ⊗ [0, gx, gy, gz] ---
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // --- Combine with gradient correction and integrate ---
    q0 += (qDot0 - state->beta * s0) * dt;
    q1 += (qDot1 - state->beta * s1) * dt;
    q2 += (qDot2 - state->beta * s2) * dt;
    q3 += (qDot3 - state->beta * s3) * dt;

    // --- Normalize quaternion ---
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    recip = 1.0f / norm;
    state->q0 = q0 * recip;
    state->q1 = q1 * recip;
    state->q2 = q2 * recip;
    state->q3 = q3 * recip;

    // --- Update earth-frame magnetic reference from new quaternion ---
    // Rotate measured mag from body to earth frame: h = R(q) * m
    q0 = state->q0; q1 = state->q1; q2 = state->q2; q3 = state->q3;
    q0q1 = q0*q1; q0q2 = q0*q2; q0q3 = q0*q3;
    q1q1 = q1*q1; q1q2 = q1*q2; q1q3 = q1*q3;
    q2q2 = q2*q2; q2q3 = q2*q3;
    q3q3 = q3*q3;

    float hx = 2.0f*(mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
    float hy = 2.0f*(mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));
    float hz = 2.0f*(mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

    // Project to horizontal magnitude + vertical component
    state->bx = sqrtf(hx*hx + hy*hy);
    state->bz = hz;
}

void madgwick_imu_update(MadgwickState *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float q0 = state->q0;
    float q1 = state->q1;
    float q2 = state->q2;
    float q3 = state->q3;

    // --- Normalize accelerometer; fall back to gyro-only if it fails ---
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    float recip;
    bool accel_valid = (norm >= NORM_EPSILON);
    if (accel_valid) {
        recip = 1.0f / norm;
        ax *= recip;
        ay *= recip;
        az *= recip;
    }

    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    float we_x = 0.0f, we_y = 0.0f, we_z = 0.0f;

    if (accel_valid) {
        // --- Objective function (accel only, 3 components) ---
        float f1 = 2.0f*(q1*q3 - q0*q2) - ax;
        float f2 = 2.0f*(q0*q1 + q2*q3) - ay;
        float f3 = 1.0f - 2.0f*(q1*q1 + q2*q2) - az;

        // --- Gradient from 3×4 Jacobian ---
        s0 = -2.0f*q2*f1 + 2.0f*q1*f2;
        s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 4.0f*q1*f3;
        s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3;
        s3 =  2.0f*q1*f1 + 2.0f*q2*f2;

        // --- Normalize gradient ---
        norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm > NORM_EPSILON) {
            recip = 1.0f / norm;
            s0 *= recip;
            s1 *= recip;
            s2 *= recip;
            s3 *= recip;
        }

        // --- Gyro error for zeta ---
        we_x = 2.0f*(q0*s1 - q1*s0 - q2*s3 + q3*s2);
        we_y = 2.0f*(q0*s2 + q1*s3 - q2*s0 - q3*s1);
        we_z = 2.0f*(q0*s3 - q1*s2 + q2*s1 - q3*s0);
    }

    // --- Update gyro bias estimate (zero if accel failed — no correction) ---
    state->gyro_bias_x += state->zeta * we_x * dt;
    state->gyro_bias_y += state->zeta * we_y * dt;
    state->gyro_bias_z += state->zeta * we_z * dt;

    gx -= state->gyro_bias_x;
    gy -= state->gyro_bias_y;
    gz -= state->gyro_bias_z;

    // --- Gyro-derived quaternion derivative ---
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // --- Combine and integrate (gradient term is zero if accel failed) ---
    q0 += (qDot0 - state->beta * s0) * dt;
    q1 += (qDot1 - state->beta * s1) * dt;
    q2 += (qDot2 - state->beta * s2) * dt;
    q3 += (qDot3 - state->beta * s3) * dt;

    // --- Normalize quaternion ---
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    recip = 1.0f / norm;
    state->q0 = q0 * recip;
    state->q1 = q1 * recip;
    state->q2 = q2 * recip;
    state->q3 = q3 * recip;
}

void madgwick_get_euler(const MadgwickState *state, EulerAngles *euler) {
    float q0 = state->q0, q1 = state->q1, q2 = state->q2, q3 = state->q3;

    // Roll (X axis)
    float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (Y axis) — clamp asin argument to avoid NaN at the poles
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    euler->pitch = asinf(sinp) * RAD_TO_DEG;

    // Yaw (Z axis)
    float siny_cosp = 2.0f * (q0*q3 + q1*q2);
    float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}