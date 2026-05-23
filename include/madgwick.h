#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// CONVENTIONS
// --------------------------------------------------------
// Coordinate frame: NED (North-East-Down).
//   - Accelerometer reads (0, 0, +1g) at rest with body Z pointing down.
//   - Magnetometer reads positive X toward magnetic north.
//   - If your sensor is mounted Z-up, negate the Z components of accel
//     and mag before passing them in.
//
// Units:
//   - Gyro:  rad/s  (convert from deg/s by multiplying by PI / 180)
//   - Accel: any (normalized internally)
//   - Mag:   any (normalized internally)
//   - dt:    seconds

// --------------------------------------------------------
// TUNING GAINS
// --------------------------------------------------------
// Defaults from Madgwick's paper: beta ≈ sqrt(3/4) * gyro_noise_rad_per_sec
// assuming about 5 deg/s of gyro measurement error.
// zeta starts at 0 — turn it up after the rest of the filter is verified.
#define MADGWICK_BETA_DEFAULT  0.033f
#define MADGWICK_ZETA_DEFAULT  0.0f

// --------------------------------------------------------
// DATA STRUCTS
// --------------------------------------------------------

typedef struct {
    // Orientation quaternion (w, x, y, z) — earth-to-body rotation
    float q0, q1, q2, q3;

    // Gyro bias estimate [rad/s] — slowly integrated by the zeta term
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;

    // Tuning gains
    float beta;  // gradient descent step size
    float zeta;  // gyro bias drift correction rate
} MadgwickState;

typedef struct {
    float roll;   // [rad]  rotation about body X
    float pitch;  // [rad]  rotation about body Y
    float yaw;    // [rad]  rotation about body Z
} EulerAngles;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialize filter state to identity quaternion and zero bias estimate.
 * Call once in setup() after sensors are initialized.
 */
void madgwick_init(MadgwickState *state, float beta, float zeta);

/**
 * Run one filter tick with accel + gyro + mag (full 9-DOF MARG update).
 *   gx, gy, gz — gyro [rad/s]
 *   ax, ay, az — accel [any unit, normalized internally]
 *   mx, my, mz — mag   [any unit, normalized internally]
 *   dt         — seconds since last update
 */
void madgwick_marg_update(MadgwickState *state,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt);

/**
 * Run one filter tick with accel + gyro only (6-DOF fallback).
 * Use this when a fresh mag sample isn't available — yaw will drift
 * but roll and pitch stay corrected.
 */
void madgwick_imu_update(MadgwickState *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);

/**
 * Convert the current quaternion to roll/pitch/yaw [rad].
 * Computed on demand — call only when needed (telemetry, logging, debug).
 * Watch for gimbal lock near pitch = ±90°.
 */
void madgwick_get_euler(const MadgwickState *state, EulerAngles *euler);