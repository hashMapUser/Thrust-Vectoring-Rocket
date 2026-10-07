#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// MADGWICK MARG / IMU ORIENTATION FILTER
// --------------------------------------------------------
// Implementation of the Madgwick 2010 filter with gyro bias
// drift correction (zeta term), per:
//   Madgwick, S. O. H. (2010). "An efficient orientation filter
//   for inertial and inertial/magnetic sensor arrays."
//
// Designed for a fixed-rate scheduler (≥50 Hz). At 125 Hz pass
// dt = 1.0f / 125.0f, or measure dt with micros() each tick.
//
// --------------------------------------------------------
// CONVENTIONS
// --------------------------------------------------------
// Coordinate frame: NED (North-East-Down).
//   - Accelerometer reads (0, 0, +1g) at rest with body Z pointing down.
//   - Magnetometer X axis points toward magnetic north when level.
//   - If your sensor is mounted Z-up, negate the Z component of accel
//     and mag before passing them in, OR rotate the output quaternion
//     downstream.
//
// Units:
//   - Gyro:  rad/s  (convert from the LSM6DSOX driver's deg/s by
//                    multiplying by 0.01745329252f)
//   - Accel: any unit (normalized internally; sign matters)
//   - Mag:   any unit (normalized internally; sign matters)
//   - dt:    seconds
//
// Quaternion convention: (q0, q1, q2, q3) = (w, x, y, z), earth-to-body.

// --------------------------------------------------------
// TUNING GAINS
// --------------------------------------------------------
// Defaults from Madgwick's paper:
//   beta = sqrt(3/4) * gyro_meas_error
// For MEMS IMUs, 0.033 - 0.05 is a good starting range.
// zeta starts at 0 — bring the filter up with zeta=0 first, then
// raise to ~0.001-0.01 once you've verified bias estimates converge
// near your static calibration values.
#define MADGWICK_BETA_DEFAULT  0.033f
#define MADGWICK_ZETA_DEFAULT  0.0f

// --------------------------------------------------------
// DATA STRUCTS
// --------------------------------------------------------

typedef struct {
    // Orientation quaternion (w, x, y, z) — earth-to-body rotation
    float q0, q1, q2, q3;

    // Gyro bias estimate [rad/s] — integrated by the zeta term over time.
    // After convergence (~30s of still operation with zeta > 0), these
    // should match the static bias found by lsm6dsox_calibrate_gyro().
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;

    // Reference magnetic field in earth frame:
    //   bx = horizontal (north-pointing) magnitude
    //   bz = vertical (down-pointing) component
    // Updated each MARG tick from the measured mag rotated into earth frame.
    float bx, bz;

    // Tuning gains
    float beta;  // gradient descent step size (accel/mag trust)
    float zeta;  // gyro bias drift correction rate
} MadgwickState;

typedef struct {
    float roll;   // [deg]  rotation about body X
    float pitch;  // [deg]  rotation about body Y
    float yaw;    // [deg]  rotation about body Z
} EulerAngles;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialize filter state to identity quaternion, zero bias estimate,
 * and default reference magnetic field. Call once in setup().
 *
 * @param state  Filter state to initialize.
 * @param beta   Gradient gain. Use MADGWICK_BETA_DEFAULT to start.
 * @param zeta   Bias drift gain. Use MADGWICK_ZETA_DEFAULT (zero) initially.
 */
void madgwick_init(MadgwickState *state, float beta, float zeta);

/**
 * Full 9-DOF MARG update (gyro + accel + mag).
 * Updates the quaternion, gyro bias estimate, and earth-frame magnetic
 * reference in place.
 *
 * @param gx, gy, gz  Gyro readings [rad/s]
 * @param ax, ay, az  Accelerometer readings [any unit; normalized internally]
 * @param mx, my, mz  Magnetometer readings [any unit; normalized internally]
 * @param dt          Time since last update [s]
 */
void madgwick_marg_update(MadgwickState *state,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float dt);

/**
 * 6-DOF IMU fallback (gyro + accel only).
 * Use when a fresh mag sample isn't available — roll and pitch stay
 * corrected, yaw drifts at the residual gyro bias rate.
 */
void madgwick_imu_update(MadgwickState *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);

/**
 * Set the quaternion straight from one accelerometer reading, with yaw
 * zero, so the filter starts at the measured tilt instead of converging
 * to it from identity at beta's rate. Same accel convention as the
 * update functions. Leaves the state unchanged if the reading has no
 * magnitude.
 *
 * @param ax, ay, az  Accelerometer reading at rest [any unit]
 */
void madgwick_set_from_accel(MadgwickState *state, float ax, float ay, float az);

/**
 * Heading correction from a magnetometer reading — kept apart from the
 * gradient step so the magnetometer can only turn the attitude about the
 * vertical. Tilt stays referenced to the accelerometer alone, so a
 * disturbed field can't pull it. Turns toward the heading where the
 * field's horizontal part points along filter +x ("north"), at
 * gain × error, capped at max_rate. Call after the IMU update. Skipped
 * when the field is too close to vertical to give a heading.
 *
 * @param mx, my, mz  Magnetometer reading in the filter's sensor frame [any unit]
 * @param gain        [1/s]; 0 disables
 * @param max_rate    [rad/s]
 */
void madgwick_heading_update(MadgwickState *state, float mx, float my, float mz,
                             float gain, float max_rate, float dt);

/**
 * Turn the attitude about the vertical in one step so the reading's
 * horizontal part points along filter +x. For seeding at boot, after
 * madgwick_set_from_accel().
 *
 * @return false if the field is too close to vertical to give a heading.
 */
bool madgwick_align_heading(MadgwickState *state, float mx, float my, float mz);

/**
 * Convert the current quaternion to roll/pitch/yaw in degrees.
 * Computed on demand — call only when needed (telemetry, logging, debug).
 * Pitch saturates near ±90° due to Euler gimbal lock; use the quaternion
 * directly for TVC math to avoid this singularity.
 */
void madgwick_get_euler(const MadgwickState *state, EulerAngles *euler);