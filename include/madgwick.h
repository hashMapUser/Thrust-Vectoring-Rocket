#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// MADGWICK FILTER TUNING
// --------------------------------------------------------

// Beta is the gradient descent step size.
// Higher = faster convergence to accel/mag reference but noisier output.
// Lower  = smoother but slower to correct gyro drift.
//
// Suggested values:
//   0.033  — Madgwick's original recommendation for 100 Hz IMU
//   0.10   — good starting point for 833 Hz with moderate dynamics
//   0.50   — aggressive convergence, use only if gyro noise is low
//
// For a TVC rocket: tune this on the bench first. During fast maneuvers
// the filter should trust the gyro more (lower beta). Start at 0.10
// and adjust based on how much lag vs noise you see in the Euler output.
#define MADGWICK_BETA       0.10f

// --------------------------------------------------------
// STRUCTS
// --------------------------------------------------------

/**
 * Full attitude state — quaternion + Euler angles.
 * Quaternion is the ground truth; Euler angles are derived for readability.
 *
 * Euler convention: ZYX (yaw → pitch → roll), aerospace standard.
 *   roll  — rotation about X axis [degrees]
 *   pitch — rotation about Y axis [degrees]  ← primary TVC axis
 *   yaw   — rotation about Z axis [degrees]  ← primary TVC axis
 */
typedef struct {
    // Quaternion (internal math representation)
    float q0, q1, q2, q3;   // w, x, y, z

    // Euler angles derived from quaternion [degrees]
    float roll;
    float pitch;
    float yaw;
} MadgwickState;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialize the filter to a known upright orientation.
 * Call once in setup() after sensors are initialized.
 *
 * @param state  Filter state to initialize.
 */
void madgwick_init(MadgwickState *state);

/**
 * Run one filter update step using all three sensors.
 *
 * Inputs must be raw physical units as produced by the sensor drivers:
 *   gx/gy/gz — angular rate   [deg/s]   from LSM6DSOX
 *   ax/ay/az — linear accel   [g]        from LSM6DSOX
 *   mx/my/mz — magnetic field [Gauss]    from MMC5603NJ
 *
 * @param state  Filter state updated in place.
 * @param gx/gy/gz  Gyro rates [deg/s]
 * @param ax/ay/az  Accel [g]
 * @param mx/my/mz  Mag [Gauss]
 * @param dt        Time since last call [seconds]
 */
void madgwick_update(MadgwickState *state,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float dt);

/**
 * Fallback update using only gyro + accel (no magnetometer).
 * Yaw will drift. Use this if the magnetometer reading is invalid
 * or during a mag calibration phase.
 *
 * @param state  Filter state updated in place.
 * @param gx/gy/gz  Gyro rates [deg/s]
 * @param ax/ay/az  Accel [g]
 * @param dt        Time since last call [seconds]
 */
void madgwick_update_imu(MadgwickState *state,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);