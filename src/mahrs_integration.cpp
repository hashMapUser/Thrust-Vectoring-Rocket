// --------------------------------------------------------
// MADGWICK FILTER — FLIGHT LOOP INTEGRATION
// --------------------------------------------------------
// This file demonstrates how to feed sensor data from the LSM6DSOX
// driver and magnetometer driver into madgwick_marg_update() with the
// correct axis remap and sign conventions for your rocket body frame.
//
// Body frame convention:
//   +X = nose to nozzle (points down when rocket is vertical on pad)
//   +Y = left
//   +Z = horizontal, parallel to pad
//
// On the pad, gravity acts in +X direction; accelerometer reads (-1, 0, 0) g.
//
// Filter frame convention (NED):
//   +X = north-ish (horizontal)
//   +Y = east-ish (horizontal)
//   +Z = down
//
// The remap below converts body-frame sensor readings into the filter's
// NED frame AND negates accel to convert specific force into the gravity
// vector convention Madgwick expects.

#include "madgwick.h"
#include "lsm6dsox.h"
#include "mag.h"
#include "flight_sm.h"

// --------------------------------------------------------
// CONSTANTS
// --------------------------------------------------------

static const float DT = 1.0f / 125.0f;  // 125 Hz fixed-rate tick
// DEG_TO_RAD is provided by Arduino.h (via SPI.h) — no local definition needed

// Beta gain schedule — accel/mag trust varies with flight phase.
// During boost, thrust dominates the accelerometer reading and
// corrupts the "down" reference, so we drop beta near zero and
// rely on gyro integration.
static const float BETA_IDLE   = 0.05f;   // high — fast convergence on pad
static const float BETA_BOOST  = 0.0f;    // gyro-only during powered flight
static const float BETA_COAST  = 0.033f;  // nominal — coast and descent
static const float BETA_LAND   = 0.05f;   // high — re-anchor after landing

static const float ZETA_DEFAULT = 0.001f; // turn on once filter is verified

// --------------------------------------------------------
// FILTER STATE
// --------------------------------------------------------

static MadgwickState mahrs;

// --------------------------------------------------------
// FLIGHT STATE HOOKS
// --------------------------------------------------------

// Called once in setup() after sensor drivers are initialized
void mahrs_init() {
    madgwick_init(&mahrs, BETA_IDLE, ZETA_DEFAULT);
}

// Called by the flight state machine when transitioning between phases
void mahrs_set_phase(FlightState phase) {
    switch (phase) {
        case STATE_IDLE:
        case STATE_ARMED:
            mahrs.beta = BETA_IDLE;
            break;
        case STATE_POWERED:
            mahrs.beta = BETA_BOOST;
            break;
        case STATE_COAST:
        case STATE_APOGEE:
        case STATE_DESCENT:
            mahrs.beta = BETA_COAST;
            break;
        case STATE_LANDED:
            mahrs.beta = BETA_LAND;
            break;
        default:
            break;
    }
}

// --------------------------------------------------------
// MAIN TICK
// --------------------------------------------------------

// Call once per 125 Hz scheduler tick, after fresh IMU + mag reads.
void mahrs_tick(const LSM6DSOX_Data *imu, const mag_data *mag) {
    if (!imu->valid || !mag->valid) {
        // Sensor failure — fall back to gyro-only integration
        if (imu->valid) {
            float gx_n =  imu->gz * DEG_TO_RAD;
            float gy_n = -imu->gy * DEG_TO_RAD;
            float gz_n =  imu->gx * DEG_TO_RAD;
            float ax_n = -imu->az;
            float ay_n =  imu->ay;
            float az_n = -imu->ax;
            madgwick_imu_update(&mahrs, gx_n, gy_n, gz_n, ax_n, ay_n, az_n, DT);
        }
        return;
    }

    // --- Gyro remap: body (X-down, Y-left, Z-horiz) -> filter (NED) ---
    // No sign flip needed for angular rates beyond the axis remap.
    float gx_n =  imu->gz * DEG_TO_RAD;
    float gy_n = -imu->gy * DEG_TO_RAD;
    float gz_n =  imu->gx * DEG_TO_RAD;

    // --- Accel remap + specific-force-to-gravity-vector negation ---
    // The LSM6DSOX reports specific force (opposite gravity at rest).
    // Madgwick's objective function expects the gravity vector directly,
    // so we negate after the axis remap.
    //
    // Pad check: body accel = (-1, 0, 0) g  ->  filter accel = (0, 0, +1) g
    float ax_n = -imu->az;
    float ay_n =  imu->ay;
    float az_n = -imu->ax;

    // --- Mag remap: axis only, no sign flip ---
    // Magnetometer reports the actual field direction, no "specific force"
    // weirdness.
    float mx_n =  mag->mag_x;
    float my_n = -mag->mag_y;
    float mz_n =  mag->mag_z;

    madgwick_marg_update(&mahrs,
                         gx_n, gy_n, gz_n,
                         ax_n, ay_n, az_n,
                         mx_n, my_n, mz_n,
                         DT);
}

// --------------------------------------------------------
// ATTITUDE READOUT
// --------------------------------------------------------

// Wrapper that renames filter Euler angles to rocket-meaningful labels.
// VERIFY THESE MAPPINGS ON THE BENCH BEFORE TRUSTING IN FLIGHT.
typedef struct {
    float tip_a;     // tip-over angle about body Y (rotation about filter-Y axis)
    float tip_b;     // tip-over angle about body Z (rotation about filter-X axis)
    float spin;      // rotation about body X (longitudinal / filter-Z)
} RocketAttitude;

void mahrs_get_attitude(RocketAttitude *out) {
    EulerAngles e;
    madgwick_get_euler(&mahrs, &e);

    // Filter-frame Euler maps onto rocket-frame as follows:
    //   filter roll  (rotation about filter X = body Z)   -> tip about body Z
    //   filter pitch (rotation about filter Y = -body Y)  -> tip about body Y (sign-flipped)
    //   filter yaw   (rotation about filter Z = body X)   -> spin about longitudinal axis
    out->tip_b = e.roll;
    out->tip_a = -e.pitch;   // negate because body Y = -filter Y
    out->spin  = e.yaw;
}

// Convenience: get the raw filter quaternion for TVC math.
// Note this is in the FILTER frame, not the body frame.
// For TVC you'll want to apply the inverse remap when interpreting it.
void mahrs_get_quaternion(float *q0, float *q1, float *q2, float *q3) {
    *q0 = mahrs.q0;
    *q1 = mahrs.q1;
    *q2 = mahrs.q2;
    *q3 = mahrs.q3;
}