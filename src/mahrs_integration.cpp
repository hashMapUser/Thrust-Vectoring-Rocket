// --------------------------------------------------------
// MADGWICK FILTER — FLIGHT LOOP INTEGRATION
// --------------------------------------------------------
// Feeds the LSM6DSOX (gyro + accel) into madgwick_imu_update() and the
// magnetometer into madgwick_heading_update(), with the axis remap and
// sign conventions for the rocket body frame. All inputs are body frame —
// lsm6dsox_to_body() / mag_to_body() turn chip axes into it first.
//
// The magnetometer only sets heading (rotation about the vertical — roll
// about the long axis while the rocket is upright). Tilt comes from the
// accelerometer alone, so a field disturbed by servos, currents or the
// motor case can't pull the angles the TVC steers by.
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

// Beta gain schedule — accel/mag trust varies with flight phase.
// During boost, thrust dominates the accelerometer reading and
// corrupts the "down" reference, so we drop beta near zero and
// rely on gyro integration.
// Low on the pad: the attitude is seeded from the accelerometer at boot
// (mahrs_seed_from_accel), so fast convergence isn't needed, and the
// normalized gradient step chatters by about 2·beta·dt every tick — 0.05
// gave ~0.22° p-p of jitter at rest in simulation, 0.01 ~0.09°. Still
// corrects tilt at up to ~1.1°/s, well above any leftover gyro bias.
static const float BETA_IDLE   = 0.01f;
static const float BETA_BOOST  = 0.0f;    // gyro-only during powered flight
static const float BETA_COAST  = 0.033f;  // nominal — coast and descent
static const float BETA_LAND   = 0.05f;   // high — re-anchor after landing

static const float ZETA_DEFAULT = 0.001f; // turn on once filter is verified

// Magnetometer heading gain schedule [1/s] — heading error decays with
// time constant 1/gain. Off during boost for the same reason beta is
// (and motor current is at its highest). Corrections are capped at
// MAG_MAX_RATE so a bad reading can't whip the heading round.
static const float MAG_GAIN_IDLE  = 0.2f;
static const float MAG_GAIN_BOOST = 0.0f;
static const float MAG_GAIN_COAST = 0.1f;
static const float MAG_GAIN_LAND  = 0.2f;
static const float MAG_MAX_RATE   = 10.0f * DEG_TO_RAD;

static float mag_gain = MAG_GAIN_IDLE;

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

// Called once in setup() after mahrs_init(), with a body-frame accel
// reading [g] taken at rest. Starts the filter at the measured tilt —
// without it the filter starts at vertical (identity) and slides to the
// true attitude at BETA_IDLE's rate, ~20 s for a board lying on its side.
// Same remap + negation as mahrs_tick().
void mahrs_seed_from_accel(float ax, float ay, float az) {
    madgwick_set_from_accel(&mahrs, -az, ay, -ax);
}

// Called once in setup() after mahrs_seed_from_accel(), with a body-frame
// mag reading taken at rest: turns the seeded attitude to face magnetic
// north, so the heading doesn't have to converge at MAG_GAIN_IDLE.
// Same remap as mahrs_tick(). Returns false if the field is too close to
// vertical to give a heading.
bool mahrs_align_heading(float mx, float my, float mz) {
    return madgwick_align_heading(&mahrs, mz, -my, mx);
}

// Called by the flight state machine when transitioning between phases
void mahrs_set_phase(FlightState phase) {
    switch (phase) {
        case STATE_IDLE:
        case STATE_ARMED:
            mahrs.beta = BETA_IDLE;
            mag_gain   = MAG_GAIN_IDLE;
            break;
        case STATE_POWERED:
            mahrs.beta = BETA_BOOST;
            mag_gain   = MAG_GAIN_BOOST;
            break;
        case STATE_COAST:
        case STATE_APOGEE:
        case STATE_DESCENT:
            mahrs.beta = BETA_COAST;
            mag_gain   = MAG_GAIN_COAST;
            break;
        case STATE_LANDED:
            mahrs.beta = BETA_LAND;
            mag_gain   = MAG_GAIN_LAND;
            break;
        default:
            break;
    }
}

// --------------------------------------------------------
// MAIN TICK
// --------------------------------------------------------

// Call once per 125 Hz scheduler tick, after fresh IMU + mag reads. Both
// in the body frame; mag->valid false (no fresh sample) skips the heading
// correction for this tick, leaving heading on the gyro.
void mahrs_tick(const LSM6DSOX_Data *imu, const mag_data *mag) {
    if (!imu->valid) return;

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

    madgwick_imu_update(&mahrs, gx_n, gy_n, gz_n, ax_n, ay_n, az_n, DT);

    // --- Mag remap: the same axis remap as the gyro, no sign flip ---
    // The magnetometer reports the field direction itself — no specific-
    // force negation like the accelerometer.
    if (mag->valid) {
        float mx_n =  mag->mag_z;
        float my_n = -mag->mag_y;
        float mz_n =  mag->mag_x;
        madgwick_heading_update(&mahrs, mx_n, my_n, mz_n, mag_gain, MAG_MAX_RATE, DT);
    }
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

// Upward specific force [g] in the earth frame: the body-frame accel turned
// by the current attitude — 1 g at rest whichever way the vehicle is lying.
// Call after mahrs_tick() so the attitude is this tick's.
float mahrs_vertical_accel_g(const LSM6DSOX_Data *imu) {
    // Same remap as the gyro; no negation — this stays specific force.
    float fx =  imu->az;
    float fy = -imu->ay;
    float fz =  imu->ax;

    // "Down" row of R(q) (filter sensor frame -> NED), negated for up.
    float q0 = mahrs.q0, q1 = mahrs.q1, q2 = mahrs.q2, q3 = mahrs.q3;
    float down = 2.0f*(q1*q3 - q0*q2) * fx
               + 2.0f*(q2*q3 + q0*q1) * fy
               + (1.0f - 2.0f*(q1*q1 + q2*q2)) * fz;
    return -down;
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