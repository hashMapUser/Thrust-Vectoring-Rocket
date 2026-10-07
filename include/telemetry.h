#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// BENCH TELEMETRY — $TLM frames over USB serial
// --------------------------------------------------------
// One ASCII line per frame, in the same style as the $HLTH frames:
//
//   $TLM,t_ms,state,flags,q0,q1,q2,q3,tip_a,tip_b,spin,gx,gy,gz,ax,ay,az,
//        mx,my,mz,alt_m,vel_ms,baro_alt_m,press_hpa,temp_c,pid_p,pid_y,
//        servo_p_us,servo_y_us,pack_v,loop_us*XX
//
// XX is the XOR of every character between '$' and '*', as two hex
// digits. Field order must mirror TLM_FIELDS in src/telemetry_panel.py.
//
// Off by default so the PlatformIO serial monitor stays readable:
// send 'T' to start the stream, 't' to stop it. bench_gui.py sends
// these for you when its Telemetry tab is open.

// Send every Nth control tick: 125 Hz / 3 ≈ 42 Hz.
#define TLM_DIVIDER  3

// A baro sample older than this clears TLM_F_BARO_OK.
#define TLM_BARO_STALE_MS  250

// flags bits
#define TLM_F_IMU_VALID   0x01
#define TLM_F_BARO_OK     0x02   // baro produced a sample within TLM_BARO_STALE_MS
#define TLM_F_TVC_LIVE    0x04   // fsm.tvc_enabled — PID is driving the servos
#define TLM_F_ARM_SWITCH  0x08   // SW401 closed (debounced)
#define TLM_F_PAD_REST    0x10   // pad-rest latched, launch detection armed
#define TLM_F_LAUNCH_DET  0x20   // launch accel latched, waiting on altitude confirm
#define TLM_F_GROUND_CAP  0x40   // capturing the launch ground reference
#define TLM_F_MAIN_FIRED  0x80

typedef struct {
    uint32_t t_ms;
    uint8_t  state;              // FlightState
    uint8_t  flags;              // TLM_F_* bits

    float q0, q1, q2, q3;        // filter frame (NED) — see mahrs_get_quaternion()
    float tip_a, tip_b, spin;    // [deg] the angles the PID acts on

    float gx, gy, gz;            // [deg/s] body frame, bias-corrected
    float ax, ay, az;            // [g] body frame
    float mx, my, mz;            // [Gauss] magnetometer's own axes, calibrated;
                                 // NaN when there's no fresh sample

    float alt_m;                 // altitude estimator output [m]
    float vel_ms;                // vertical velocity estimate [m/s]
    float baro_alt_m;            // raw barometric altitude [m]
    float press_hpa;             // last valid baro sample
    float temp_c;

    float pid_p, pid_y;          // [deg] nozzle deflection commanded
    float servo_p_us, servo_y_us;

    float    pack_v;             // via ARM_SENSE — reads 0 when SW401 is open
    uint32_t loop_us;            // time spent in this tick before telemetry
} TelemetryFrame;

void telemetry_set_enabled(bool on);
bool telemetry_enabled();

/**
 * Format and send one $TLM line, if the stream is on and a host is
 * connected. Never blocks: if the USB TX buffer can't take the whole
 * line, the frame is dropped rather than stalling the control loop.
 *
 * @return true if the frame was written.
 */
bool telemetry_emit(const TelemetryFrame *f);
