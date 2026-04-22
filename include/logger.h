#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flight_sm.h"

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

#define SD_CS_PIN           BUILTIN_SDCARD
#define SD_MAX_FILES        999

// RAM ring buffer — stores high-rate flight data during flight.
// 4000 records × ~120 bytes each = ~480 KB
// Teensy 4.0 has 1 MB RAM so this is safe.
// At 100 Hz this covers 40 seconds — more than any motor burn + coast phase.
#define LOG_RAM_CAPACITY    4000

// Checkpoint file — small file written to SD during flight.
// Contains only state transitions and timestamps.
// Survives a crash when the full RAM dump cannot be written.
#define LOG_CHECKPOINT_FILE "CHKPT.TXT"

// --------------------------------------------------------
// LOG RECORD — fixed-width struct stored in RAM
// --------------------------------------------------------

typedef struct {
    uint32_t timestamp_ms;

    // Attitude
    float roll, pitch, yaw;
    float q0, q1, q2, q3;

    // IMU
    float gx, gy, gz;
    float ax, ay, az;

    // Magnetometer
    float mx, my, mz;

    // Barometer / altitude
    float temperature_c;
    float pressure_hpa;
    float altitude_m;
    float velocity_ms;

    // Control outputs
    float servo_pitch_us;
    float servo_yaw_us;
    float pid_pitch_out;
    float pid_yaw_out;

    // Status
    uint8_t flight_state;
    bool    imu_valid;
    bool    baro_valid;
    bool    mag_valid;
} LogRecord;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Initialise the logger.
 * Opens the checkpoint file on SD for immediate writing.
 * Allocates the RAM ring buffer.
 * @return true on success; false if SD not found.
 */
bool logger_init();

/**
 * Store one record in the RAM ring buffer.
 * Never touches SD — safe to call every loop iteration.
 * Overwrites oldest record when buffer is full (ring behavior).
 */
void logger_write(const LogRecord *rec);

/**
 * Write a state transition checkpoint to SD immediately.
 * Cheap — only writes one line of text.
 * Call on every FSM state change.
 *
 * @param state      New flight state.
 * @param altitude_m Current altitude [m].
 */
void logger_checkpoint(FlightState state, float altitude_m);

/**
 * Dump the entire RAM buffer to a new CSV file on SD.
 * Blocking — takes several seconds. Call after landing only.
 * @return true if dump succeeded.
 */
bool logger_dump_to_sd();

/**
 * How many records are currently stored in the RAM buffer.
 */
uint16_t logger_record_count();

/**
 * True if SD card is available and checkpoint file is open.
 */
bool logger_sd_ready();