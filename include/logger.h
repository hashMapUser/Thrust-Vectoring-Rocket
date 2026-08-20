#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flight_sm.h"

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

// GD25Q128 NOR flash is wired incorrectly on this board and cannot be used
// for flight logging. Flight data is held in a RAM ring buffer in flight,
// then dumped to the SD card (FLIGHT_XXX.CSV) when logger_finalize() runs.

// RAM ring buffer — absorbs high-rate writes during flight.
// 4000 records × sizeof(LogRecord) bytes. DMAMEM places this in OCRAM2.
// At 125 Hz this covers ~32 s; once full, the oldest records are
// overwritten (ring buffer), so only the most recent ~32 s survive to
// logger_finalize() if the flight runs longer than that.
#define LOG_RAM_CAPACITY    4000

// --------------------------------------------------------
// LOG RECORD — packed fixed-width struct
// --------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;

    // Attitude
    float roll, pitch, yaw;
    float q0, q1, q2, q3;

    // IMU
    float gx, gy, gz;
    float ax, ay, az;

    // Magnetometer (zeros when not fitted)
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
 * Initialise the logger: bring up the SD card and pick the next free
 * FLIGHT_XXX.CSV / FLIGHT_XXX.LOG filename pair.
 * @return true if the SD card is ready; false if SD.begin() failed
 *         (logging still works, but nothing will be saved).
 */
bool logger_init();

/**
 * Store one record in the RAM ring buffer. Never touches the SD card,
 * never blocks. Call every loop iteration.
 */
void logger_write(const LogRecord *rec);

/**
 * Append a state-transition checkpoint line directly to the SD .LOG file.
 * Call on every FSM state change. State changes are infrequent, so the
 * SD write here is fine, and it survives a power loss before finalize().
 */
void logger_checkpoint(FlightState state, float altitude_m);

/**
 * Dump the RAM ring buffer to the SD .CSV file, oldest record first, and
 * close it out. Call on landing. Idempotent — a second call is a no-op.
 */
void logger_finalize();

/**
 * How many records are currently in the RAM ring buffer.
 */
uint16_t logger_record_count();
