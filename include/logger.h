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
// If the SD card isn't present or its init failed, the RAM buffer is the
// only copy of the data — logger_finalize() falls back to streaming it as
// CSV over USB serial instead of silently discarding it, and
// logger_usb_dump() does the same thing on demand.

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
 * Falls back to logger_usb_dump() if the SD card isn't ready or the file
 * can't be opened, so a failed card never means lost data.
 */
void logger_finalize();

/**
 * Stream the RAM ring buffer as CSV directly over USB serial, between
 * "-----BEGIN FLIGHT CSV-----" / "-----END FLIGHT CSV-----" markers.
 * Use this to retrieve flight data when there's no SD card, or to inspect
 * it without pulling the card. Safe to call any time; not gated by
 * logger_finalize()'s idempotence, so it can be called repeatedly.
 */
void logger_usb_dump();

/**
 * How many records are currently in the RAM ring buffer.
 */
uint16_t logger_record_count();

/**
 * Write the CSV header / one record as a CSV row to any Print (an SD
 * FsFile or Serial). Same format as FLIGHT_XXX.CSV — used by the bench
 * test so its SD file can be read with the same tools as flight data.
 */
class Print;
void logger_print_csv_header(Print &out);
void logger_print_csv_row(Print &out, const LogRecord *rec);
