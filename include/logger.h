#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flight_sm.h"

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

// SD card disabled for this flight (H6: SD VDD wired to 5V).
// Flight data logs to GD25Q128 NOR flash; dump via USB after landing.

// RAM ring buffer — absorbs high-rate writes during flight.
// 4000 records × sizeof(LogRecord) bytes. DMAMEM places this in OCRAM2.
#define LOG_RAM_CAPACITY    4000

// --------------------------------------------------------
// LOG RECORD — packed fixed-width struct for flash storage
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
 * Initialise the logger.
 * Brings up NOR flash (SPI2), loads or initialises the flash header.
 * @return true if flash is ready; false if GD25Q128 not found.
 */
bool logger_init();

/**
 * Store one record in the RAM ring buffer and coalesce into flash pages.
 * Never blocks. Call every loop iteration.
 */
void logger_write(const LogRecord *rec);

/**
 * Write a state-transition checkpoint to the flash checkpoint sector.
 * Call on every FSM state change.
 */
void logger_checkpoint(FlightState state, float altitude_m);

/**
 * Finalise the flight: write the TVCR header (record_count, flight_epoch_ms)
 * and flush any partial page. Call on landing or USB dump request.
 */
void logger_finalize();

/**
 * Stream all flash records over USB Serial in TVCR binary protocol.
 * Host sends 'R'; Teensy responds with header + raw records + CRC32.
 * Blocks until all data is sent. Call only from LANDED state.
 */
void logger_usb_dump();

/**
 * How many records are currently in the RAM ring buffer.
 */
uint16_t logger_record_count();
