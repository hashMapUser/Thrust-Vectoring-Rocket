#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flight_sm.h"

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

// Every record goes into a RAM ring buffer. From liftoff confirmation
// (with the LOG_FLASH_PRELAUNCH_RECORDS before it) the records are also
// streamed to the GD25Q128 flash, which survives a reset or a power loss.
// On landing, logger_finalize() copies the flight to the SD card
// (FLIGHT_XXX.CSV) — from flash when it holds the flight, otherwise from
// the RAM buffer. A flight left in flash but not yet on the SD card (power
// lost before landing, no card) is copied at the next boot.
// With no SD card, the data streams as CSV over USB serial instead, and
// logger_usb_dump() does the same thing on demand.

// RAM ring buffer — every record, and the queue feeding the flash.
// 4000 records × sizeof(LogRecord) bytes. DMAMEM places this in OCRAM2.
// At 125 Hz this covers ~32 s; once full, the oldest records are
// overwritten. It's the whole flight log only when the flash is down.
#define LOG_RAM_CAPACITY    4000

// Flash flight-log region, clear of the bench test's scratch areas (Test 4
// at 0x010000, Test 5 staging from 0x100000). 4 MB holds ~4 minutes at
// 125 Hz. Page 0 is a header; records are packed two per 256-byte page.
#define LOG_FLASH_BASE      0x200000u
#define LOG_FLASH_SIZE      0x400000u

// Records from before liftoff confirmation copied into the flash log too,
// so it includes the pad and the launch trigger (at most 1 s before
// confirmation): 2 s at 125 Hz. Small enough to drain into flash within
// ~2 s, so a reset early in the boost loses none of it.
#define LOG_FLASH_PRELAUNCH_RECORDS  250

// Blocks erased in the background once the switch arms the pad, so liftoff
// doesn't start with erases (only when the previous flight is safe on SD).
#define LOG_FLASH_PREERASE_BYTES     0x100000u

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
 * Initialise the logger: bring up the SD card and the flash, copy a flight
 * still waiting in flash to the card, and pick the next free
 * FLIGHT_XXX.CSV / FLIGHT_XXX.LOG filename pair.
 * @return true if the SD card is ready; false if SD.begin() failed
 *         (logging still works — to flash when it's up — but nothing
 *         reaches the card this session).
 */
bool logger_init();

/**
 * After a mid-flight watchdog reset, instead of logger_init(): bring the
 * flash back up and keep appending to the flight already in it. Skips the
 * SD card (its init can block for seconds); logger_finalize() then leaves
 * the flight in flash, and the next normal boot copies it to the card.
 */
void logger_resume();

/**
 * Register a function called once per CSV row during logger_finalize()
 * and logger_usb_dump(). Writing the whole buffer takes far longer than
 * the watchdog timeout, so the flight loop passes its wdt.feed() here.
 */
void logger_set_keepalive(void (*keepalive)());

/**
 * Store one record in the RAM ring buffer and move the flash log along —
 * at most one non-blocking flash operation per call. Never touches the SD
 * card, never blocks. Call every loop iteration.
 */
void logger_write(const LogRecord *rec);

/**
 * Append a state-transition checkpoint line directly to the SD .LOG file.
 * Call on every FSM state change. State changes are infrequent, so the
 * SD write here is fine, and it survives a power loss before finalize().
 * Also drives the flash log: ARMED starts the background pre-erase,
 * POWERED starts recording the flight.
 */
void logger_checkpoint(FlightState state, float altitude_m);

/**
 * Close out the flight: finish writing it to flash, then copy it to the
 * SD .CSV file, oldest record first — from flash when it holds the flight,
 * otherwise from the RAM buffer. Call on landing. Idempotent — a second
 * call is a no-op. Falls back to logger_usb_dump() if the SD card isn't
 * ready or the file can't be opened, so a failed card never means lost data.
 */
void logger_finalize();

/**
 * Stream the flight as CSV directly over USB serial, between
 * "-----BEGIN FLIGHT CSV-----" / "-----END FLIGHT CSV-----" markers — the
 * flight in flash if there is one, otherwise the RAM ring buffer.
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
