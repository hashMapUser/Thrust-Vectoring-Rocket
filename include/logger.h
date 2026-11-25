#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "flight_sm.h"

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

// Every record goes into a RAM ring buffer. From the SW401 arming edge —
// or from liftoff, if it flew with the switch open — (with the
// LOG_FLASH_PRELAUNCH_RECORDS before it) the records are also streamed to
// the GD25Q128 flash, which survives a reset or a power loss. So a launch
// the firmware never detected is still in flash.
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
// at 0x010000, Test 5 staging from 0x100000) — everything from 2 MB to the
// end of the 16 MB chip. 14 MB holds ~15 minutes at 125 Hz. Page 0 is a
// header; records are packed two per 256-byte page.
#define LOG_FLASH_BASE      0x200000u
#define LOG_FLASH_SIZE      0xE00000u

// Records from before the flash log starts copied into it too: 4 s at
// 125 Hz. For a flight with the switch open the log starts at liftoff
// confirmation — up to ~2 s after ignition by the climb backstop — so this
// still reaches back past ignition.
#define LOG_FLASH_PRELAUNCH_RECORDS  500

// A long wait on the pad can fill the region before launch. Only then —
// still armed and no launch trigger seen (logger_allow_wrap()) — does the
// log start over, carrying the last 20 s from the RAM ring so the moments
// before a launch right after are kept. Kept well under the ring size so
// the flash can catch up before the ring overwrites them.
#define LOG_FLASH_RESTART_RECORDS    2500

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
 * flash back up and keep appending to the log already in it (a new one if
 * the last is already on the card). Skips the SD card (its init can block
 * for seconds); logger_finalize() then leaves the flight in flash, and the
 * next normal boot copies it to the card.
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
 * Whether the flash log may start over if it fills up. Call every tick
 * before logger_write(): true only while armed on the pad with no launch
 * trigger seen. Data from before this recording began (an earlier log not
 * yet on the card) is never recycled either way.
 */
void logger_allow_wrap(bool allowed);

/**
 * Append a state-transition checkpoint line directly to the SD .LOG file.
 * Call on every FSM state change. State changes are infrequent, so the
 * SD write here is fine, and it survives a power loss before finalize().
 * Also drives the flash log: ARMED starts recording, IDLE (disarmed)
 * pauses it, POWERED starts it if it isn't running (flew unarmed). A
 * flash log not yet on the SD card is appended to, never erased.
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
 * it without pulling the card (tools/flash_dump.py saves it to a file).
 * If the flash log is recording it's paused for the dump and picked up
 * again after — the loop is blocked meanwhile, so not on the pad.
 * Not gated by logger_finalize()'s idempotence; call it repeatedly.
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
