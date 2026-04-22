#include <Arduino.h>
#include <SD.h>
#include <math.h>
#include "logger.h"

// --------------------------------------------------------
// RAM RING BUFFER
// --------------------------------------------------------
// Stores records during flight. Uses a ring so if the buffer
// fills up (very long flight) we keep the most recent data
// rather than stopping or blocking.

static LogRecord  _buf[LOG_RAM_CAPACITY];
static uint16_t   _head       = 0;   // next write position
static uint16_t   _count      = 0;   // records stored (0 to LOG_RAM_CAPACITY)
static bool       _wrapped    = false; // true once buffer has filled once

// --------------------------------------------------------
// SD STATE
// --------------------------------------------------------
static bool   _sd_ready       = false;
static File   _checkpoint;
static char   _dump_filename[20];

// CSV header — must match LogRecord field order exactly
static const char CSV_HEADER[] =
    "timestamp_ms,"
    "roll,pitch,yaw,"
    "q0,q1,q2,q3,"
    "gx,gy,gz,"
    "ax,ay,az,"
    "mx,my,mz,"
    "temp_c,pressure_hpa,"
    "altitude_m,velocity_ms,"
    "servo_pitch_us,servo_yaw_us,"
    "pid_pitch,pid_yaw,"
    "state,imu_ok,baro_ok,mag_ok\n";

// --------------------------------------------------------
// PRIVATE HELPERS
// --------------------------------------------------------

// Find the next available FLIGHT_NNN.CSV filename
static bool find_filename(char *out, size_t len) {
    for (int i = 1; i <= SD_MAX_FILES; i++) {
        snprintf(out, len, "FLIGHT_%03d.CSV", i);
        if (!SD.exists(out)) return true;
    }
    return false;
}

// Write one LogRecord as a CSV line to an open File
static void write_record_csv(File &f, const LogRecord *r) {
    char tmp[32];

    // Helper lambdas via macros to keep this readable
    #define WF(val, dec) dtostrf(val, 1, dec, tmp); f.print(tmp); f.print(',');
    #define WU(val)      f.print((uint32_t)(val));  f.print(',');
    #define WI(val)      f.print((int)(val));        f.print(',');

    WU(r->timestamp_ms);
    WF(r->roll,  2);  WF(r->pitch, 2);  WF(r->yaw,  2);
    WF(r->q0,    4);  WF(r->q1,    4);  WF(r->q2,   4);  WF(r->q3, 4);
    WF(r->gx,    2);  WF(r->gy,    2);  WF(r->gz,   2);
    WF(r->ax,    3);  WF(r->ay,    3);  WF(r->az,   3);
    WF(r->mx,    4);  WF(r->my,    4);  WF(r->mz,   4);
    WF(r->temperature_c,  2);
    WF(r->pressure_hpa,   2);
    WF(r->altitude_m,     2);
    WF(r->velocity_ms,    3);
    WF(r->servo_pitch_us, 1);
    WF(r->servo_yaw_us,   1);
    WF(r->pid_pitch_out,  3);
    WF(r->pid_yaw_out,    3);

    // Last fields — no trailing comma, end with newline
    f.print(r->flight_state);            f.print(',');
    f.print(r->imu_valid  ? 1 : 0);      f.print(',');
    f.print(r->baro_valid ? 1 : 0);      f.print(',');
    f.println(r->mag_valid ? 1 : 0);

    #undef WF
    #undef WU
    #undef WI
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

bool logger_init() {
    _head    = 0;
    _count   = 0;
    _wrapped = false;

    // Try to bring up SD
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[LOGGER] SD not found — RAM logging only, no checkpoint");
        _sd_ready = false;
        return false;
    }

    // Open / create checkpoint file (append mode — survives reboots)
    _checkpoint = SD.open(LOG_CHECKPOINT_FILE, FILE_WRITE);
    if (!_checkpoint) {
        Serial.println("[LOGGER] Could not open checkpoint file");
        _sd_ready = false;
        return false;
    }

    // Write a boot marker to the checkpoint
    _checkpoint.print("\n--- BOOT ");
    _checkpoint.print(millis());
    _checkpoint.println(" ms ---");
    _checkpoint.flush();

    // Pre-find the dump filename now so we don't have to search at landing
    find_filename(_dump_filename, sizeof(_dump_filename));

    _sd_ready = true;
    Serial.print("[LOGGER] SD ready. RAM buffer: ");
    Serial.print(LOG_RAM_CAPACITY);
    Serial.print(" records. Dump file: ");
    Serial.println(_dump_filename);

    return true;
}

void logger_write(const LogRecord *rec) {
    // Write to RAM ring buffer — no SD access
    _buf[_head] = *rec;
    _head = (_head + 1) % LOG_RAM_CAPACITY;

    if (_count < LOG_RAM_CAPACITY) {
        _count++;
    } else {
        _wrapped = true;  // buffer is full — oldest record is being overwritten
    }
}

void logger_checkpoint(FlightState state, float altitude_m) {
    if (!_sd_ready || !_checkpoint) return;

    char line[80];
    snprintf(line, sizeof(line),
             "T=%lu ms  STATE=%-10s  ALT=%.1f m  RECORDS=%u\n",
             millis(),
             STATE_NAMES[(int)state],
             altitude_m,
             _count);

    _checkpoint.print(line);
    _checkpoint.flush();   // flush immediately — this is our crash insurance

    Serial.print("[LOGGER] Checkpoint: "); Serial.print(line);
}

bool logger_dump_to_sd() {
    if (_count == 0) {
        Serial.println("[LOGGER] Nothing to dump — buffer empty");
        return false;
    }

    Serial.print("[LOGGER] Dumping "); Serial.print(_count);
    Serial.print(" records to "); Serial.println(_dump_filename);

    // Close checkpoint before opening dump file to free file handles
    if (_checkpoint) {
        _checkpoint.print("--- DUMP START ---\n");
        _checkpoint.flush();
        _checkpoint.close();
    }

    if (!_sd_ready) {
        // SD wasn't available at boot — try again now
        if (!SD.begin(SD_CS_PIN)) {
            Serial.println("[LOGGER] SD still not available — dump failed");
            return false;
        }
        _sd_ready = true;
        find_filename(_dump_filename, sizeof(_dump_filename));
    }

    File dump = SD.open(_dump_filename, FILE_WRITE);
    if (!dump) {
        Serial.print("[LOGGER] Could not create "); Serial.println(_dump_filename);
        return false;
    }

    dump.print(CSV_HEADER);

    // Replay buffer in chronological order.
    // If buffer wrapped, oldest record is at _head.
    // If not wrapped, oldest record is at index 0.
    uint16_t start  = _wrapped ? _head : 0;
    uint16_t total  = _count;
    uint32_t t_last = millis();

    for (uint16_t i = 0; i < total; i++) {
        uint16_t idx = (start + i) % LOG_RAM_CAPACITY;
        write_record_csv(dump, &_buf[idx]);

        // Progress print every 500 records
        if ((i + 1) % 500 == 0) {
            Serial.print("[LOGGER] Wrote "); Serial.print(i + 1);
            Serial.print(" / "); Serial.println(total);
        }
    }

    dump.flush();
    dump.close();

    uint32_t elapsed = millis() - t_last;
    Serial.print("[LOGGER] Dump complete: "); Serial.print(_dump_filename);
    Serial.print(" in "); Serial.print(elapsed); Serial.println(" ms");

    // Reopen checkpoint to log completion
    _checkpoint = SD.open(LOG_CHECKPOINT_FILE, FILE_WRITE);
    if (_checkpoint) {
        _checkpoint.print("--- DUMP COMPLETE: ");
        _checkpoint.print(_dump_filename);
        _checkpoint.println(" ---");
        _checkpoint.flush();
    }

    return true;
}

uint16_t logger_record_count() {
    return _count;
}

bool logger_sd_ready() {
    return _sd_ready;
}