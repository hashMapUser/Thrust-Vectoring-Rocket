// ============================================================
//  logger.cpp
//
//  Strategy
//  --------
//  RAM ring buffer  : the only store during flight. High-rate writes every
//                     loop tick, zero SD latency. Holds the most recent
//                     LOG_RAM_CAPACITY records (~32 s at 125 Hz); older
//                     records are overwritten once the buffer wraps.
//
//  GD25Q128 flash    : Not used. This board's flash is wired incorrectly
//                     and cannot be read or written.
//
//  SD card           : Written at two points only —
//                       1. A checkpoint text line on every FSM state
//                          change, appended immediately to FLIGHT_XXX.LOG
//                          so it survives a power loss before finalize().
//                       2. The full RAM buffer dumped to FLIGHT_XXX.CSV
//                          once, on logger_finalize() (landing).
// ============================================================

#include <Arduino.h>
#include <SD.h>
#include <string.h>
#include <stdio.h>
#include "logger.h"
#include "board_pins.h"

// ============================================================
//  RAM ring buffer
// ============================================================
DMAMEM static LogRecord  _buf[LOG_RAM_CAPACITY];
static uint16_t   _head             = 0;
static uint16_t   _count            = 0;
static bool       _wrapped          = false;
static uint32_t   _flight_epoch_ms  = 0;

// ============================================================
//  SD state
// ============================================================
static bool _sd_ready  = false;
static bool _finalized = false;
static char _csv_name[20];
static char _log_name[20];

// ============================================================
//  Pick the next free FLIGHT_XXX.CSV / .LOG slot
// ============================================================
static void find_filenames() {
    for (int i = 1; i <= 999; i++) {
        snprintf(_csv_name, sizeof(_csv_name), "FLIGHT_%03d.CSV", i);
        if (!SD.exists(_csv_name)) {
            snprintf(_log_name, sizeof(_log_name), "FLIGHT_%03d.LOG", i);
            return;
        }
    }
    // All 999 slots taken — fall back to overwriting the last one.
    strcpy(_csv_name, "FLIGHT_999.CSV");
    strcpy(_log_name, "FLIGHT_999.LOG");
}

// ============================================================
//  CSV output
// ============================================================
static const char *CSV_HEADER =
    "timestamp_ms,roll,pitch,yaw,q0,q1,q2,q3,gx,gy,gz,ax,ay,az,"
    "mx,my,mz,temperature_c,pressure_hpa,altitude_m,velocity_ms,"
    "servo_pitch_us,servo_yaw_us,pid_pitch_out,pid_yaw_out,"
    "flight_state,imu_valid,baro_valid,mag_valid\n";

static void write_csv_row(File &f, const LogRecord &r) {
    char line[256];
    int n = snprintf(line, sizeof(line),
        "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.4f,%.4f,%u,%d,%d,%d\n",
        (unsigned long)r.timestamp_ms, r.roll, r.pitch, r.yaw,
        r.q0, r.q1, r.q2, r.q3,
        r.gx, r.gy, r.gz, r.ax, r.ay, r.az,
        r.mx, r.my, r.mz,
        r.temperature_c, r.pressure_hpa, r.altitude_m, r.velocity_ms,
        r.servo_pitch_us, r.servo_yaw_us, r.pid_pitch_out, r.pid_yaw_out,
        (unsigned)r.flight_state, (int)r.imu_valid, (int)r.baro_valid, (int)r.mag_valid);
    if (n > 0) f.write((const uint8_t *)line, (size_t)n);
}

// ============================================================
//  Public API
// ============================================================

bool logger_init() {
    _head            = 0;
    _count           = 0;
    _wrapped         = false;
    _finalized       = false;
    _flight_epoch_ms = 0;

    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);

    if (!SD.begin(PIN_SD_CS)) {
        Serial.println("[LOGGER] SD.begin() failed — logging to RAM only, nothing will be saved");
        _sd_ready = false;
        return false;
    }

    find_filenames();
    Serial.print("[LOGGER] SD ready — this flight will write ");
    Serial.println(_csv_name);
    _sd_ready = true;
    return true;
}

void logger_write(const LogRecord *rec) {
    if (!rec) return;

    if (_count == 0 && _flight_epoch_ms == 0)
        _flight_epoch_ms = millis();

    _buf[_head] = *rec;
    _head = (_head + 1) % LOG_RAM_CAPACITY;
    if (_count < LOG_RAM_CAPACITY) _count++;
    else _wrapped = true;
}

void logger_checkpoint(FlightState state, float altitude_m) {
    Serial.print("[LOGGER] Checkpoint: ");
    Serial.print(STATE_NAMES[(int)state]);
    Serial.print("  ALT=");
    Serial.print(altitude_m, 1);
    Serial.print(" m  records=");
    Serial.println(_count);

    if (!_sd_ready) return;

    File f = SD.open(_log_name, FILE_WRITE);
    if (!f) {
        Serial.println("[LOGGER] Checkpoint write failed — could not open log file");
        return;
    }
    f.print("T=");     f.print(millis());
    f.print("  ");      f.print(STATE_NAMES[(int)state]);
    f.print("  ALT=");  f.print(altitude_m, 1);
    f.println(" m");
    f.close();
}

void logger_finalize() {
    if (_finalized) return;
    _finalized = true;

    Serial.print("[LOGGER] Finalizing: ");
    Serial.print(_count);
    Serial.println(" records in RAM buffer");

    if (!_sd_ready) {
        Serial.println("[LOGGER] SD not ready — nothing written");
        return;
    }

    File f = SD.open(_csv_name, FILE_WRITE);
    if (!f) {
        Serial.println("[LOGGER] Could not open CSV file for writing");
        return;
    }

    f.print(CSV_HEADER);

    // Oldest record first: if the buffer wrapped, the oldest is at _head.
    uint16_t start = _wrapped ? _head : 0;
    for (uint16_t i = 0; i < _count; i++) {
        uint16_t idx = (start + i) % LOG_RAM_CAPACITY;
        write_csv_row(f, _buf[idx]);
    }

    f.flush();
    f.close();

    Serial.print("[LOGGER] Wrote ");
    Serial.print(_count);
    Serial.print(" records to ");
    Serial.println(_csv_name);
}

uint16_t logger_record_count() { return _count; }
