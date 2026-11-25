// ============================================================
//  logger.cpp
//
//  Strategy
//  --------
//  RAM ring buffer  : every record, every loop tick. Holds the most recent
//                     LOG_RAM_CAPACITY records (~32 s at 125 Hz), and is
//                     the queue that feeds the flash.
//
//  GD25Q128 flash   : from the SW401 arming edge (or liftoff, if it flew
//                     with the switch open) — plus the
//                     LOG_FLASH_PRELAUNCH_RECORDS before it — every record
//                     is streamed into the flight-log region. At most one
//                     non-blocking flash operation per loop tick (program
//                     one page, or start one block erase), so the control
//                     loop never waits on the chip. Blocks are erased just
//                     ahead of the write position. Flash survives a reset
//                     and a power loss: after a watchdog reset the log
//                     carries on where it stopped, and a log that never
//                     reached the SD card is copied there at the next boot.
//
//                     A log not yet on the card is never erased: disarming
//                     pauses it, and the next arming, an unarmed liftoff or
//                     a watchdog resume append to it. The one exception is
//                     this recording's own quiet pad wait: if the region
//                     fills while armed with no launch trigger seen, the
//                     log starts over (LOG_FLASH_RESTART_RECORDS).
//
//  SD card          : Written at three points —
//                       1. A checkpoint text line on every FSM state
//                          change, appended immediately to FLIGHT_XXX.LOG
//                          so it survives a power loss before finalize().
//                       2. The flight copied to FLIGHT_XXX.CSV once, on
//                          logger_finalize() (landing) — from flash when it
//                          holds the flight, otherwise from the RAM buffer.
//                       3. At boot, a flight left in flash but not on the card.
//
//  Flash log layout (from LOG_FLASH_BASE)
//  --------------------------------------
//  Page 0   FlashLogHeader. Its words are programmed one at a time as the
//           log moves on (recording -> complete -> saved to SD): NOR flash
//           turns 1s into 0s without an erase, so each step is a small
//           program of a word that's still erased.
//  Page 1.. Records, FLOG_RECS_PER_PAGE per page. A slot whose timestamp
//           reads 0xFFFFFFFF is empty, and the first page with no records
//           ends the log. Writing always leaves at least one erased page
//           after the last page written, so that end marker exists even
//           after a power loss.
// ============================================================

#include <Arduino.h>
#include <SdFat.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include "logger.h"
#include "board_pins.h"
#include "flash.h"

// SdFat, bound explicitly to &SPI1 — the SD card is wired to SPI1
// (pins 0/1/26/27). The Arduino SD wrapper's begin(csPin) has no way to
// select a non-default bus: it always drives the default SPI object
// (SPI0, pins 11/12/13 — the IMU's bus) regardless of which CS pin is
// passed. SdSpiConfig's explicit port argument is the only way around that.
#define SD_CLOCK_MHZ 16
#define SD_CONFIG SdSpiConfig(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(SD_CLOCK_MHZ), &SPI1)

static SdFs _sd;

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
static void (*_keepalive)() = nullptr;

// ============================================================
//  Flash log state
// ============================================================
#define FLOG_MAGIC          0x474F4C46u   // "FLOG"
#define FLOG_COMPLETE       0x454E4F44u   // "DONE"
#define FLOG_SAVED          0x45564153u   // "SAVE"
#define FLOG_EMPTY_TS       0xFFFFFFFFu
#define FLOG_RECS_PER_PAGE  ((uint16_t)(FLASH_PAGE_SIZE / sizeof(LogRecord)))
#define FLOG_DATA_START     (LOG_FLASH_BASE + FLASH_PAGE_SIZE)
#define FLOG_END            (LOG_FLASH_BASE + LOG_FLASH_SIZE)

static_assert(FLASH_PAGE_SIZE / sizeof(LogRecord) >= 1, "a LogRecord must fit in one flash page");
static_assert(LOG_FLASH_BASE % FLASH_BLOCK_SIZE == 0 && LOG_FLASH_SIZE % FLASH_BLOCK_SIZE == 0,
              "the flash log region must be whole 64 KB blocks");
static_assert(LOG_FLASH_PRELAUNCH_RECORDS < LOG_RAM_CAPACITY, "pre-launch records must fit in the RAM ring");
static_assert(LOG_FLASH_RESTART_RECORDS + 500 < LOG_RAM_CAPACITY, "restart records must leave the flash room to catch up");

typedef struct {
    uint32_t magic;              // FLOG_MAGIC once recording starts
    uint16_t record_size;        // sizeof(LogRecord) — a log in another format isn't read
    uint16_t records_per_page;
    uint32_t complete;           // FLOG_COMPLETE once logger_finalize() closed it out
    uint32_t saved;              // FLOG_SAVED once it's on the SD card
} FlashLogHeader;

typedef enum {
    FL_OFF,         // no flash chip — RAM only
    FL_IDLE,        // up, not recording (on the pad)
    FL_RECORDING,
    FL_CLOSED,      // flight closed out, or the region filled up
} FlashLogState;

static FlashLogState _fl_state       = FL_OFF;
static uint32_t      _fl_write       = 0;      // next page to program
static uint32_t      _fl_erased_to   = 0;      // everything from _fl_write up to here is erased
static bool          _fl_header_due  = false;
static uint16_t      _fl_backlog     = 0;      // records in the RAM ring not yet in flash
static uint32_t      _fl_records     = 0;      // records written to flash this flight
static uint32_t      _fl_dropped     = 0;      // records that left the ring before reaching flash
static bool          _fl_unsaved     = false;  // flash holds a log that isn't on the SD card
static bool          _fl_has_old     = false;  // the log holds data from before this recording began
static bool          _fl_wrap_ok     = false;  // logger_allow_wrap(): may start over when full
static uint16_t      _fl_unflashed   = 0;      // records stored since the flash last recorded (none of them in it)

// ============================================================
//  Pick the next free FLIGHT_XXX.CSV / .LOG slot
// ============================================================
static void find_filenames() {
    for (int i = 1; i <= 999; i++) {
        snprintf(_csv_name, sizeof(_csv_name), "FLIGHT_%03d.CSV", i);
        if (!_sd.exists(_csv_name)) {
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

// Print& rather than FsFile& — both FsFile (via SdFat's Stream base) and
// Serial (via HardwareSerial's Stream base) implement Print, so this same
// function serialises a row to either the SD card or straight over USB.
static void write_csv_row(Print &out, const LogRecord &r) {
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
    if (n > 0) out.write((const uint8_t *)line, (size_t)n);
}

// ============================================================
//  Flash log
// ============================================================

// Wait for the chip to finish, feeding the watchdog. Only used where the
// loop may block: boot, landing, a USB dump.
static bool flog_wait(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (flash_busy()) {
        if (_keepalive) _keepalive();
        if (millis() - t0 > timeout_ms) return false;
    }
    return true;
}

static void flog_read_header(FlashLogHeader *h) {
    flash_read(LOG_FLASH_BASE, (uint8_t *)h, sizeof(*h));
}

static bool flog_header_valid(const FlashLogHeader &h) {
    return h.magic == FLOG_MAGIC && h.record_size == sizeof(LogRecord) &&
           h.records_per_page == FLOG_RECS_PER_PAGE;
}

// Program one still-erased header word.
static void flog_mark(size_t offset, uint32_t value) {
    if (!flog_wait(500)) return;
    flash_page_program_start(LOG_FLASH_BASE + offset, (const uint8_t *)&value, sizeof(value));
    flog_wait(500);
}

// Find the chip and see what the log region holds. For a log not yet on
// the card, find where it ends (the first page with no records) so it can
// be appended to later without scanning in the flight loop. Feeds the
// watchdog — the scan can take ~0.5 s on a full region.
static void flog_bring_up() {
    _fl_state     = FL_OFF;
    _fl_unsaved   = false;
    _fl_write     = FLOG_DATA_START;
    _fl_erased_to = LOG_FLASH_BASE;   // nothing known to be erased yet
    _fl_backlog   = 0;
    _fl_records   = 0;
    if (!flash_init()) {
        Serial.println("[LOGGER] Flash not responding — the flight log is RAM only.");
        return;
    }
    FlashLogHeader h;
    flog_read_header(&h);
    _fl_unsaved = flog_header_valid(h) && h.saved != FLOG_SAVED;
    _fl_state   = FL_IDLE;
    if (!_fl_unsaved) return;

    uint32_t addr = FLOG_DATA_START;
    for (; addr < FLOG_END; addr += FLASH_PAGE_SIZE) {
        uint32_t ts;
        flash_read(addr, (uint8_t *)&ts, sizeof(ts));
        if (ts == FLOG_EMPTY_TS) break;
        if (_keepalive && ((addr >> 8) & 0xFF) == 0) _keepalive();
    }
    _fl_write   = addr;
    // The rest of this block is still erased; anything past it is unknown.
    _fl_erased_to = LOG_FLASH_BASE + ((addr - LOG_FLASH_BASE) / FLASH_BLOCK_SIZE + 1) * FLASH_BLOCK_SIZE;
    if (_fl_erased_to > FLOG_END) _fl_erased_to = FLOG_END;
    _fl_records = (addr - FLOG_DATA_START) / FLASH_PAGE_SIZE * FLOG_RECS_PER_PAGE;
}

static void flog_begin_fresh(uint16_t seed);

// One non-blocking flash step: erase ahead, write the header, or program
// one page of records from the RAM backlog. Does nothing while the chip is
// still busy with the previous step. flush: write a part-filled last page.
static void flog_service(bool flush) {
    if (_fl_state != FL_RECORDING) return;
    if (flash_busy()) return;

    // Recording: keep a whole erased block ahead of the write position.
    if (_fl_erased_to < FLOG_END && _fl_erased_to < _fl_write + FLASH_BLOCK_SIZE) {
        flash_erase_block_start(_fl_erased_to);
        _fl_erased_to += FLASH_BLOCK_SIZE;
        return;
    }
    if (_fl_header_due) {
        FlashLogHeader h;
        memset(&h, 0xFF, sizeof(h));
        h.magic            = FLOG_MAGIC;
        h.record_size      = sizeof(LogRecord);
        h.records_per_page = FLOG_RECS_PER_PAGE;
        flash_page_program_start(LOG_FLASH_BASE, (const uint8_t *)&h, sizeof(h));
        _fl_header_due = false;
        return;
    }
    if (_fl_backlog == 0 || (_fl_backlog < FLOG_RECS_PER_PAGE && !flush)) return;

    // Leave an erased page after this one — it marks the end of the log.
    if (_fl_write + 2 * FLASH_PAGE_SIZE > _fl_erased_to) {
        if (_fl_erased_to >= FLOG_END) {
            if (_fl_wrap_ok && !_fl_has_old) {
                Serial.println("[LOGGER] Flash log full during a quiet pad wait — starting it over "
                               "with the last 20 s.");
                flog_begin_fresh(LOG_FLASH_RESTART_RECORDS);
            } else {
                _fl_state = FL_CLOSED;
                Serial.println("[LOGGER] Flash log region full — later records are in RAM only.");
            }
        }
        return;
    }

    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    uint16_t n   = _fl_backlog < FLOG_RECS_PER_PAGE ? _fl_backlog : FLOG_RECS_PER_PAGE;
    uint16_t idx = (uint16_t)((_head + LOG_RAM_CAPACITY - _fl_backlog) % LOG_RAM_CAPACITY);
    for (uint16_t i = 0; i < n; i++)
        memcpy(page + i * sizeof(LogRecord), &_buf[(idx + i) % LOG_RAM_CAPACITY], sizeof(LogRecord));
    flash_page_program_start(_fl_write, page, FLASH_PAGE_SIZE);
    _fl_write   += FLASH_PAGE_SIZE;
    _fl_backlog -= n;
    _fl_records += n;
}

// Start a new log at the start of the region: block 0 (the old header
// with it) is erased first, then the header goes down and the records
// follow. `seed` records from the RAM ring go in first — at least the
// backlog still waiting.
static void flog_begin_fresh(uint16_t seed) {
    uint16_t backlog = _count < seed ? _count : seed;
    if (backlog < _fl_backlog) backlog = _fl_backlog;
    _fl_write      = FLOG_DATA_START;
    _fl_erased_to  = LOG_FLASH_BASE;
    _fl_header_due = true;
    _fl_backlog    = backlog;
    _fl_records    = 0;
    _fl_dropped    = 0;
    _fl_has_old    = false;
    _fl_unsaved    = true;               // this log, until it's on the card
    _fl_state      = FL_RECORDING;
}

// Start recording. A log already in flash but not on the SD card is
// appended to, never erased; otherwise a new log starts. The end of an
// existing log is already known (flog_bring_up() or the last recording),
// so this never blocks.
static void flog_open(uint16_t seed) {
    if (_fl_state != FL_IDLE) return;
    if (_fl_unsaved) {
        // Only records the log doesn't already have — after a pause the
        // newest ones in the ring may already be in flash.
        if (seed > _fl_unflashed) seed = _fl_unflashed;
        _fl_header_due = false;
        _fl_backlog    = _count < seed ? _count : seed;
        _fl_dropped    = 0;
        _fl_has_old    = true;
        _fl_state      = FL_RECORDING;
        Serial.print("[LOGGER] Flash log recording — appending to the log already in flash "
                     "(not on an SD card yet), after ");
        Serial.print(_fl_records);
        Serial.println(" records.");
        return;
    }
    flog_begin_fresh(seed);
    Serial.print("[LOGGER] Flash log recording, starting ");
    Serial.print(_fl_backlog);
    Serial.println(" records back.");
}

// Write out the backlog. Blocks, feeding the watchdog. False if the flash
// stopped responding (the log is then given up — RAM only).
static bool flog_flush() {
    uint32_t t0 = millis();
    while (_fl_state == FL_RECORDING && (_fl_backlog > 0 || _fl_header_due)) {
        flog_service(true);
        if (_keepalive) _keepalive();
        if (millis() - t0 > 5000) {
            Serial.println("[LOGGER] Flash stopped responding — the flight log is RAM only.");
            _fl_state = FL_OFF;
            return false;
        }
    }
    return flog_wait(500);
}

// Disarmed on the pad: write out what's waiting and stop. The log stays
// in flash; the next recording appends to it.
static void flog_pause() {
    if (_fl_state != FL_RECORDING) return;
    if (!flog_flush()) return;
    if (_fl_state == FL_RECORDING) _fl_state = FL_IDLE;
    Serial.print("[LOGGER] Flash log paused: ");
    Serial.print(_fl_records);
    Serial.println(" records kept in flash.");
}

// Landing: write out the backlog and mark the flight complete. Blocks,
// feeding the watchdog. Returns true if flash holds this flight.
static bool flog_close() {
    if (_fl_state == FL_CLOSED) return _fl_records > 0;   // region filled up in flight
    if (_fl_state == FL_IDLE)   return _fl_unsaved && _fl_records > 0;   // paused, e.g. disarmed
    if (_fl_state != FL_RECORDING) return false;

    if (!flog_flush()) return false;
    if (_fl_state == FL_RECORDING) {
        flog_mark(offsetof(FlashLogHeader, complete), FLOG_COMPLETE);
        _fl_state = FL_CLOSED;
    }
    // Every page up to the write position should hold records — an empty
    // one would end the log early when it's read back.
    {
        uint32_t empty = 0, first_empty = 0;
        for (uint32_t a = FLOG_DATA_START; a < _fl_write; a += FLASH_PAGE_SIZE) {
            uint32_t ts;
            flash_read(a, (uint8_t *)&ts, sizeof(ts));
            if (ts == FLOG_EMPTY_TS) { if (!empty) first_empty = a; empty++; }
            if (_keepalive && ((a >> 8) & 0x3F) == 0) _keepalive();
        }
        if (empty) {
            Serial.print("[LOGGER] WARNING: ");
            Serial.print(empty);
            Serial.print(" flash log page(s) didn't program, first at 0x");
            Serial.println(first_empty, HEX);
        }
    }
    Serial.print("[LOGGER] Flash log closed: ");
    Serial.print(_fl_records);
    Serial.print(" records");
    if (_fl_dropped) {
        Serial.print(", ");
        Serial.print(_fl_dropped);
        Serial.print(" lost (flash fell behind)");
    }
    Serial.println(".");
    return _fl_records > 0;
}

// Every record in the flash log, as CSV rows. Returns how many. When this
// session wrote the log, its end is known (_fl_write) and a page that
// didn't program is just skipped; otherwise (a log from an earlier boot)
// the first page with no records is taken as the end.
static uint32_t flog_write_csv(Print &out) {
    static uint8_t page[FLASH_PAGE_SIZE];
    const bool     end_known = (_fl_state != FL_OFF && _fl_write > FLOG_DATA_START);
    const uint32_t end       = end_known ? _fl_write : FLOG_END;
    uint32_t n = 0;
    for (uint32_t addr = FLOG_DATA_START; addr < end; addr += FLASH_PAGE_SIZE) {
        flash_read(addr, page, FLOG_RECS_PER_PAGE * sizeof(LogRecord));
        bool any = false;
        for (uint16_t i = 0; i < FLOG_RECS_PER_PAGE; i++) {
            LogRecord r;
            memcpy(&r, page + i * sizeof(LogRecord), sizeof(r));
            if (r.timestamp_ms == FLOG_EMPTY_TS) continue;
            any = true;
            write_csv_row(out, r);
            n++;
            if (_keepalive) _keepalive();
        }
        if (!any && !end_known) break;   // first empty page: end of the log
    }
    return n;
}

// Copy the flash log to `name` on the SD card and mark it saved.
static bool flog_copy_to_sd(const char *name) {
    if (!flog_wait(3000)) return false;
    FsFile f = _sd.open(name, O_WRONLY | O_CREAT | O_TRUNC);
    if (!f) return false;
    f.print(CSV_HEADER);
    uint32_t n = flog_write_csv(f);
    f.sync();
    bool ok = !f.getWriteError();
    f.close();
    if (!ok) return false;
    flog_mark(offsetof(FlashLogHeader, saved), FLOG_SAVED);
    _fl_unsaved = false;
    Serial.print("[LOGGER] Wrote ");
    Serial.print(n);
    Serial.print(" records from flash to ");
    Serial.println(name);
    return true;
}

// Does flash hold a flight worth dumping right now?
static bool flog_has_flight() {
    if (_fl_state == FL_OFF || _fl_state == FL_RECORDING) return false;
    if (!flog_wait(3000)) return false;
    FlashLogHeader h;
    flog_read_header(&h);
    return flog_header_valid(h);
}

static void dump_csv_over_serial(bool from_flash) {
    Serial.println(from_flash ? "[LOGGER] Streaming the flight in flash as CSV over USB serial."
                              : "[LOGGER] Streaming the RAM buffer as CSV over USB serial.");
    Serial.println("[LOGGER] Copy everything between the BEGIN/END markers into a .csv file.");
    Serial.println("-----BEGIN FLIGHT CSV-----");
    Serial.print(CSV_HEADER);
    if (from_flash) {
        flog_write_csv(Serial);
    } else {
        uint16_t start = _wrapped ? _head : 0;
        for (uint16_t i = 0; i < _count; i++) {
            uint16_t idx = (start + i) % LOG_RAM_CAPACITY;
            write_csv_row(Serial, _buf[idx]);
            if (_keepalive) _keepalive();
        }
    }
    Serial.println("-----END FLIGHT CSV-----");
}

// ============================================================
//  Public API
// ============================================================

void logger_set_keepalive(void (*keepalive)()) {
    _keepalive = keepalive;
}

static void ram_reset() {
    _head            = 0;
    _count           = 0;
    _wrapped         = false;
    _finalized       = false;
    _flight_epoch_ms = 0;
}

bool logger_init() {
    ram_reset();
    flog_bring_up();

    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);

    SPI1.setMISO(PIN_SD_MISO);
    SPI1.setMOSI(PIN_SD_MOSI);
    SPI1.setSCK(PIN_SD_SCK);

    if (!_sd.begin(SD_CONFIG)) {
        Serial.println("[LOGGER] SD init failed — nothing will be saved to a card this session");
        _sd.initErrorPrint(&Serial);   // reports CMD0/CMD8/ACMD41 stage
        if (_fl_unsaved)
            Serial.println("[LOGGER] Flash holds a flight that isn't on a card yet — insert one and "
                           "reboot to copy it, or send U to dump it over USB.");
        _sd_ready = false;
        return false;
    }
    _sd_ready = true;

    find_filenames();
    if (_fl_unsaved) {
        Serial.print("[LOGGER] Flash holds a flight that never reached the SD card — copying it to ");
        Serial.println(_csv_name);
        if (flog_copy_to_sd(_csv_name)) find_filenames();   // that slot is taken now
        else Serial.println("[LOGGER] Copy failed — it stays in flash (send U to dump it).");
    }

    Serial.print("[LOGGER] SD ready — this flight will write ");
    Serial.println(_csv_name);
    return true;
}

void logger_resume() {
    ram_reset();
    _sd_ready = false;
    flog_bring_up();
    if (_fl_state == FL_OFF) return;
    bool appending = _fl_unsaved;
    flog_open(0);
    if (appending)
        Serial.println("[LOGGER] Flash log continuing after the reset. SD card skipped — "
                       "it's copied at the next boot.");
}

void logger_allow_wrap(bool allowed) {
    _fl_wrap_ok = allowed;
}

void logger_write(const LogRecord *rec) {
    if (!rec) return;

    if (_count == 0 && _flight_epoch_ms == 0)
        _flight_epoch_ms = millis();

    _buf[_head] = *rec;
    _head = (_head + 1) % LOG_RAM_CAPACITY;
    if (_count < LOG_RAM_CAPACITY) _count++;
    else _wrapped = true;

    if (_fl_state == FL_RECORDING) {
        if (_fl_backlog < LOG_RAM_CAPACITY - 1) _fl_backlog++;
        else _fl_dropped++;   // flash fell a whole ring behind: the oldest unwritten record is gone
        _fl_unflashed = 0;
    } else if (_fl_unflashed < 0xFFFF) {
        _fl_unflashed++;
    }
    flog_service(false);
}

void logger_checkpoint(FlightState state, float altitude_m) {
    Serial.print("[LOGGER] Checkpoint: ");
    Serial.print(STATE_NAMES[(int)state]);
    Serial.print("  ALT=");
    Serial.print(altitude_m, 1);
    Serial.print(" m  records=");
    Serial.println(_count);

    if (state == STATE_ARMED)   flog_open(LOG_FLASH_PRELAUNCH_RECORDS);
    if (state == STATE_IDLE)    flog_pause();                          // disarmed on the pad
    if (state == STATE_POWERED) flog_open(LOG_FLASH_PRELAUNCH_RECORDS); // flew with SW401 open

    if (!_sd_ready) return;

    FsFile f = _sd.open(_log_name, O_WRONLY | O_CREAT | O_APPEND);
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

    bool from_flash = flog_close();

    Serial.print("[LOGGER] Finalizing: ");
    if (from_flash) {
        Serial.print(_fl_records);
        Serial.println(" records in flash");
    } else {
        Serial.print(_count);
        Serial.println(" records in RAM buffer");
    }

    if (!_sd_ready) {
        if (from_flash)
            Serial.println("[LOGGER] No SD card this session — the flight stays in flash and is "
                           "copied to the card at the next boot with one inserted.");
        dump_csv_over_serial(from_flash);
        return;
    }

    if (from_flash) {
        if (flog_copy_to_sd(_csv_name)) return;
        Serial.println("[LOGGER] Copy from flash failed — writing the RAM buffer instead "
                       "(the flight stays in flash for the next boot).");
    }

    FsFile f = _sd.open(_csv_name, O_WRONLY | O_CREAT | O_TRUNC);
    if (!f) {
        Serial.println("[LOGGER] Could not open CSV file for writing — falling back to USB serial");
        dump_csv_over_serial(false);
        return;
    }

    f.print(CSV_HEADER);

    // Oldest record first: if the buffer wrapped, the oldest is at _head.
    uint16_t start = _wrapped ? _head : 0;
    for (uint16_t i = 0; i < _count; i++) {
        uint16_t idx = (start + i) % LOG_RAM_CAPACITY;
        write_csv_row(f, _buf[idx]);
        if (_keepalive) _keepalive();
    }

    f.sync();
    f.close();

    Serial.print("[LOGGER] Wrote ");
    Serial.print(_count);
    Serial.print(" records to ");
    Serial.println(_csv_name);
}

void logger_usb_dump() {
    bool was_recording = (_fl_state == FL_RECORDING);
    if (was_recording) {
        Serial.println("[LOGGER] Pausing the flash log for the dump — the flight loop is blocked until it's done.");
        flog_pause();
    }
    dump_csv_over_serial(flog_has_flight());
    if (was_recording) flog_open(0);
}

uint16_t logger_record_count() { return _count; }

void logger_print_csv_header(Print &out) { out.print(CSV_HEADER); }

void logger_print_csv_row(Print &out, const LogRecord *rec) {
    if (rec) write_csv_row(out, *rec);
}
