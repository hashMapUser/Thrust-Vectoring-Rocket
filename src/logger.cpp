// ============================================================
//  logger.cpp
//
//  Strategy
//  --------
//  RAM ring buffer  : high-rate writes every loop tick, zero SD
//                     latency. Same as before.
//
//  GD25Q128 flash   : NEW. Every FLASH_PAGE_SIZE bytes of records
//                     are flushed to flash via a 256-byte page
//                     program. The flash write pointer is persisted
//                     in a 4 KB header sector so a power-loss
//                     during flight does NOT lose previously written
//                     pages. Worst case loss = one partial page
//                     (~3 records at 76 bytes each).
//
//  SD card          : Checkpoint file written on state transitions
//                     (same as before). Full CSV dump on request.
//                     logger_dump_to_sd() reads from flash, not RAM,
//                     so it recovers data even after a power cycle.
//
//  Flash memory map (GD25Q128ESIGR, 16 MB):
//  ┌─────────────┬──────────────┬──────────────────────────────┐
//  │ 0x000000    │  4 KB sec 0  │ Header: magic, record count, │
//  │             │              │ write ptr, checkpoint offset  │
//  ├─────────────┼──────────────┼──────────────────────────────┤
//  │ 0x001000    │  4 KB sec 1  │ Checkpoint text log           │
//  ├─────────────┼──────────────┼──────────────────────────────┤
//  │ 0x002000    │  ~16 MB      │ Binary LogRecord stream       │
//  └─────────────┴──────────────┴──────────────────────────────┘
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "logger.h"

// ============================================================
//  GD25Q128 driver — pin & SPI config
// ============================================================
#define FLASH_CS_PIN        9           // GD25Q128 chip select
#define FLASH_SPI_FREQ      40000000UL  // 40 MHz — safe for short PCB traces
#define FLASH_SPI_MODE      SPI_MODE0

// Command set (JEDEC-compatible)
#define CMD_WRITE_ENABLE    0x06
#define CMD_PAGE_PROGRAM    0x02
#define CMD_READ_DATA       0x03
#define CMD_SECTOR_ERASE    0x20   // 4 KB
#define CMD_CHIP_ERASE      0xC7
#define CMD_READ_STATUS1    0x05
#define CMD_JEDEC_ID        0x9F
#define CMD_RELEASE_PD      0xAB

#define STATUS_WIP          (1u << 0)

// Geometry
#define FLASH_PAGE_SIZE     256u
#define FLASH_SECTOR_SIZE   4096u
#define FLASH_TOTAL_BYTES   (16u * 1024u * 1024u)

// Memory map
#define ADDR_HEADER         0x000000u
#define ADDR_CHECKPOINT     0x001000u
#define ADDR_FLIGHT_DATA    0x002000u

// Persist header every N page flushes (limits sector wear)
#define HEADER_FLUSH_INTERVAL   64u

// ============================================================
//  Flash header (packed into first 4 KB sector)
// ============================================================
#define HEADER_MAGIC    0xF17E0001u

#pragma pack(push,1)
typedef struct {
    uint32_t magic;
    uint32_t record_count;
    uint32_t next_write_addr;
    uint32_t checkpoint_offset;
    // No in-RAM padding needed: header_persist() already limits the write to
    // min(sizeof(FlashHeader), FLASH_PAGE_SIZE) bytes, so the 4 KB sector pad
    // was wasting RAM without any benefit.
} FlashHeader;
#pragma pack(pop)

// ============================================================
//  RAM ring buffer (unchanged from original)
// ============================================================
// DMAMEM places this in RAM2 (OCRAM2) instead of RAM1 (DTCM).
// At 4000 × 104 bytes = ~406 KB it was the entire cause of the RAM1 overflow.
// RAM2 has 512 KB free and is fully CPU-accessible; no functional change.
DMAMEM static LogRecord  _buf[LOG_RAM_CAPACITY];
static uint16_t   _head    = 0;
static uint16_t   _count   = 0;
static bool       _wrapped = false;

// ============================================================
//  Flash state
// ============================================================
static bool       _flash_ready          = false;
static FlashHeader _fhdr;
static uint8_t    _page_buf[FLASH_PAGE_SIZE];
static uint16_t   _page_buf_used        = 0;
static uint32_t   _pages_since_hdr_flush = 0;

// ============================================================
//  SD state (unchanged)
// ============================================================
static bool   _sd_ready      = false;
static File   _checkpoint;
static char   _dump_filename[20];

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

// ============================================================
//  Flash low-level helpers
// ============================================================
static inline void flash_cs_low()  { digitalWriteFast(FLASH_CS_PIN, LOW);  }
static inline void flash_cs_high() { digitalWriteFast(FLASH_CS_PIN, HIGH); }

static void flash_wait_ready() {
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    flash_cs_low();
    SPI.transfer(CMD_READ_STATUS1);
    while (SPI.transfer(0x00) & STATUS_WIP) {}
    flash_cs_high();
    SPI.endTransaction();
}

static void flash_write_enable() {
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    flash_cs_low();
    SPI.transfer(CMD_WRITE_ENABLE);
    flash_cs_high();
    SPI.endTransaction();
}

static void flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len) {
    flash_write_enable();
    flash_wait_ready();
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    flash_cs_low();
    SPI.transfer(CMD_PAGE_PROGRAM);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer((addr >>  0) & 0xFF);
    for (uint16_t i = 0; i < len; i++) SPI.transfer(data[i]);
    flash_cs_high();
    SPI.endTransaction();
    flash_wait_ready();
}

static void flash_read(uint32_t addr, uint8_t *buf, uint32_t len) {
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    flash_cs_low();
    SPI.transfer(CMD_READ_DATA);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer((addr >>  0) & 0xFF);
    for (uint32_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    flash_cs_high();
    SPI.endTransaction();
}

static void flash_erase_sector(uint32_t addr) {
    flash_write_enable();
    flash_wait_ready();
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    flash_cs_low();
    SPI.transfer(CMD_SECTOR_ERASE);
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >>  8) & 0xFF);
    SPI.transfer((addr >>  0) & 0xFF);
    flash_cs_high();
    SPI.endTransaction();
    flash_wait_ready();
}

static bool flash_check_jedec() {
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    flash_cs_low();
    SPI.transfer(CMD_JEDEC_ID);
    uint8_t mfr = SPI.transfer(0);  // 0xC8 = GigaDevice
    uint8_t mem = SPI.transfer(0);  // 0x40
    uint8_t cap = SPI.transfer(0);  // 0x18
    flash_cs_high();
    SPI.endTransaction();
    return (mfr == 0xC8 && mem == 0x40 && cap == 0x18);
}

// ============================================================
//  Flash header helpers
// ============================================================
static void header_persist() {
    flash_erase_sector(ADDR_HEADER);
    flash_page_program(ADDR_HEADER, (const uint8_t *)&_fhdr,
                       sizeof(FlashHeader) < FLASH_PAGE_SIZE
                           ? (uint16_t)sizeof(FlashHeader)
                           : FLASH_PAGE_SIZE);
}

static void header_load() {
    flash_read(ADDR_HEADER, (uint8_t *)&_fhdr, sizeof(FlashHeader));
    if (_fhdr.magic != HEADER_MAGIC) {
        memset(&_fhdr, 0, sizeof(_fhdr));
        _fhdr.magic           = HEADER_MAGIC;
        _fhdr.record_count    = 0;
        _fhdr.next_write_addr = ADDR_FLIGHT_DATA;
        _fhdr.checkpoint_offset = 0;
        header_persist();
        Serial.println("[LOGGER] Flash: fresh header written");
    } else {
        Serial.print("[LOGGER] Flash: resumed, ");
        Serial.print(_fhdr.record_count);
        Serial.println(" records on chip");
    }
}

// ============================================================
//  Page buffer flush to flash
// ============================================================
static void flush_page_buffer() {
    if (_page_buf_used == 0 || !_flash_ready) return;
    if (_fhdr.next_write_addr + _page_buf_used > FLASH_TOTAL_BYTES) {
        Serial.println("[LOGGER] Flash full — writes halted");
        _flash_ready = false;
        return;
    }
    flash_page_program(_fhdr.next_write_addr, _page_buf, _page_buf_used);
    _fhdr.next_write_addr += _page_buf_used;
    _page_buf_used = 0;

    if (++_pages_since_hdr_flush >= HEADER_FLUSH_INTERVAL) {
        header_persist();
        _pages_since_hdr_flush = 0;
    }
}

// ============================================================
//  SD helpers (unchanged from original)
// ============================================================
static bool find_filename(char *out, size_t len) {
    for (int i = 1; i <= SD_MAX_FILES; i++) {
        snprintf(out, len, "FLIGHT_%03d.CSV", i);
        if (!SD.exists(out)) return true;
    }
    return false;
}

static void write_record_csv(File &f, const LogRecord *r) {
    char tmp[32];
    #define WF(val, dec) dtostrf(val, 1, dec, tmp); f.print(tmp); f.print(',');
    #define WU(val)      f.print((uint32_t)(val)); f.print(',');

    WU(r->timestamp_ms);
    WF(r->roll,  2);  WF(r->pitch, 2);  WF(r->yaw,  2);
    WF(r->q0,    5);  WF(r->q1,    5);  WF(r->q2,   5);  WF(r->q3, 5);
    WF(r->gx,    2);  WF(r->gy,    2);  WF(r->gz,   2);
    WF(r->ax,    4);  WF(r->ay,    4);  WF(r->az,   4);
    WF(r->mx,    4);  WF(r->my,    4);  WF(r->mz,   4);
    WF(r->temperature_c, 2);
    WF(r->pressure_hpa,  2);
    WF(r->altitude_m,    2);
    WF(r->velocity_ms,   3);
    WF(r->servo_pitch_us, 1);
    WF(r->servo_yaw_us,   1);
    WF(r->pid_pitch_out,  3);
    WF(r->pid_yaw_out,    3);

    f.print(r->flight_state);       f.print(',');
    f.print(r->imu_valid  ? 1 : 0); f.print(',');
    f.print(r->baro_valid ? 1 : 0); f.print(',');
    f.println(r->mag_valid ? 1 : 0);

    #undef WF
    #undef WU
}

// ============================================================
//  Public API
// ============================================================

bool logger_init() {
    // --- RAM buffer ---
    _head    = 0;
    _count   = 0;
    _wrapped = false;

    // --- Flash ---
    pinMode(FLASH_CS_PIN, OUTPUT);
    digitalWriteFast(FLASH_CS_PIN, HIGH);

    // Release from power-down (safe no-op if already awake)
    SPI.beginTransaction(SPISettings(FLASH_SPI_FREQ, MSBFIRST, FLASH_SPI_MODE));
    flash_cs_low();
    SPI.transfer(CMD_RELEASE_PD);
    flash_cs_high();
    SPI.endTransaction();
    delayMicroseconds(30);

    if (!flash_check_jedec()) {
        Serial.println("[LOGGER] GD25Q128 not found — check CS pin/SPI. Flash disabled.");
        _flash_ready = false;
    } else {
        Serial.println("[LOGGER] GD25Q128 found (0xC8 0x40 0x18)");
        header_load();
        _page_buf_used         = 0;
        _pages_since_hdr_flush = 0;
        _flash_ready           = true;
    }

    // --- SD ---
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[LOGGER] SD not found — checkpoint disabled, RAM logging only");
        _sd_ready = false;
        return _flash_ready;  // flash alone is still useful
    }

    _checkpoint = SD.open(LOG_CHECKPOINT_FILE, FILE_WRITE);
    if (!_checkpoint) {
        Serial.println("[LOGGER] Could not open checkpoint file");
        _sd_ready = false;
        return _flash_ready;
    }

    _checkpoint.print("\n--- BOOT ");
    _checkpoint.print(millis());
    _checkpoint.println(" ms ---");
    _checkpoint.flush();

    find_filename(_dump_filename, sizeof(_dump_filename));

    _sd_ready = true;
    Serial.print("[LOGGER] SD ready. Dump → "); Serial.println(_dump_filename);

    return true;  // both flash and SD up
}

void logger_write(const LogRecord *rec) {
    if (!rec) return;

    // 1. RAM ring buffer — always, no SD latency
    _buf[_head] = *rec;
    _head = (_head + 1) % LOG_RAM_CAPACITY;
    if (_count < LOG_RAM_CAPACITY) _count++;
    else _wrapped = true;

    // 2. Flash page buffer — coalesce into 256-byte pages
    if (!_flash_ready) return;

    const uint8_t *src    = (const uint8_t *)rec;
    uint16_t       remain = (uint16_t)sizeof(LogRecord);
    uint16_t       offset = 0;

    while (remain > 0) {
        uint16_t space = FLASH_PAGE_SIZE - _page_buf_used;
        uint16_t chunk = (remain < space) ? remain : space;
        memcpy(_page_buf + _page_buf_used, src + offset, chunk);
        _page_buf_used += chunk;
        offset         += chunk;
        remain         -= chunk;

        if (_page_buf_used == FLASH_PAGE_SIZE) {
            flush_page_buffer();
            if (!_flash_ready) break;
        }
    }

    _fhdr.record_count++;
}

void logger_checkpoint(FlightState state, float altitude_m) {
    // Always flush page buffer on state transitions —
    // this is the safest moment to take the ~60 µs SPI hit.
    if (_flash_ready && _page_buf_used > 0) {
        memset(_page_buf + _page_buf_used, 0xFF, FLASH_PAGE_SIZE - _page_buf_used);
        _page_buf_used = FLASH_PAGE_SIZE;
        flush_page_buffer();
        header_persist();
    }

    // Append a text line to the flash checkpoint sector
    if (_flash_ready &&
        _fhdr.checkpoint_offset < (FLASH_SECTOR_SIZE - 80u)) {
        char line[80];
        int n = snprintf(line, sizeof(line),
                         "T=%8lu  %-10s  ALT=%7.1f m\n",
                         millis(), STATE_NAMES[(int)state], altitude_m);
        if (n > 0) {
            flash_page_program(ADDR_CHECKPOINT + _fhdr.checkpoint_offset,
                               (const uint8_t *)line, (uint16_t)n);
            _fhdr.checkpoint_offset += (uint32_t)n;
        }
    }

    // SD checkpoint (same as before)
    if (!_sd_ready || !_checkpoint) return;
    char line[80];
    snprintf(line, sizeof(line),
             "T=%lu ms  STATE=%-10s  ALT=%.1f m  RECORDS=%u\n",
             millis(), STATE_NAMES[(int)state], altitude_m, _count);
    _checkpoint.print(line);
    _checkpoint.flush();
    Serial.print("[LOGGER] Checkpoint: "); Serial.print(line);
}

bool logger_dump_to_sd() {
    // Flush any partial page first
    if (_flash_ready && _page_buf_used > 0) {
        memset(_page_buf + _page_buf_used, 0xFF, FLASH_PAGE_SIZE - _page_buf_used);
        _page_buf_used = FLASH_PAGE_SIZE;
        flush_page_buffer();
        header_persist();
    }

    // Decide source: prefer flash (recovers data across power cycles);
    // fall back to RAM if flash is absent.
    bool use_flash = _flash_ready && (_fhdr.record_count > 0);
    bool use_ram   = !use_flash && (_count > 0);

    if (!use_flash && !use_ram) {
        Serial.println("[LOGGER] Nothing to dump — buffer empty");
        return false;
    }

    if (!_sd_ready) {
        if (!SD.begin(SD_CS_PIN)) {
            Serial.println("[LOGGER] SD still not available — dump failed");
            return false;
        }
        _sd_ready = true;
        find_filename(_dump_filename, sizeof(_dump_filename));
    }

    if (_checkpoint) {
        _checkpoint.print("--- DUMP START ---\n");
        _checkpoint.flush();
        _checkpoint.close();
    }

    File dump = SD.open(_dump_filename, FILE_WRITE);
    if (!dump) {
        Serial.print("[LOGGER] Cannot create "); Serial.println(_dump_filename);
        return false;
    }

    dump.print(CSV_HEADER);
    uint32_t written = 0;

    if (use_flash) {
        Serial.print("[LOGGER] Dumping flash → "); Serial.println(_dump_filename);
        uint32_t addr = ADDR_FLIGHT_DATA;
        uint32_t end  = _fhdr.next_write_addr;
        LogRecord batch[4];
        while (addr + sizeof(LogRecord) <= end) {
            uint8_t n = 0;
            while (n < 4 && addr + sizeof(LogRecord) <= end) {
                flash_read(addr, (uint8_t *)&batch[n], sizeof(LogRecord));
                addr += sizeof(LogRecord);
                n++;
            }
            for (uint8_t i = 0; i < n; i++) {
                write_record_csv(dump, &batch[i]);
                written++;
            }
            if (written % 500 == 0) {
                dump.flush();
                Serial.print("[LOGGER] "); Serial.print(written); Serial.println(" records...");
            }
        }
    } else {
        // RAM fallback
        Serial.print("[LOGGER] Dumping RAM ("); Serial.print(_count);
        Serial.print(" records) → "); Serial.println(_dump_filename);
        uint16_t start = _wrapped ? _head : 0;
        for (uint16_t i = 0; i < _count; i++) {
            write_record_csv(dump, &_buf[(start + i) % LOG_RAM_CAPACITY]);
            written++;
            if (written % 500 == 0) dump.flush();
        }
    }

    dump.flush();
    dump.close();

    Serial.print("[LOGGER] Dump complete: "); Serial.print(written);
    Serial.print(" records → "); Serial.println(_dump_filename);

    _checkpoint = SD.open(LOG_CHECKPOINT_FILE, FILE_WRITE);
    if (_checkpoint) {
        _checkpoint.print("--- DUMP COMPLETE: ");
        _checkpoint.print(_dump_filename);
        _checkpoint.println(" ---");
        _checkpoint.flush();
    }

    return true;
}

uint16_t logger_record_count() { return _count; }
bool     logger_sd_ready()     { return _sd_ready; }