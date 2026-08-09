// ============================================================
//  logger.cpp
//
//  Strategy
//  --------
//  RAM ring buffer  : high-rate writes every loop tick; zero flash latency.
//
//  GD25Q128 flash   : Every FLASH_PAGE_SIZE bytes of records are flushed
//                     to flash via a 256-byte page program. The flash write
//                     pointer is persisted in sector 0 so a power loss during
//                     flight does NOT lose previously written pages.
//                     Worst-case loss = one partial page (~1-2 records).
//
//  SD card          : Disabled for this flight (H6 — SD VDD wired to 5 V).
//
//  USB dump         : On receipt of byte 'R' over Serial, Teensy streams:
//                       "TVCR" + uint16 version + uint16 record_size +
//                       uint32 record_count + raw records + uint32 CRC32
//
//  Flash memory map (GD25Q128, 16 MB):
//  ┌─────────────┬──────────────┬──────────────────────────────┐
//  │ 0x000000    │  4 KB sec 0  │ TVCR header (magic, counts…) │
//  ├─────────────┼──────────────┼──────────────────────────────┤
//  │ 0x001000    │  4 KB sec 1  │ Checkpoint text log           │
//  ├─────────────┼──────────────┼──────────────────────────────┤
//  │ 0x002000    │  ~16 MB      │ Binary LogRecord stream       │
//  └─────────────┴──────────────┴──────────────────────────────┘
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "logger.h"
#include "flash.h"

// ============================================================
//  Flash memory map
// ============================================================
#define ADDR_HEADER         0x000000u
#define ADDR_CHECKPOINT     0x001000u
#define ADDR_FLIGHT_DATA    0x002000u

#define HEADER_FLUSH_INTERVAL   64u

// ============================================================
//  TVCR flash header (packed into the first 4 KB sector)
// ============================================================
#define HEADER_MAGIC    0x54564352u  // "TVCR"
#define HEADER_VERSION  1u

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;             // 0x54564352
    uint16_t version;           // 1
    uint16_t record_size;       // sizeof(LogRecord)
    uint32_t record_count;
    uint32_t flight_epoch_ms;   // millis() at first logger_write() call
    uint32_t next_write_addr;   // internal — resume pointer after power cycle
    uint32_t checkpoint_offset; // internal — bytes used in checkpoint sector
} FlashHeader;
#pragma pack(pop)

// ============================================================
//  RAM ring buffer
// ============================================================
DMAMEM static LogRecord  _buf[LOG_RAM_CAPACITY];
static uint16_t   _head    = 0;
static uint16_t   _count   = 0;
static bool       _wrapped = false;

// ============================================================
//  Flash state
// ============================================================
static bool        _flash_ready           = false;
static FlashHeader _fhdr;
static uint8_t     _page_buf[FLASH_PAGE_SIZE];
static uint16_t    _page_buf_used         = 0;
static uint32_t    _pages_since_hdr_flush = 0;

// ============================================================
//  CRC32 (IEEE 802.3 poly 0xEDB88320)
// ============================================================
static uint32_t crc32_update(uint32_t crc, const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (crc & 1u ? 0xEDB88320u : 0u);
        }
    }
    return crc;
}

// ============================================================
//  Flash header helpers
// ============================================================
static void header_persist() {
    flash_erase_sector(ADDR_HEADER);
    flash_page_program(ADDR_HEADER,
                       (const uint8_t *)&_fhdr,
                       (uint16_t)sizeof(FlashHeader));
}

static void header_load() {
    flash_read(ADDR_HEADER, (uint8_t *)&_fhdr, sizeof(FlashHeader));
    if (_fhdr.magic != HEADER_MAGIC) {
        memset(&_fhdr, 0, sizeof(_fhdr));
        _fhdr.magic           = HEADER_MAGIC;
        _fhdr.version         = HEADER_VERSION;
        _fhdr.record_size     = (uint16_t)sizeof(LogRecord);
        _fhdr.record_count    = 0;
        _fhdr.flight_epoch_ms = 0;
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
//  Public API
// ============================================================

bool logger_init() {
    _head    = 0;
    _count   = 0;
    _wrapped = false;

    if (!flash_init()) {
        Serial.println("[LOGGER] GD25Q128 not found — flash disabled");
        _flash_ready = false;
        return false;
    }

    header_load();
    _page_buf_used         = 0;
    _pages_since_hdr_flush = 0;
    _flash_ready           = true;
    return true;
}

void logger_write(const LogRecord *rec) {
    if (!rec) return;

    // Set flight epoch on the very first record written
    if (_count == 0 && _fhdr.flight_epoch_ms == 0)
        _fhdr.flight_epoch_ms = millis();

    // RAM ring buffer — always, zero latency
    _buf[_head] = *rec;
    _head = (_head + 1) % LOG_RAM_CAPACITY;
    if (_count < LOG_RAM_CAPACITY) _count++;
    else _wrapped = true;

    // Flash page buffer — coalesce records into 256-byte pages
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
    // Flush page buffer on state transitions — safest moment for the SPI hit
    if (_flash_ready && _page_buf_used > 0) {
        memset(_page_buf + _page_buf_used, 0xFF, FLASH_PAGE_SIZE - _page_buf_used);
        _page_buf_used = FLASH_PAGE_SIZE;
        flush_page_buffer();
        header_persist();
    }

    // Append a text line to the flash checkpoint sector
    if (_flash_ready && _fhdr.checkpoint_offset < (FLASH_SECTOR_SIZE - 80u)) {
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

    Serial.print("[LOGGER] Checkpoint: ");
    Serial.print(STATE_NAMES[(int)state]);
    Serial.print("  ALT=");
    Serial.print(altitude_m, 1);
    Serial.print(" m  records=");
    Serial.println(_fhdr.record_count);
}

void logger_finalize() {
    if (!_flash_ready) return;

    // Flush any partial page
    if (_page_buf_used > 0) {
        memset(_page_buf + _page_buf_used, 0xFF, FLASH_PAGE_SIZE - _page_buf_used);
        _page_buf_used = FLASH_PAGE_SIZE;
        flush_page_buffer();
    }

    // Write final header with accurate record count and epoch
    header_persist();
    Serial.print("[LOGGER] Finalized: ");
    Serial.print(_fhdr.record_count);
    Serial.println(" records on flash");
}

void logger_usb_dump() {
    logger_finalize();

    if (!_flash_ready || _fhdr.record_count == 0) {
        Serial.println("[LOGGER] Nothing to dump");
        return;
    }

    uint32_t count    = _fhdr.record_count;
    uint32_t rec_size = sizeof(LogRecord);
    uint32_t payload  = count * rec_size;

    Serial.print("[LOGGER] Streaming ");
    Serial.print(count);
    Serial.print(" records (");
    Serial.print(payload);
    Serial.println(" bytes)...");

    // ── TVCR protocol header ──────────────────────────────────────
    // magic (4) + version (2) + record_size (2) + record_count (4) = 12 bytes
    uint8_t hdr[12];
    hdr[0]  = 'T'; hdr[1]  = 'V'; hdr[2]  = 'C'; hdr[3]  = 'R';
    hdr[4]  = (uint8_t)(HEADER_VERSION & 0xFF);
    hdr[5]  = (uint8_t)(HEADER_VERSION >> 8);
    hdr[6]  = (uint8_t)(rec_size & 0xFF);
    hdr[7]  = (uint8_t)(rec_size >> 8);
    hdr[8]  = (uint8_t)(count & 0xFF);
    hdr[9]  = (uint8_t)((count >> 8) & 0xFF);
    hdr[10] = (uint8_t)((count >> 16) & 0xFF);
    hdr[11] = (uint8_t)((count >> 24) & 0xFF);
    Serial.write(hdr, 12);

    // ── Raw record payload + CRC32 ────────────────────────────────
    uint32_t crc      = 0xFFFFFFFFu;
    uint32_t addr     = ADDR_FLIGHT_DATA;
    uint32_t end_addr = _fhdr.next_write_addr;
    uint8_t  chunk[256];

    while (addr < end_addr) {
        uint32_t bytes = end_addr - addr;
        if (bytes > sizeof(chunk)) bytes = sizeof(chunk);
        flash_read(addr, chunk, bytes);
        Serial.write(chunk, (size_t)bytes);
        crc = crc32_update(crc, chunk, bytes);
        addr += bytes;
    }

    crc ^= 0xFFFFFFFFu;
    uint8_t crc_buf[4];
    crc_buf[0] = (uint8_t)(crc & 0xFF);
    crc_buf[1] = (uint8_t)((crc >> 8) & 0xFF);
    crc_buf[2] = (uint8_t)((crc >> 16) & 0xFF);
    crc_buf[3] = (uint8_t)((crc >> 24) & 0xFF);
    Serial.write(crc_buf, 4);

    Serial.println("[LOGGER] USB dump complete");
}

uint16_t logger_record_count() { return _count; }
