#include <Arduino.h>
#include <SPI.h>
#include "flash.h"

// JEDEC command set (compatible with all GD25Qxxx parts)
#define CMD_WRITE_ENABLE    0x06
#define CMD_PAGE_PROGRAM    0x02
#define CMD_READ_DATA       0x03
#define CMD_SECTOR_ERASE    0x20   // 4 KB
#define CMD_BLOCK_ERASE     0xD8   // 64 KB
#define CMD_CHIP_ERASE      0xC7
#define CMD_READ_STATUS1    0x05
#define CMD_JEDEC_ID        0x9F
#define CMD_RELEASE_PD      0xAB   // release from deep power-down

#define STATUS_WIP          (1u << 0)

static const SPISettings kCfg(GD25Q128_SPI_FREQ, MSBFIRST, GD25Q128_SPI_MODE);

static inline void cs_low()  { digitalWriteFast(PIN_FLASH_CS, LOW);  }
static inline void cs_high() { digitalWriteFast(PIN_FLASH_CS, HIGH); }

static void write_enable() {
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_WRITE_ENABLE);
    cs_high();
    SPI2.endTransaction();
}

// --------------------------------------------------------
// Public API
// --------------------------------------------------------

void flash_wait_ready() {
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_READ_STATUS1);
    while (SPI2.transfer(0x00) & STATUS_WIP) {}
    cs_high();
    SPI2.endTransaction();
}

static void spi2_start() {
    SPI2.setMOSI(PIN_FLASH_MOSI);
    SPI2.setMISO(PIN_FLASH_MISO);
    SPI2.setSCK(PIN_FLASH_SCK);
    SPI2.begin();
}

static uint32_t read_jedec() {
    // Release from power-down; safe no-op if already awake
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_RELEASE_PD);
    cs_high();
    SPI2.endTransaction();
    delayMicroseconds(30);  // tRES1: 20 µs max

    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_JEDEC_ID);
    uint32_t id = (uint32_t)SPI2.transfer(0x00) << 16;
    id |= (uint32_t)SPI2.transfer(0x00) << 8;
    id |= SPI2.transfer(0x00);
    cs_high();
    SPI2.endTransaction();
    return id;
}

// Bring back a chip that's been knocked into a mode where it ignores normal
// commands. If CS# glitches mid-transfer, the chip reads the following data
// bytes as commands: 0x38 puts it in QPI mode (commands on four lines),
// where it ignores every 1-line command until it's power-cycled or sent 0xFF
// in QPI form. So: 0xFF over all four lines (IO2/IO3 are WP#/HOLD#, pulled
// high on the board; IO1 gets the Teensy's pull-up), then a software reset.
static void bb_byte(uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        digitalWriteFast(PIN_FLASH_MOSI, (b >> i) & 1);
        delayMicroseconds(2);
        digitalWriteFast(PIN_FLASH_SCK, HIGH);
        delayMicroseconds(2);
        digitalWriteFast(PIN_FLASH_SCK, LOW);
    }
}

static void flash_recover() {
    pinMode(PIN_FLASH_SCK, OUTPUT);   digitalWriteFast(PIN_FLASH_SCK, LOW);
    pinMode(PIN_FLASH_MOSI, OUTPUT);  digitalWriteFast(PIN_FLASH_MOSI, HIGH);
    pinMode(PIN_FLASH_MISO, INPUT_PULLUP);

    cs_low();  bb_byte(0xFF); cs_high();   // QPI exit: all four lines high for 2+ clocks
    delayMicroseconds(10);
    cs_low();  bb_byte(0xFF); cs_high();   // and, in SPI mode, continuous-read reset
    delayMicroseconds(10);
    cs_low();  bb_byte(0x66); cs_high();   // reset enable
    delayMicroseconds(10);
    cs_low();  bb_byte(0x99); cs_high();   // reset
    delay(1);                              // tRST: 30 us typ, longer if it was mid-erase

    pinMode(PIN_FLASH_MISO, INPUT);
    spi2_start();                          // hand the pins back to SPI2
}

bool flash_init() {
    pinMode(PIN_FLASH_CS, OUTPUT);
    digitalWriteFast(PIN_FLASH_CS, HIGH);
    spi2_start();

    const uint32_t expect = ((uint32_t)GD25Q128_MFR_ID << 16) |
                            ((uint32_t)GD25Q128_MEM_TYPE << 8) | GD25Q128_CAPACITY;
    uint32_t id = read_jedec();
    if (id != expect) {
        flash_recover();
        uint32_t again = read_jedec();
        if (again == expect) {
            Serial.print("[FLASH] GD25Q128 recovered by a reset sequence (it first read 0x");
            Serial.print(id, HEX);
            Serial.println(") — it had been left in a non-SPI mode.");
            id = again;
        }
    }

    if (id != expect) {
        Serial.print("[FLASH] JEDEC mismatch: 0x");
        Serial.print((id >> 16) & 0xFF, HEX); Serial.print(" 0x");
        Serial.print((id >> 8) & 0xFF, HEX);  Serial.print(" 0x");
        Serial.print(id & 0xFF, HEX);
        Serial.println(" (reset sequence didn't help)");
        return false;
    }
    Serial.println("[FLASH] GD25Q128 found (0xC8 0x40 0x18)");
    return true;
}

void flash_read(uint32_t addr, uint8_t *buf, uint32_t len) {
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_READ_DATA);
    SPI2.transfer((addr >> 16) & 0xFF);
    SPI2.transfer((addr >>  8) & 0xFF);
    SPI2.transfer((addr >>  0) & 0xFF);
    for (uint32_t i = 0; i < len; i++) buf[i] = SPI2.transfer(0x00);
    cs_high();
    SPI2.endTransaction();
}

void flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len) {
    write_enable();
    flash_wait_ready();
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_PAGE_PROGRAM);
    SPI2.transfer((addr >> 16) & 0xFF);
    SPI2.transfer((addr >>  8) & 0xFF);
    SPI2.transfer((addr >>  0) & 0xFF);
    for (uint16_t i = 0; i < len; i++) SPI2.transfer(data[i]);
    cs_high();
    SPI2.endTransaction();
    flash_wait_ready();
}

void flash_erase_sector(uint32_t addr) {
    write_enable();
    flash_wait_ready();
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_SECTOR_ERASE);
    SPI2.transfer((addr >> 16) & 0xFF);
    SPI2.transfer((addr >>  8) & 0xFF);
    SPI2.transfer((addr >>  0) & 0xFF);
    cs_high();
    SPI2.endTransaction();
    flash_wait_ready();
}

void flash_erase_chip() {
    write_enable();
    flash_wait_ready();
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_CHIP_ERASE);
    cs_high();
    SPI2.endTransaction();
    flash_wait_ready();
}

// --------------------------------------------------------
// Non-blocking operations
// --------------------------------------------------------

bool flash_busy() {
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_READ_STATUS1);
    uint8_t status = SPI2.transfer(0x00);
    cs_high();
    SPI2.endTransaction();
    return (status & STATUS_WIP) != 0;
}

void flash_page_program_start(uint32_t addr, const uint8_t *data, uint16_t len) {
    write_enable();
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_PAGE_PROGRAM);
    SPI2.transfer((addr >> 16) & 0xFF);
    SPI2.transfer((addr >>  8) & 0xFF);
    SPI2.transfer((addr >>  0) & 0xFF);
    SPI2.transfer(data, nullptr, len);   // bulk transfer — ~4x faster than byte by byte
    cs_high();
    SPI2.endTransaction();
}

void flash_erase_block_start(uint32_t addr) {
    write_enable();
    SPI2.beginTransaction(kCfg);
    cs_low();
    SPI2.transfer(CMD_BLOCK_ERASE);
    SPI2.transfer((addr >> 16) & 0xFF);
    SPI2.transfer((addr >>  8) & 0xFF);
    SPI2.transfer((addr >>  0) & 0xFF);
    cs_high();
    SPI2.endTransaction();
}
