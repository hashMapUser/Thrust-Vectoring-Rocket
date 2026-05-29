#include <Arduino.h>
#include <SPI.h>
#include "flash.h"

// JEDEC command set (compatible with all GD25Qxxx parts)
#define CMD_WRITE_ENABLE    0x06
#define CMD_PAGE_PROGRAM    0x02
#define CMD_READ_DATA       0x03
#define CMD_SECTOR_ERASE    0x20   // 4 KB
#define CMD_CHIP_ERASE      0xC7
#define CMD_READ_STATUS1    0x05
#define CMD_JEDEC_ID        0x9F
#define CMD_RELEASE_PD      0xAB   // release from deep power-down

#define STATUS_WIP          (1u << 0)

static const SPISettings kCfg(GD25Q128_SPI_FREQ, MSBFIRST, GD25Q128_SPI_MODE);

static inline void cs_low()  { digitalWriteFast(GD25Q128_CS_PIN, LOW);  }
static inline void cs_high() { digitalWriteFast(GD25Q128_CS_PIN, HIGH); }

static void write_enable() {
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_WRITE_ENABLE);
    cs_high();
    SPI1.endTransaction();
}

// --------------------------------------------------------
// Public API
// --------------------------------------------------------

void flash_wait_ready() {
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_READ_STATUS1);
    while (SPI1.transfer(0x00) & STATUS_WIP) {}
    cs_high();
    SPI1.endTransaction();
}

bool flash_init() {
    pinMode(GD25Q128_CS_PIN, OUTPUT);
    digitalWriteFast(GD25Q128_CS_PIN, HIGH);

    SPI1.setMOSI(GD25Q128_MOSI_PIN);
    SPI1.setMISO(GD25Q128_MISO_PIN);
    SPI1.setSCK(GD25Q128_SCK_PIN);
    SPI1.begin();

    // Release from power-down; safe no-op if already awake
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_RELEASE_PD);
    cs_high();
    SPI1.endTransaction();
    delayMicroseconds(30);  // tRES1: 20 µs max

    // Verify JEDEC ID
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_JEDEC_ID);
    uint8_t mfr = SPI1.transfer(0x00);
    uint8_t mem = SPI1.transfer(0x00);
    uint8_t cap = SPI1.transfer(0x00);
    cs_high();
    SPI1.endTransaction();

    if (mfr != GD25Q128_MFR_ID || mem != GD25Q128_MEM_TYPE || cap != GD25Q128_CAPACITY) {
        Serial.print("[FLASH] JEDEC mismatch: 0x");
        Serial.print(mfr, HEX); Serial.print(" 0x");
        Serial.print(mem, HEX); Serial.print(" 0x");
        Serial.println(cap, HEX);
        return false;
    }
    Serial.println("[FLASH] GD25Q128 found (0xC8 0x40 0x18)");
    return true;
}

void flash_read(uint32_t addr, uint8_t *buf, uint32_t len) {
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_READ_DATA);
    SPI1.transfer((addr >> 16) & 0xFF);
    SPI1.transfer((addr >>  8) & 0xFF);
    SPI1.transfer((addr >>  0) & 0xFF);
    for (uint32_t i = 0; i < len; i++) buf[i] = SPI1.transfer(0x00);
    cs_high();
    SPI1.endTransaction();
}

void flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len) {
    write_enable();
    flash_wait_ready();
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_PAGE_PROGRAM);
    SPI1.transfer((addr >> 16) & 0xFF);
    SPI1.transfer((addr >>  8) & 0xFF);
    SPI1.transfer((addr >>  0) & 0xFF);
    for (uint16_t i = 0; i < len; i++) SPI1.transfer(data[i]);
    cs_high();
    SPI1.endTransaction();
    flash_wait_ready();
}

void flash_erase_sector(uint32_t addr) {
    write_enable();
    flash_wait_ready();
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_SECTOR_ERASE);
    SPI1.transfer((addr >> 16) & 0xFF);
    SPI1.transfer((addr >>  8) & 0xFF);
    SPI1.transfer((addr >>  0) & 0xFF);
    cs_high();
    SPI1.endTransaction();
    flash_wait_ready();
}

void flash_erase_chip() {
    write_enable();
    flash_wait_ready();
    SPI1.beginTransaction(kCfg);
    cs_low();
    SPI1.transfer(CMD_CHIP_ERASE);
    cs_high();
    SPI1.endTransaction();
    flash_wait_ready();
}
