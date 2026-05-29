#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// GD25Q128ESIGR — SPI NOR Flash (128 Mbit / 16 MB)
// Teensy 4.0 SPI1 bus
// --------------------------------------------------------
#define GD25Q128_CS_PIN     0
#define GD25Q128_MISO_PIN   1
#define GD25Q128_MOSI_PIN   26
#define GD25Q128_SCK_PIN    27

#define GD25Q128_SPI_FREQ   40000000UL   // 40 MHz — safe for short PCB traces
#define GD25Q128_SPI_MODE   SPI_MODE0

// JEDEC ID bytes
#define GD25Q128_MFR_ID     0xC8
#define GD25Q128_MEM_TYPE   0x40
#define GD25Q128_CAPACITY   0x18

// --------------------------------------------------------
// Geometry
// --------------------------------------------------------
#define FLASH_PAGE_SIZE     256u
#define FLASH_SECTOR_SIZE   4096u
#define FLASH_TOTAL_BYTES   (16u * 1024u * 1024u)

// --------------------------------------------------------
// Public API
// --------------------------------------------------------

/**
 * Configure SPI1 pins, release the chip from power-down, and verify
 * the JEDEC ID (0xC8 / 0x40 / 0x18).
 * Must be called before any other flash function.
 * @return true if GD25Q128 responds correctly; false if absent or wrong ID.
 */
bool flash_init();

/**
 * Burst-read `len` bytes starting at `addr` into `buf`.
 */
void flash_read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * Program up to 256 bytes into one page.
 * The address must be page-aligned; writes that cross a page boundary wrap
 * within the page (hardware behaviour). Blocks until WIP clears.
 */
void flash_page_program(uint32_t addr, const uint8_t *data, uint16_t len);

/**
 * Erase the 4 KB sector that contains `addr`. Blocks until WIP clears (~50 ms).
 */
void flash_erase_sector(uint32_t addr);

/**
 * Erase the entire chip (~30 s). Use during ground reset only.
 */
void flash_erase_chip();

/**
 * Poll STATUS1 until the Write-In-Progress bit clears.
 * Called internally by flash_page_program / flash_erase_*; exposed for callers
 * that need to verify the chip is idle before a time-critical operation.
 */
void flash_wait_ready();
