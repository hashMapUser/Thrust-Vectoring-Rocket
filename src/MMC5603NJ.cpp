#include <Wire.h>
#include <math.h>
#include "MMC5603NJ.h"


// --------------------------------------------------------
// PRIVATE HELPERS — I2C
// --------------------------------------------------------

/**
 * Write a single byte to a register.
 * Returns true on success.
 */
static bool write_register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MMC5603NJ_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

/**
 * Burst-read `length` bytes starting at `reg` into `buf`.
 * Returns true on success.
 */
static bool read_registers(uint8_t reg, uint8_t length, uint8_t *buf) {
    Wire.beginTransmission(MMC5603NJ_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t received = Wire.requestFrom((uint8_t)MMC5603NJ_ADDRESS, length);
    if (received != length) return false;

    for (uint8_t i = 0; i < length; i++) {
        buf[i] = Wire.read();
    }

    return true;
}

bool MMC5603NJ_init() {
    Wire.setClock(MMC5603NJ_I2C_CLOCK);

    // 1. Verify chip ID
    uint8_t chip_id = 0;
    if (!read_registers(MMC5603NJ_REG_PROD_ID, 1, &chip_id)) return false;
    if (chip_id != MMC5603NJ_CHIP_ID) return false;


    // Set ODR to 1-75Hz 
    // 4. Enable pressure + temperature in normal (continuous) mode
    uint8_t pwr = BMP390_PRESS_EN | BMP390_TEMP_EN | BMP390_MODE_NORMAL;
    if (!write_register(BMP390_REG_PWR_CTRL, pwr)) return false;

    return true;
}