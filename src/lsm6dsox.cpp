#include <Arduino.h>
#include <SPI.h>
#include "lsm6dsox.h"

// --------------------------------------------------------
// SPI SETTINGS — static instance, not recreated per-call.
// Matches the working standalone sketch exactly.
// --------------------------------------------------------
static SPISettings _spi_settings(LSM6DSOX_SPI_CLOCK, MSBFIRST, SPI_MODE3);

// --------------------------------------------------------
// PRIVATE — SPI HELPERS
// --------------------------------------------------------

static void write_register(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(_spi_settings);
    digitalWrite(LSM6DSOX_CS_PIN, LOW);
    SPI.transfer(reg & 0x7F);   // bit 7 = 0 → write (matches working code)
    SPI.transfer(value);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);
    SPI.endTransaction();
}

static uint8_t read_register(uint8_t reg) {
    SPI.beginTransaction(_spi_settings);
    digitalWrite(LSM6DSOX_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);   // bit 7 = 1 → read (matches working code)
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);
    SPI.endTransaction();
    return value;
}

/**
 * Burst-read `length` bytes starting at `reg`.
 * IF_INC=1 set in init so the sensor auto-increments the register address.
 */
static void read_registers(uint8_t reg, uint8_t length, uint8_t *buf) {
    SPI.beginTransaction(_spi_settings);
    digitalWrite(LSM6DSOX_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);   // read flag
    for (uint8_t i = 0; i < length; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    digitalWrite(LSM6DSOX_CS_PIN, HIGH);
    SPI.endTransaction();
}

// --------------------------------------------------------
// PRIVATE — RAW CONVERSION
// --------------------------------------------------------

static inline int16_t to_int16(uint8_t low, uint8_t high) {
    return (int16_t)((uint16_t)(high << 8) | low);
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

bool lsm6dsox_init() {
    // CS pin setup and SPI.begin() must be called in setup() BEFORE this
    // function — see main.cpp and the note in lsm6dsox.h

    // 1. Software reset — identical to working sketch
    write_register(LSM6DSOX_REG_CTRL3_C, LSM6DSOX_SW_RESET);
    delay(50);

    // 2. WHO_AM_I
    uint8_t chip_id = read_register(LSM6DSOX_REG_WHO_AM_I);
    Serial.print("    LSM6DSOX WHO_AM_I: 0x"); Serial.println(chip_id, HEX);
    if (chip_id != LSM6DSOX_CHIP_ID) return false;

    // 3. CTRL3_C: BDU (0x40) + IF_INC (0x04) = 0x44
    write_register(LSM6DSOX_REG_CTRL3_C, LSM6DSOX_CTRL3_INIT);

    // 4. Accel: 833 Hz, ±16 g
    write_register(LSM6DSOX_REG_CTRL1_XL, LSM6DSOX_XL_833HZ_16G);

    // 5. Gyro: 833 Hz, ±2000 dps
    write_register(LSM6DSOX_REG_CTRL2_G, LSM6DSOX_G_833HZ_2000DPS);

    return true;
}

void lsm6dsox_read(LSM6DSOX_Data *out) {
    // Burst-read 12 bytes: gyro X/Y/Z then accel X/Y/Z, low byte first per axis
    uint8_t data[12];
    read_registers(LSM6DSOX_REG_OUTX_L_G, 12, data);

    out->gx = to_int16(data[0], data[1]) * LSM6DSOX_GYRO_SCALE;
    out->gy = to_int16(data[2], data[3]) * LSM6DSOX_GYRO_SCALE;
    out->gz = to_int16(data[4], data[5]) * LSM6DSOX_GYRO_SCALE;

    out->ax = to_int16(data[6],  data[7])  * LSM6DSOX_ACCEL_SCALE;
    out->ay = to_int16(data[8],  data[9])  * LSM6DSOX_ACCEL_SCALE;
    out->az = to_int16(data[10], data[11]) * LSM6DSOX_ACCEL_SCALE;

    out->valid = true;
}