#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "lsm6dsox.h"

// --------------------------------------------------------
// PRIVATE — SPI HELPERS
// --------------------------------------------------------

/**
 * Write a single byte to a register over SPI.
 *
 * Frame: [addr | WRITE] [value]
 */
static void write_register(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(LSM6DSOX_SPI_SETTINGS);
    digitalWrite(LSM6DSOX_CS_PIN, LOW);

    SPI.transfer(reg | LSM6DSOX_SPI_WRITE);  // bit 7 = 0 → write
    SPI.transfer(value);

    digitalWrite(LSM6DSOX_CS_PIN, HIGH);
    SPI.endTransaction();
}

/**
 * Read a single register byte over SPI.
 *
 * Frame: [addr | READ] [dummy → returns value]
 */
static uint8_t read_register(uint8_t reg) {
    SPI.beginTransaction(LSM6DSOX_SPI_SETTINGS);
    digitalWrite(LSM6DSOX_CS_PIN, LOW);

    SPI.transfer(reg | LSM6DSOX_SPI_READ);   // bit 7 = 1 → read
    uint8_t value = SPI.transfer(0x00);       // clock out dummy to receive data

    digitalWrite(LSM6DSOX_CS_PIN, HIGH);
    SPI.endTransaction();

    return value;
}

/**
 * Burst-read `length` bytes starting at `reg` into `buf`.
 *
 * Frame: [addr | READ] [dummy × length]
 *
 * Auto-increment is handled by the sensor (IF_INC=1 set in init),
 * so the register address advances automatically on each byte clocked out.
 */
static void read_registers(uint8_t reg, uint8_t length, uint8_t *buf) {
    SPI.beginTransaction(LSM6DSOX_SPI_SETTINGS);
    digitalWrite(LSM6DSOX_CS_PIN, LOW);

    SPI.transfer(reg | LSM6DSOX_SPI_READ);   // address phase

    for (uint8_t i = 0; i < length; i++) {
        buf[i] = SPI.transfer(0x00);          // data phase — dummy out, data in
    }

    digitalWrite(LSM6DSOX_CS_PIN, HIGH);
    SPI.endTransaction();
}

// --------------------------------------------------------
// PRIVATE — RAW CONVERSION
// --------------------------------------------------------

/**
 * Combine two bytes (little-endian, low byte first) into a signed 16-bit int.
 */
static inline int16_t to_int16(uint8_t low, uint8_t high) {
    return (int16_t)((uint16_t)(high << 8) | low);
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

bool lsm6dsox_init() {
    // NOTE: pinMode(LSM6DSOX_CS_PIN, OUTPUT) + digitalWrite(HIGH) must have
    // been called BEFORE SPI.begin() in setup(). See header for explanation.

    // 1. Software reset — clears all registers to default state.
    //    After reset CTRL3_C returns to 0x00, so we write SW_RESET alone first.
    write_register(LSM6DSOX_REG_CTRL3_C, LSM6DSOX_SW_RESET);
    delay(50);   // datasheet: boot time after reset is max 10 ms; 50 ms is safe

    // 2. Verify chip ID
    uint8_t chip_id = read_register(LSM6DSOX_REG_WHO_AM_I);
    Serial.print("    LSM6DSOX WHO_AM_I: 0x"); Serial.println(chip_id, HEX);
    if (chip_id != LSM6DSOX_CHIP_ID) return false;

    // 3. CTRL3_C: enable BDU + IF_INC
    //    BDU  — output registers only update after both bytes read (no half-updates)
    //    IF_INC — auto-increment register address during burst reads
    write_register(LSM6DSOX_REG_CTRL3_C, LSM6DSOX_CTRL3_INIT);

    // 4. Accelerometer: 833 Hz, ±16 g
    write_register(LSM6DSOX_REG_CTRL1_XL, LSM6DSOX_XL_833HZ_16G);

    // 5. Gyroscope: 833 Hz, ±2000 dps
    write_register(LSM6DSOX_REG_CTRL2_G, LSM6DSOX_G_833HZ_2000DPS);

    return true;
}

void lsm6dsox_read(LSM6DSOX_Data *out) {
    // Burst-read 12 bytes starting at OUTX_L_G (0x22):
    //   data[0..5]  = gyro  X/Y/Z  (low byte, high byte per axis)
    //   data[6..11] = accel X/Y/Z  (low byte, high byte per axis)
    uint8_t data[12];
    read_registers(LSM6DSOX_REG_OUTX_L_G, 12, data);

    // --- Gyro [deg/s] ---
    out->gx = to_int16(data[0], data[1]) * LSM6DSOX_GYRO_SCALE;
    out->gy = to_int16(data[2], data[3]) * LSM6DSOX_GYRO_SCALE;
    out->gz = to_int16(data[4], data[5]) * LSM6DSOX_GYRO_SCALE;

    // --- Accel [g] ---
    out->ax = to_int16(data[6],  data[7])  * LSM6DSOX_ACCEL_SCALE;
    out->ay = to_int16(data[8],  data[9])  * LSM6DSOX_ACCEL_SCALE;
    out->az = to_int16(data[10], data[11]) * LSM6DSOX_ACCEL_SCALE;

    out->valid = true;
}