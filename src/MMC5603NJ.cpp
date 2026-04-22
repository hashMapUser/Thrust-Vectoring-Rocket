#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "mmc5603nj.h"

// --------------------------------------------------------
// PRIVATE — I2C HELPERS
// --------------------------------------------------------

static bool write_register(uint8_t reg, uint8_t value) {
    Wire1.beginTransmission(MMC5603NJ_ADDRESS);
    Wire1.write(reg);
    Wire1.write(value);
    return Wire1.endTransmission() == 0;
}

static bool read_register(uint8_t reg, uint8_t *value) {
    Wire1.beginTransmission(MMC5603NJ_ADDRESS);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((uint8_t)MMC5603NJ_ADDRESS, (uint8_t)1) != 1) return false;
    *value = Wire1.read();
    return true;
}

/**
 * Burst-read `length` bytes starting at `reg` into `buf`.
 * MMC5603NJ auto-increments the register address on I2C — no extra config needed.
 */
static bool read_registers(uint8_t reg, uint8_t length, uint8_t *buf) {
    Wire1.beginTransmission(MMC5603NJ_ADDRESS);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;

    uint8_t received = Wire1.requestFrom((uint8_t)MMC5603NJ_ADDRESS, length);
    if (received != length) return false;

    for (uint8_t i = 0; i < length; i++) {
        buf[i] = Wire1.read();
    }
    return true;
}

// --------------------------------------------------------
// PRIVATE — 18-BIT ASSEMBLY
// --------------------------------------------------------

/**
 * Assemble one 18-bit axis reading from three raw bytes.
 * Output is offset binary — zero field = 2^17 = 131072.
 * Subtract MMC5603NJ_ZERO_OFFSET before scaling to get a signed value.
 */
static float assemble_axis(uint8_t out0, uint8_t out1, uint8_t out2) {
    uint32_t raw = ((uint32_t)out0 << 12)
                 | ((uint32_t)out1 <<  4)
                 | ((uint32_t)out2 >>  4);

    int32_t signed_raw = (int32_t)raw - MMC5603NJ_ZERO_OFFSET;
    return (float)signed_raw * MMC5603NJ_SCALE;
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

bool mmc5603nj_init() {
    // Explicitly set pins and clock in case this runs before bmp390_init().
    // Calling Wire1.begin() more than once is safe on Teensy.
    Wire1.setSDA(MMC5603NJ_PIN_SDA);
    Wire1.setSCL(MMC5603NJ_PIN_SCL);
    Wire1.begin();
    Wire1.setClock(MMC5603NJ_I2C_CLOCK);    

    // 1. Verify chip ID — print raw value so a mismatch is diagnosable
    uint8_t chip_id = 0;
    if (!read_register(MMC5603NJ_REG_PROD_ID, &chip_id)) {
        Serial.println("    MMC5603NJ: I2C read failed — check wiring");
        return false;
    }
    Serial.print("    MMC5603NJ PROD_ID: 0x"); Serial.println(chip_id, HEX);
    if (chip_id != MMC5603NJ_CHIP_ID) return false;

    // 2. Fire SET coil — removes residual magnetization datasheet
    //    recommends running SET once at startup before the first measurement.
    if (!write_register(MMC5603NJ_REG_CTRL0, MMC5603NJ_SET_COIL)) return false;
    delay(1);   // SET pulse completes in < 1 ms

    return true;
}

void mmc5603nj_read(MMC5603NJ_Data *out) {
    // 1. Trigger a single measurement
    if (!write_register(MMC5603NJ_REG_CTRL0, MMC5603NJ_TM_M)) {
        out->valid = false;
        out->mag_x = out->mag_y = out->mag_z = NAN;
        return;
    }

    // 2. Poll STATUS1 until Meas_M_Done (bit 6) is set.
    //    Typical measurement time is ~8 ms at default bandwidth.
    //    Bail out after MMC5603NJ_MEAS_TIMEOUT_MS to avoid blocking forever.
    uint32_t start = millis();
    uint8_t  status = 0;

    while (true) {
        if (!read_register(MMC5603NJ_REG_STATUS1, &status)) {
            out->valid = false;
            out->mag_x = out->mag_y = out->mag_z = NAN;
            return;
        }
        if (status & MMC5603NJ_MEAS_M_DONE) break;

        if ((millis() - start) > MMC5603NJ_MEAS_TIMEOUT_MS) {
            out->valid = false;
            out->mag_x = out->mag_y = out->mag_z = NAN;
            return;
        }
        delay(1);
    }

    // 3. Burst-read all 9 output bytes starting at XOUT0 (0x00)
    //    Layout:
    //      data[0] = XOUT0   data[1] = XOUT1   data[2] = YOUT0
    //      data[3] = YOUT1   data[4] = ZOUT0   data[5] = ZOUT1
    //      data[6] = XOUT2   data[7] = YOUT2   data[8] = ZOUT2
    uint8_t data[9];
    if (!read_registers(MMC5603NJ_REG_XOUT0, 9, data)) {
        out->valid = false;
        out->mag_x = out->mag_y = out->mag_z = NAN;
        return;
    }

    // 4. Assemble 18-bit values and scale to Gauss
    out->mag_x = assemble_axis(data[0], data[1], data[6]);
    out->mag_y = assemble_axis(data[2], data[3], data[7]);
    out->mag_z = assemble_axis(data[4], data[5], data[8]);
    out->valid = true;
}