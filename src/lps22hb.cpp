#include <Wire.h>
#include <math.h>
#include "lps22hb.h"

// --------------------------------------------------------
// PRIVATE HELPERS — I2C
// --------------------------------------------------------

static bool write_register_lps(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(LPS22HB_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool read_registers_lps(uint8_t reg, uint8_t length, uint8_t *buf) {
    Wire.beginTransmission(LPS22HB_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t received = Wire.requestFrom((uint8_t)LPS22HB_ADDRESS, length);
    if (received != length) return false;

    for (uint8_t i = 0; i < length; i++) buf[i] = Wire.read();
    return true;
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

bool lps22hb_init(void) {
    Wire.setSDA(PIN_BARO_SDA);
    Wire.setSCL(PIN_BARO_SCL);
    Wire.begin();
    Wire.setClock(LPS22HB_I2C_CLOCK);

    uint8_t who_am_i = 0;
    if (!read_registers_lps(LPS22HB_REG_WHO_AM_I, 1, &who_am_i)) return false;
    if (who_am_i != LPS22HB_CHIP_ID) return false;

    // Enable register auto-increment for burst reads
    if (!write_register_lps(LPS22HB_REG_CTRL2, LPS22HB_CTRL2_INC)) return false;

    // 75 Hz continuous output, block data update on.
    // Max ODR for LPS22HB is 75 Hz; the control loop runs at 125 Hz so we must
    // gate each read on P_DA rather than polling blindly.
    if (!write_register_lps(LPS22HB_REG_CTRL1, LPS22HB_CTRL1_75HZ_BDU)) return false;

    return true;
}

void lps22hb_read(LPS22HB_Data *out) {
    // Gate on P_DA (pressure data available) in STATUS register.
    // The control loop runs at 125 Hz but the sensor only produces new data at 75 Hz.
    // Returning the previous stale value without this check would corrupt any
    // consecutive-decrease apogee detector.
    uint8_t status = 0;
    if (!read_registers_lps(LPS22HB_REG_STATUS, 1, &status) ||
        !(status & LPS22HB_STATUS_P_DA)) {
        out->valid         = false;
        out->pressure_pa   = NAN;
        out->temperature_c = NAN;
        return;
    }

    // Burst-read 5 bytes: PRESS_XL, PRESS_L, PRESS_H, TEMP_L, TEMP_H
    uint8_t data[5];
    if (!read_registers_lps(LPS22HB_REG_PRESS_XL, 5, data)) {
        out->valid         = false;
        out->pressure_pa   = NAN;
        out->temperature_c = NAN;
        return;
    }

    // 24-bit signed pressure, little-endian. Unit: 1/4096 hPa.
    int32_t raw_press = (int32_t)(((uint32_t)data[2] << 16)
                                | ((uint32_t)data[1] <<  8)
                                |  (uint32_t)data[0]);
    if (raw_press & 0x00800000) raw_press |= (int32_t)0xFF000000;

    // 16-bit signed temperature, little-endian. Unit: 1/100 °C.
    int16_t raw_temp = (int16_t)(((uint16_t)data[4] << 8) | data[3]);

    out->pressure_pa   = (raw_press / 4096.0f) * 100.0f;   // hPa → Pa
    out->temperature_c = raw_temp / 100.0f;
    out->valid         = true;
}
