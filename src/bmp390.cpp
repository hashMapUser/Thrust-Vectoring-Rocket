#include <Wire.h>
#include <math.h>
#include "bmp390.h"

// --------------------------------------------------------
// PRIVATE HELPERS — I2C
// --------------------------------------------------------

/**
 * Write a single byte to a register.
 * Returns true on success.
 */
static bool write_register(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BMP390_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

/**
 * Burst-read `length` bytes starting at `reg` into `buf`.
 * Returns true on success.
 */
static bool read_registers(uint8_t reg, uint8_t length, uint8_t *buf) {
    Wire.beginTransmission(BMP390_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t received = Wire.requestFrom((uint8_t)BMP390_ADDRESS, length);
    if (received != length) return false;

    for (uint8_t i = 0; i < length; i++) {
        buf[i] = Wire.read();
    }

    return true;
}

// --------------------------------------------------------
// PRIVATE HELPERS — CALIBRATION
// --------------------------------------------------------

/**
 * Parse the 21-byte NVM calibration dump into the BMP390_Calib struct,
 * applying the datasheet fixed-point → float scaling factors.
 */
static void parse_calibration(const uint8_t *raw, BMP390_Calib *cal) {
    // --- Temperature ---
    uint16_t par_t1 = (uint16_t)((raw[1] << 8) | raw[0]);
    uint16_t par_t2 = (uint16_t)((raw[3] << 8) | raw[2]);
    int8_t   par_t3 = (int8_t)raw[4];

    cal->t1 = (float)par_t1 * 2.56e2f;          // * 2^8  (Bosch ref: NVM / 2^-8)
    cal->t2 = (float)par_t2 / 1.073741824e9f;   // / 2^30
    cal->t3 = (float)par_t3 / 2.81474977e14f;   // / 2^48

    // --- Pressure ---
    int16_t  par_p1  = (int16_t) ((raw[6]  << 8) | raw[5]);
    int16_t  par_p2  = (int16_t) ((raw[8]  << 8) | raw[7]);
    int8_t   par_p3  = (int8_t)   raw[9];
    int8_t   par_p4  = (int8_t)   raw[10];
    uint16_t par_p5  = (uint16_t)((raw[12] << 8) | raw[11]);
    uint16_t par_p6  = (uint16_t)((raw[14] << 8) | raw[13]);
    int8_t   par_p7  = (int8_t)   raw[15];
    int8_t   par_p8  = (int8_t)   raw[16];
    int16_t  par_p9  = (int16_t) ((raw[18] << 8) | raw[17]);
    int8_t   par_p10 = (int8_t)   raw[19];
    int8_t   par_p11 = (int8_t)   raw[20];

    cal->p1  = ((float)par_p1  - 16384.0f) / 1.048576e6f;    // (x - 2^14) / 2^20
    cal->p2  = ((float)par_p2  - 16384.0f) / 5.36870912e8f;  // (x - 2^14) / 2^29
    cal->p3  = (float)par_p3  / 4.294967296e9f;              // / 2^32
    cal->p4  = (float)par_p4  / 1.37438953e11f;              // / 2^37
    cal->p5  = (float)par_p5  * 8.0f;                        // / 2^-3  → * 8
    cal->p6  = (float)par_p6  / 64.0f;                       // / 2^6
    cal->p7  = (float)par_p7  / 256.0f;                      // / 2^8
    cal->p8  = (float)par_p8  / 32768.0f;                    // / 2^15
    cal->p9  = (float)par_p9  / 2.81474977e14f;              // / 2^48
    cal->p10 = (float)par_p10 / 2.81474977e14f;              // / 2^48
    cal->p11 = (float)par_p11 / 3.68934882e19f;              // / 2^65

    cal->t_lin = 0.0f;
}

// --------------------------------------------------------
// PRIVATE HELPERS — COMPENSATION MATH
// --------------------------------------------------------

/**
 * Apply datasheet temperature compensation formula.
 * Stores t_lin in cal for subsequent pressure compensation.
 */
static float compensate_temperature(uint32_t raw, BMP390_Calib *cal) {
    float pd1   = (float)raw - cal->t1;
    float pd2   = pd1 * cal->t2;
    cal->t_lin  = pd2 + (pd1 * pd1) * cal->t3;
    return cal->t_lin;
}

/**
 * Apply datasheet pressure compensation formula.
 * Requires cal->t_lin to be current — call compensate_temperature first.
 */
static float compensate_pressure(uint32_t raw, const BMP390_Calib *cal) {
    float tl = cal->t_lin;

    // Offset term
    float out1 = cal->p5
                 + cal->p6  * tl
                 + cal->p7  * tl * tl
                 + cal->p8  * tl * tl * tl;

    // Sensitivity term
    float out2 = (float)raw * (
                     cal->p1
                     + cal->p2  * tl
                     + cal->p3  * tl * tl
                     + cal->p4  * tl * tl * tl
                 );

    // Higher-order correction
    float p2   = (float)raw * (float)raw;
    float pd3  = p2 * (cal->p9 + cal->p10 * tl);
    float pd4  = p2 * (float)raw * cal->p11;

    return out1 + out2 + pd3 + pd4;
}

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

bool bmp390_init(BMP390_Calib *cal) {
    // Set pins explicitly so the wiring is defined in one place (bmp390.h)
    // rather than relying on Wire defaults
    Wire.setSDA(BMP390_PIN_SDA);
    Wire.setSCL(BMP390_PIN_SCL);
    Wire.begin();
    Wire.setClock(BMP390_I2C_CLOCK);

    // 1. Verify chip ID
    uint8_t chip_id = 0;
    if (!read_registers(BMP390_REG_CHIP_ID, 1, &chip_id)) return false;
    if (chip_id != BMP390_CHIP_ID) return false;

    // 2. Load calibration NVM (21 bytes starting at 0x31)
    uint8_t calib_raw[21];
    if (!read_registers(BMP390_REG_CALIB, 21, calib_raw)) return false;
    parse_calibration(calib_raw, cal);

    // 3. Configure oversampling — OSR = 1x for both (fastest, change as needed)
    if (!write_register(BMP390_REG_OSR, 0x00)) return false;

    // 4. Enable pressure + temperature in normal (continuous) mode
    uint8_t pwr = BMP390_PRESS_EN | BMP390_TEMP_EN | BMP390_MODE_NORMAL;
    if (!write_register(BMP390_REG_PWR_CTRL, pwr)) return false;

    return true;
}

void bmp390_read(BMP390_Calib *cal, BMP390_Data *out) {
    // Burst-read 6 bytes: pressure (0x04–0x06) then temperature (0x07–0x09)
    uint8_t data[6];
    if (!read_registers(BMP390_REG_PRESS_0, 6, data)) {
        out->valid         = false;
        out->pressure_pa   = NAN;
        out->temperature_c = NAN;
        return;
    }

    uint32_t raw_press = ((uint32_t)data[2] << 16)
                       | ((uint32_t)data[1] <<  8)
                       |  (uint32_t)data[0];

    uint32_t raw_temp  = ((uint32_t)data[5] << 16)
                       | ((uint32_t)data[4] <<  8)
                       |  (uint32_t)data[3];

    // Temperature MUST be compensated first — it updates cal->t_lin
    out->temperature_c = compensate_temperature(raw_temp, cal);
    out->pressure_pa   = compensate_pressure(raw_press, cal);
    out->valid         = true;
}