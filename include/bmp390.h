#pragma once

#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------
// I2C ADDRESS
// --------------------------------------------------------
#define BMP390_ADDRESS       0x77   // SDO high; use 0x76 if SDO pulled low

// --------------------------------------------------------
// REGISTER MAP
// --------------------------------------------------------
#define BMP390_REG_CHIP_ID   0x00   // expected value: 0x60
#define BMP390_REG_ERR       0x02
#define BMP390_REG_STATUS    0x03

#define BMP390_REG_PRESS_0   0x04   // pressure XLSB
#define BMP390_REG_PRESS_1   0x05
#define BMP390_REG_PRESS_2   0x06   // pressure MSB
#define BMP390_REG_TEMP_0    0x07   // temperature XLSB
#define BMP390_REG_TEMP_1    0x08
#define BMP390_REG_TEMP_2    0x09   // temperature MSB

#define BMP390_REG_PWR_CTRL  0x1B   // enable pressure / temperature / mode
#define BMP390_REG_OSR       0x1C   // oversampling settings
#define BMP390_REG_ODR       0x1D   // output data rate

#define BMP390_REG_CALIB     0x31   // first calibration NVM register (21 bytes)

// PWR_CTRL bit masks
#define BMP390_PRESS_EN      (1 << 0)
#define BMP390_TEMP_EN       (1 << 1)
#define BMP390_MODE_NORMAL   (0x03 << 4)

// Expected chip ID
#define BMP390_CHIP_ID       0x60

// I2C fast-mode clock
#define BMP390_I2C_CLOCK     400000UL

// --------------------------------------------------------
// PIN ASSIGNMENTS (Teensy 4.0 default Wire bus)
// Change these if you move to Wire1 or Wire2
// --------------------------------------------------------
#define BMP390_PIN_SDA       18
#define BMP390_PIN_SCL       19

// --------------------------------------------------------
// STRUCTS
// --------------------------------------------------------

/**
 * Compensated floating-point calibration coefficients.
 * Populated once during init; treated as read-only afterward.
 */
typedef struct {
    // Temperature
    float t1, t2, t3;

    // Pressure
    float p1,  p2,  p3,  p4;
    float p5,  p6,  p7,  p8;
    float p9,  p10, p11;

    // Intermediate temperature value reused by pressure compensation.
    // Updated every time temperature is compensated.
    float t_lin;
} BMP390_Calib;

/**
 * Single measurement result.
 */
typedef struct {
    float temperature_c;  // degrees Celsius
    float pressure_pa;    // Pascals
    bool  valid;          // false if an I2C error occurred
} BMP390_Data;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

/**
 * Verify chip ID, load and convert calibration NVM, configure OSR and mode.
 * Call once in setup().
 *
 * @param cal  Output struct to populate with calibration coefficients.
 * @return true on success; false if the sensor is absent or I2C fails.
 */
bool bmp390_init(BMP390_Calib *cal);

/**
 * Read temperature and pressure in a single 6-byte burst and apply
 * datasheet compensation.  Temperature MUST be computed before pressure
 * (updates cal->t_lin), which this function enforces internally.
 *
 * @param cal  Calibration struct populated by bmp390_init().
 * @param out  Output measurement.  out->valid is false on any I2C error.
 */
void bmp390_read(BMP390_Calib *cal, BMP390_Data *out);