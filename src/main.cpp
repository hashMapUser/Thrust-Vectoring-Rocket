#include <Arduino.h>
#include <SPI.h>

#define CS_PIN 10

// Try both modes — LSM6DSOX supports Mode 0 and Mode 3
static SPISettings spi_mode0(500000, MSBFIRST, SPI_MODE0);
static SPISettings spi_mode3(500000, MSBFIRST, SPI_MODE3);

uint8_t raw_read(uint8_t reg, SPISettings &cfg) {
    SPI.beginTransaction(cfg);
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(2);          // let CS settle before first clock edge
    SPI.transfer(reg | 0x80);      // bit7=1 → read
    uint8_t val = SPI.transfer(0x00);
    delayMicroseconds(2);
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    return val;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    delay(200);                    // let sensor power up fully
    SPI.begin();
    delay(100);

    Serial.println("=== Raw SPI diagnostic (no driver) ===");
    Serial.println("WHO_AM_I (0x0F) expected: 0x6C");
    Serial.println("CTRL3_C  (0x12) expected: 0x04 (IF_INC default)");
}

void loop() {
    uint8_t id_m0  = raw_read(0x0F, spi_mode0);
    uint8_t id_m3  = raw_read(0x0F, spi_mode3);
    uint8_t ctrl_m0 = raw_read(0x12, spi_mode0);
    uint8_t ctrl_m3 = raw_read(0x12, spi_mode3);

    Serial.print("Mode0 — WHO_AM_I=0x"); Serial.print(id_m0, HEX);
    Serial.print("  CTRL3_C=0x");        Serial.println(ctrl_m0, HEX);

    Serial.print("Mode3 — WHO_AM_I=0x"); Serial.print(id_m3, HEX);
    Serial.print("  CTRL3_C=0x");        Serial.println(ctrl_m3, HEX);

    Serial.println("---");
    delay(1000);
}
