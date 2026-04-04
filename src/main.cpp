#include <Arduino.h>
#include <Wire.h>
#include "bmp390.cpp" // Include your custom driver!

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  delay(100); 

  // You can now use the functions exactly as if they were written in this file
  uint8_t chip_id_buffer[1];
  read_registers(0x00, 1, chip_id_buffer); 

  Serial.print("BMP390 Chip ID: 0x");
  Serial.println(chip_id_buffer[0], HEX);
}

void loop() {
  // Main flight loop
}