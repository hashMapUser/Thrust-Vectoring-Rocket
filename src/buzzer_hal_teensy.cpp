#include <Arduino.h>
#include "buzzer_hal.h"

/* Hardware PWM via FlexPWM — zero CPU cost during tone, no interrupt overhead.
 * analogWriteResolution defaults to 8-bit; 128/255 ≈ 50% duty cycle. */

static void teensy_tone_on(uint8_t pin, uint16_t freq_hz) {
    analogWriteFrequency(pin, freq_hz);
    analogWrite(pin, 128);
}

static void teensy_tone_off(uint8_t pin) {
    analogWrite(pin, 0);
    digitalWrite(pin, LOW);
}

static uint32_t teensy_millis(void) {
    return millis();
}

extern "C" const buzzer_hal_t BUZZER_HAL_TEENSY = {
    .tone_on   = teensy_tone_on,
    .tone_off  = teensy_tone_off,
    .millis_fn = teensy_millis,
};
