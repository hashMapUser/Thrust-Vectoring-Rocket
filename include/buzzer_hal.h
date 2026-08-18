#pragma once
#include <stdint.h>

/* Platform abstraction for the buzzer driver.
 * buzzer.c calls only these three functions — no Arduino.h in the driver. */
typedef struct {
    /* Start a free-running ~50% duty square wave at freq_hz on pin.
     * Must be hardware-generated — no per-edge CPU involvement. */
    void     (*tone_on) (uint8_t pin, uint16_t freq_hz);

    /* Stop output and leave the pin driven LOW. */
    void     (*tone_off)(uint8_t pin);

    /* Monotonic milliseconds since boot. */
    uint32_t (*millis_fn)(void);
} buzzer_hal_t;
