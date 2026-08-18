#pragma once
/* NO Arduino.h — this header is pure C99. */
#include <stdint.h>
#include <stdbool.h>
#include "buzzer_hal.h"

typedef enum {
    BUZZ_SILENT = 0,
    BUZZ_BOOT,           /* one chirp — firmware reached setup()         */
    BUZZ_SELFTEST_PASS,  /* two chirps — all sensors enumerated          */
    BUZZ_SELFTEST_FAIL,  /* three long tones — a sensor failed init      */
    BUZZ_IDLE,           /* one chirp every 3 s — alive, disarmed        */
    BUZZ_ARMED,          /* rapid double chirp every 1 s — ORDNANCE LIVE */
    BUZZ_LOCATOR,        /* 1 Hz continuous — post-landing recovery      */
    BUZZ_PATTERN_COUNT
} buzzer_pattern_t;

typedef struct {
    const buzzer_hal_t *hal;
    uint8_t             pin;
    uint16_t            freq_hz;
    buzzer_pattern_t    pattern;
    uint32_t            step_start_ms;
    uint8_t             step;
    bool                output_on;
} buzzer_t;

#ifdef __cplusplus
extern "C" {
#endif

void buzzer_init  (buzzer_t *b, const buzzer_hal_t *hal,
                   uint8_t pin, uint16_t freq_hz);
void buzzer_set   (buzzer_t *b, buzzer_pattern_t p);
void buzzer_update(buzzer_t *b);   /* call every loop iteration — never blocks */
void buzzer_off   (buzzer_t *b);

#ifdef __cplusplus
}
#endif
