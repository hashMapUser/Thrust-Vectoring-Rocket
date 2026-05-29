#pragma once

#include <Arduino.h>
#include "flight_sm.h"

// --------------------------------------------------------
// CONFIG
// --------------------------------------------------------

#define BUZZER_PIN    7    // piezo buzzer — active low or high depending on module
#define LED_PIN       8    // status LED

// Beep pattern timing [ms]
#define BEEP_SHORT    80
#define BEEP_LONG     400
#define BEEP_GAP      120
#define PATTERN_GAP   1500  // pause between pattern repeats

// --------------------------------------------------------
// INDICATOR STATE
// --------------------------------------------------------

typedef struct {
    uint32_t last_update_ms;
    uint8_t  step;
    bool     buzzer_on;
    bool     led_on;
} IndicatorState;

// --------------------------------------------------------
// PUBLIC API
// --------------------------------------------------------

void indicator_init(IndicatorState *ind);

/**
 * Update buzzer and LED pattern based on current flight state.
 * Non-blocking — call every loop iteration.
 */
void indicator_update(IndicatorState *ind, FlightState state);