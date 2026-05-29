#include <Arduino.h>
#include "indicator.h"

// Beep patterns per state — encoded as arrays of durations [ms]
// Positive = ON, negative = OFF, 0 = end of pattern
//
// IDLE:    single slow pulse every 1.5s   (I'm alive)
// ARMED:   double fast beep               (armed — stand clear)
// POWERED: triple fast beep               (motor burning)
// COAST:   silent
// APOGEE:  long single beep               (apogee reached)
// DESCENT: four short beeps               (descending)
// MAIN:    continuous fast beep           (main deployed — find me)
// LANDED:  SOS-ish pattern               (find me on the ground)
// ABORT:   rapid continuous beep         (fault)

struct Pattern {
    const int16_t *steps;
    uint8_t        count;
};

static const int16_t PAT_IDLE[]    = { BEEP_SHORT, -PATTERN_GAP };
static const int16_t PAT_ARMED[]   = { BEEP_SHORT, -BEEP_GAP, BEEP_SHORT, -PATTERN_GAP };
static const int16_t PAT_POWERED[] = { BEEP_SHORT, -BEEP_GAP, BEEP_SHORT, -BEEP_GAP, BEEP_SHORT, -PATTERN_GAP };
static const int16_t PAT_COAST[]   = { -(int16_t)PATTERN_GAP };
static const int16_t PAT_APOGEE[]  = { BEEP_LONG, -PATTERN_GAP };
static const int16_t PAT_DESCENT[] = { BEEP_SHORT, -BEEP_GAP, BEEP_SHORT, -BEEP_GAP,
                                        BEEP_SHORT, -BEEP_GAP, BEEP_SHORT, -PATTERN_GAP };
static const int16_t PAT_MAIN[]    = { BEEP_SHORT, -BEEP_GAP };
static const int16_t PAT_LANDED[]  = { BEEP_LONG, -BEEP_GAP, BEEP_LONG, -BEEP_GAP,
                                        BEEP_LONG, -PATTERN_GAP };
static const int16_t PAT_ABORT[]   = { BEEP_SHORT, -(int16_t)BEEP_GAP/2 };

#define PAT(arr) { arr, sizeof(arr)/sizeof(arr[0]) }

static const Pattern PATTERNS[] = {
    PAT(PAT_IDLE),     // STATE_IDLE
    PAT(PAT_ARMED),    // STATE_ARMED
    PAT(PAT_POWERED),  // STATE_POWERED
    PAT(PAT_COAST),    // STATE_COAST
    PAT(PAT_APOGEE),   // STATE_APOGEE
    PAT(PAT_DESCENT),  // STATE_DESCENT
    PAT(PAT_MAIN),     // STATE_MAIN
    PAT(PAT_LANDED),   // STATE_LANDED
    PAT(PAT_ABORT),    // STATE_ABORT
};

void indicator_init(IndicatorState *ind) {
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN,    OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN,    LOW);
    ind->last_update_ms = millis();
    ind->step           = 0;
    ind->buzzer_on      = false;
    ind->led_on         = false;
}

void indicator_update(IndicatorState *ind, FlightState state) {
    if (state >= 9) return;  // bounds check

    const Pattern &pat  = PATTERNS[(int)state];
    uint32_t now        = millis();
    uint32_t elapsed    = now - ind->last_update_ms;

    int16_t step_val = pat.steps[ind->step % pat.count];
    uint32_t duration = (uint32_t)abs(step_val);

    if (elapsed >= duration) {
        ind->last_update_ms = now;
        ind->step           = (ind->step + 1) % pat.count;

        // Next step
        bool active = pat.steps[ind->step % pat.count] > 0;
        ind->buzzer_on = active;
        ind->led_on    = active;

        digitalWrite(BUZZER_PIN, active ? HIGH : LOW);
        digitalWrite(LED_PIN,    active ? HIGH : LOW);
    }
}