/* buzzer.c — pattern state machine.
 * MUST NOT include Arduino.h. All platform calls go through buzzer_hal_t. */
#include "buzzer.h"
#include "buzzer_hal.h"

/* INVARIANT: do not #include <Arduino.h> here.
 * All platform calls go through buzzer_hal_t. */

/* ------------------------------------------------------------------ */
/* Pattern table                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    bool     on;
    uint16_t ms;   /* 0xFFFF = hold this step until pattern is changed */
} Step;

/* BOOT: one 80 ms chirp then hold silent */
static const Step PAT_BOOT[] = {
    { true,  80     },
    { false, 0xFFFF },
};

/* SELFTEST_PASS: two chirps then hold silent */
static const Step PAT_SELFTEST_PASS[] = {
    { true,  80     },
    { false, 80     },
    { true,  80     },
    { false, 0xFFFF },
};

/* SELFTEST_FAIL: three long tones then hold silent */
static const Step PAT_SELFTEST_FAIL[] = {
    { true,  400    },
    { false, 150    },
    { true,  400    },
    { false, 150    },
    { true,  400    },
    { false, 0xFFFF },
};

/* IDLE: one short chirp every 3 s */
static const Step PAT_IDLE[] = {
    { true,  50   },
    { false, 2950 },
};

/* ARMED: rapid double-chirp every 1 s — must be unmistakably distinct */
static const Step PAT_ARMED[] = {
    { true,  60  },
    { false, 60  },
    { true,  60  },
    { false, 820 },
};

/* LOCATOR: 1 Hz continuous — find-me after landing */
static const Step PAT_LOCATOR[] = {
    { true,  500 },
    { false, 500 },
};

/* Silent: off indefinitely */
static const Step PAT_SILENT[] = {
    { false, 0xFFFF },
};

typedef struct {
    const Step *steps;
    uint8_t     count;
} Pattern;

#define PAT(arr) { arr, (uint8_t)(sizeof(arr) / sizeof(arr[0])) }

static const Pattern PATTERNS[BUZZ_PATTERN_COUNT] = {
    PAT(PAT_SILENT),        /* BUZZ_SILENT        */
    PAT(PAT_BOOT),          /* BUZZ_BOOT          */
    PAT(PAT_SELFTEST_PASS), /* BUZZ_SELFTEST_PASS */
    PAT(PAT_SELFTEST_FAIL), /* BUZZ_SELFTEST_FAIL */
    PAT(PAT_IDLE),          /* BUZZ_IDLE          */
    PAT(PAT_ARMED),         /* BUZZ_ARMED         */
    PAT(PAT_LOCATOR),       /* BUZZ_LOCATOR       */
};

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

static void apply_output(buzzer_t *b, bool want_on) {
    if (want_on == b->output_on) return;   /* skip if already correct */
    b->output_on = want_on;
    if (want_on) {
        b->hal->tone_on(b->pin, b->freq_hz);
    } else {
        b->hal->tone_off(b->pin);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void buzzer_init(buzzer_t *b, const buzzer_hal_t *hal,
                 uint8_t pin, uint16_t freq_hz) {
    b->hal           = hal;
    b->pin           = pin;
    b->freq_hz       = freq_hz;
    b->pattern       = BUZZ_SILENT;
    b->step          = 0;
    b->step_start_ms = hal->millis_fn();
    b->output_on     = false;
    hal->tone_off(pin);
}

void buzzer_set(buzzer_t *b, buzzer_pattern_t p) {
    if (p == b->pattern) return;   /* idempotent — do not restart */
    b->pattern       = p;
    b->step          = 0;
    b->step_start_ms = b->hal->millis_fn();
    /* Apply first step output immediately so there is no perceptible lag. */
    const Step *s = &PATTERNS[p].steps[0];
    apply_output(b, s->on);
}

void buzzer_update(buzzer_t *b) {
    const Pattern *pat = &PATTERNS[b->pattern];
    const Step    *s   = &pat->steps[b->step];

    if (s->ms == 0xFFFF) return;   /* terminal step — hold until pattern changes */

    uint32_t now     = b->hal->millis_fn();
    uint32_t elapsed = now - b->step_start_ms;   /* unsigned wrap-safe */

    if (elapsed < s->ms) return;   /* still in this step */

    /* Advance to next step (wrap to 0 at end). */
    b->step = (b->step + 1) % pat->count;
    b->step_start_ms = now;
    s = &pat->steps[b->step];
    apply_output(b, s->on);
}

void buzzer_off(buzzer_t *b) {
    b->pattern   = BUZZ_SILENT;
    b->step      = 0;
    apply_output(b, false);
}
