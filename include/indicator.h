#pragma once
/* Buzzer patterns moved to buzzer.h / buzzer.c (T13).
 * This header is retained as a stub; indicator_update() is a no-op. */
#include "flight_sm.h"

typedef struct { uint8_t _unused; } IndicatorState;

#ifdef __cplusplus
extern "C" {
#endif
void indicator_init  (IndicatorState *ind);
void indicator_update(IndicatorState *ind, FlightState state);
#ifdef __cplusplus
}
#endif
