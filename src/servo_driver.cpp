#include <Arduino.h>
#include "servo_driver.h"

// Servo pulses come from the FlexPWM hardware, not the Servo library. On
// Teensy 4 that library times every pulse edge from an IntervalTimer
// interrupt, so anything that delays interrupts — USB serial traffic,
// EEPROM writes — stretches a pulse and the servo twitches. Hardware PWM
// holds the width exactly, whatever the CPU is doing.
//
// Pin 5 is FlexPWM2 submodule 1 and pin 6 is FlexPWM2 submodule 2: each
// submodule has its own frequency, so 50 Hz here doesn't touch the buzzer
// (pin 3, FlexPWM4) or the LEDs on submodule 0 (pins 4 and 33, plain GPIO).
#define SERVO_PWM_HZ      50
#define SERVO_PERIOD_US   (1000000.0f / SERVO_PWM_HZ)
#define SERVO_PWM_BITS    15        // 20 ms / 32768 = 0.61 us per step
#define DEFAULT_PWM_BITS  8         // analogWrite's default, which buzzer_hal_teensy.cpp assumes

static float _pitch_us = SERVO_PITCH_CENTER_US;
static float _yaw_us   = SERVO_YAW_CENTER_US;
static bool  _enabled  = false;

// One pulse width, clamped to the servo's measured stops. Resolution is a
// global setting in the Teensy core, so it goes back to 8 bits afterwards.
static void pwm_write_us(uint8_t pin, float us, float min_us, float max_us) {
    us = constrain(us, min_us, max_us);
    uint32_t duty = (uint32_t)(us * (float)(1u << SERVO_PWM_BITS) / SERVO_PERIOD_US + 0.5f);
    analogWriteResolution(SERVO_PWM_BITS);
    analogWrite(pin, duty);
    analogWriteResolution(DEFAULT_PWM_BITS);
}

// Nozzle angle [deg] to pulse width, applying the invert flag and clamping
// to the axis's travel so it never commands past a limit.
static float angle_to_us(float angle_deg, bool invert, float center_us, float span_us,
                         float us_per_deg) {
    const float a = invert ? -angle_deg : angle_deg;
    return constrain(center_us + a * us_per_deg, center_us - span_us, center_us + span_us);
}

void servo_init() {
    analogWriteFrequency(SERVO_PITCH_PIN, SERVO_PWM_HZ);
    analogWriteFrequency(SERVO_YAW_PIN,   SERVO_PWM_HZ);

    _enabled = true;
    servo_center();

    Serial.print("[SERVO] Initialized (hardware PWM): pitch on pin ");
    Serial.print(SERVO_PITCH_PIN);
    Serial.print(", yaw on pin ");
    Serial.println(SERVO_YAW_PIN);
}

void servo_set_pitch(float angle_deg) {
    if (!_enabled) return;
    _pitch_us = angle_to_us(angle_deg, (bool)SERVO_PITCH_INVERT, SERVO_PITCH_CENTER_US,
                            SERVO_PITCH_SPAN_US, SERVO_PITCH_US_PER_DEG);
    pwm_write_us(SERVO_PITCH_PIN, _pitch_us, SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US);
}

void servo_set_yaw(float angle_deg) {
    if (!_enabled) return;
    _yaw_us = angle_to_us(angle_deg, (bool)SERVO_YAW_INVERT, SERVO_YAW_CENTER_US,
                          SERVO_YAW_SPAN_US, SERVO_YAW_US_PER_DEG);
    pwm_write_us(SERVO_YAW_PIN, _yaw_us, SERVO_YAW_MIN_US, SERVO_YAW_MAX_US);
}

void servo_center() {
    if (!_enabled) return;
    _pitch_us = SERVO_PITCH_CENTER_US;
    _yaw_us   = SERVO_YAW_CENTER_US;
    pwm_write_us(SERVO_PITCH_PIN, _pitch_us, SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US);
    pwm_write_us(SERVO_YAW_PIN,   _yaw_us,   SERVO_YAW_MIN_US,   SERVO_YAW_MAX_US);
}

// Stop the pulses (output held low), like Servo::detach() — the servos go limp.
void servo_disable() {
    analogWrite(SERVO_PITCH_PIN, 0);
    analogWrite(SERVO_YAW_PIN, 0);
    _enabled = false;
}

float servo_get_pitch_us() { return _pitch_us; }
float servo_get_yaw_us()   { return _yaw_us; }
