#include <Arduino.h>
#include <Servo.h>
#include "servo_driver.h"

static Servo _pitch_servo;
static Servo _yaw_servo;
static float _pitch_us = SERVO_PITCH_CENTER_US;
static float _yaw_us   = SERVO_YAW_CENTER_US;
static bool  _enabled  = false;

// Map angle to pulse width, applying invert flag and hard-clamping to
// the measured mechanical limits before writeMicroseconds().
static float angle_to_us(float angle_deg, bool invert, float center_us,
                          float min_us, float max_us, float max_angle_deg) {
    float a = invert ? -angle_deg : angle_deg;
    float clamped = constrain(a, -max_angle_deg, max_angle_deg);
    float scale   = clamped / max_angle_deg;   // -1 to +1
    // Span is asymmetric-safe: scales toward max_us on positive, min_us on negative.
    float us = center_us + scale * (float)(scale >= 0 ? (max_us - center_us) : (center_us - min_us));
    // Hard-clamp to mechanical limits — never command past a hard stop.
    return constrain(us, min_us, max_us);
}

void servo_init() {
    _pitch_servo.attach(PIN_SERVO_X, SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US);
    _yaw_servo.attach(PIN_SERVO_Y,   SERVO_YAW_MIN_US,   SERVO_YAW_MAX_US);

    _enabled = true;
    servo_center();

    Serial.print("[SERVO] Initialized on pins ");
    Serial.print(PIN_SERVO_X);
    Serial.print(" (X/pitch) and ");
    Serial.print(PIN_SERVO_Y);
    Serial.println(" (Y/yaw)");
}

void servo_set_pitch(float angle_deg) {
    if (!_enabled) return;
    _pitch_us = angle_to_us(angle_deg, (bool)SERVO_X_INVERT, SERVO_PITCH_CENTER_US,
                             SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US, SERVO_PITCH_MAX_ANGLE_DEG);
    _pitch_servo.writeMicroseconds((int)_pitch_us);
}

void servo_set_yaw(float angle_deg) {
    if (!_enabled) return;
    _yaw_us = angle_to_us(angle_deg, (bool)SERVO_Y_INVERT, SERVO_YAW_CENTER_US,
                           SERVO_YAW_MIN_US, SERVO_YAW_MAX_US, SERVO_YAW_MAX_ANGLE_DEG);
    _yaw_servo.writeMicroseconds((int)_yaw_us);
}

void servo_center() {
    if (!_enabled) return;
    _pitch_us = SERVO_PITCH_CENTER_US;
    _yaw_us   = SERVO_YAW_CENTER_US;
    _pitch_servo.writeMicroseconds(SERVO_PITCH_CENTER_US);
    _yaw_servo.writeMicroseconds(SERVO_YAW_CENTER_US);
}

void servo_disable() {
    _pitch_servo.detach();
    _yaw_servo.detach();
    _enabled = false;
}

float servo_get_pitch_us() { return _pitch_us; }
float servo_get_yaw_us()   { return _yaw_us; }
