#include <Arduino.h>
#include <Servo.h>
#include "servo_driver.h"

static Servo _pitch_servo;
static Servo _yaw_servo;
static float _pitch_us = SERVO_CENTER_US;
static float _yaw_us   = SERVO_CENTER_US;
static bool  _enabled  = false;

// Map angle to pulse width, applying invert flag and hard-clamping to
// the measured mechanical limits before writeMicroseconds().
static float angle_to_us(float angle_deg, bool invert) {
    float a = invert ? -angle_deg : angle_deg;
    float clamped = constrain(a, -SERVO_MAX_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
    float scale   = clamped / SERVO_MAX_ANGLE_DEG;   // -1 to +1
    float us      = SERVO_CENTER_US + scale * (float)(SERVO_MAX_US - SERVO_CENTER_US);
    // Hard-clamp to mechanical limits — never command past a hard stop.
    return constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
}

void servo_init() {
    _pitch_servo.attach(PIN_SERVO_X, SERVO_MIN_US, SERVO_MAX_US);
    _yaw_servo.attach(PIN_SERVO_Y,   SERVO_MIN_US, SERVO_MAX_US);

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
    _pitch_us = angle_to_us(angle_deg, (bool)SERVO_X_INVERT);
    _pitch_servo.writeMicroseconds((int)_pitch_us);
}

void servo_set_yaw(float angle_deg) {
    if (!_enabled) return;
    _yaw_us = angle_to_us(angle_deg, (bool)SERVO_Y_INVERT);
    _yaw_servo.writeMicroseconds((int)_yaw_us);
}

void servo_center() {
    if (!_enabled) return;
    _pitch_us = SERVO_CENTER_US;
    _yaw_us   = SERVO_CENTER_US;
    _pitch_servo.writeMicroseconds(SERVO_CENTER_US);
    _yaw_servo.writeMicroseconds(SERVO_CENTER_US);
}

void servo_disable() {
    _pitch_servo.detach();
    _yaw_servo.detach();
    _enabled = false;
}

float servo_get_pitch_us() { return _pitch_us; }
float servo_get_yaw_us()   { return _yaw_us; }
