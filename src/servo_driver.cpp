#include <Arduino.h>
#include <Servo.h>
#include "servo_driver.h"

static Servo _pitch_servo;
static Servo _yaw_servo;
static float _pitch_us = SERVO_CENTER_US;
static float _yaw_us   = SERVO_CENTER_US;
static bool  _enabled  = false;

// Map angle to pulse width
// angle = 0 → SERVO_CENTER_US
// angle = +MAX → SERVO_MAX_US
// angle = -MAX → SERVO_MIN_US
static float angle_to_us(float angle_deg) {
    float clamped = constrain(angle_deg, -SERVO_MAX_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
    float scale   = clamped / SERVO_MAX_ANGLE_DEG;  // -1 to +1
    float us      = SERVO_CENTER_US + scale * (float)(SERVO_MAX_US - SERVO_CENTER_US);
    return constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
}

void servo_init() {
    _pitch_servo.attach(SERVO_PITCH_PIN, SERVO_MIN_US, SERVO_MAX_US);
    _yaw_servo.attach(SERVO_YAW_PIN,   SERVO_MIN_US, SERVO_MAX_US);

    _enabled = true;
    servo_center();

    Serial.print("[SERVO] Initialized on pins ");
    Serial.print(SERVO_PITCH_PIN);
    Serial.print(" (pitch) and ");
    Serial.print(SERVO_YAW_PIN);
    Serial.println(" (yaw)");
}

void servo_set_pitch(float angle_deg) {
    if (!_enabled) return;
    _pitch_us = angle_to_us(angle_deg);
    _pitch_servo.writeMicroseconds((int)_pitch_us);
}

void servo_set_yaw(float angle_deg) {
    if (!_enabled) return;
    _yaw_us = angle_to_us(angle_deg);
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