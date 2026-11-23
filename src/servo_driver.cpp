#include <Arduino.h>
#include <Servo.h>
#include "servo_driver.h"

static Servo _pitch_servo;
static Servo _yaw_servo;
static float _pitch_us = SERVO_PITCH_CENTER_US;
static float _yaw_us   = SERVO_YAW_CENTER_US;
static bool  _enabled  = false;

// Nozzle angle [deg] to pulse width, applying the invert flag and clamping
// to the axis's travel so it never commands past a limit.
static float angle_to_us(float angle_deg, bool invert, float center_us, float span_us,
                         float us_per_deg) {
    const float a = invert ? -angle_deg : angle_deg;
    return constrain(center_us + a * us_per_deg, center_us - span_us, center_us + span_us);
}

void servo_init() {
    _pitch_servo.attach(SERVO_PITCH_PIN, SERVO_PITCH_MIN_US, SERVO_PITCH_MAX_US);
    _yaw_servo.attach(SERVO_YAW_PIN,     SERVO_YAW_MIN_US,   SERVO_YAW_MAX_US);

    _enabled = true;
    servo_center();

    Serial.print("[SERVO] Initialized: pitch on pin ");
    Serial.print(SERVO_PITCH_PIN);
    Serial.print(", yaw on pin ");
    Serial.println(SERVO_YAW_PIN);
}

void servo_set_pitch(float angle_deg) {
    if (!_enabled) return;
    _pitch_us = angle_to_us(angle_deg, (bool)SERVO_PITCH_INVERT, SERVO_PITCH_CENTER_US,
                            SERVO_PITCH_SPAN_US, SERVO_PITCH_US_PER_DEG);
    _pitch_servo.writeMicroseconds((int)_pitch_us);
}

void servo_set_yaw(float angle_deg) {
    if (!_enabled) return;
    _yaw_us = angle_to_us(angle_deg, (bool)SERVO_YAW_INVERT, SERVO_YAW_CENTER_US,
                          SERVO_YAW_SPAN_US, SERVO_YAW_US_PER_DEG);
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
