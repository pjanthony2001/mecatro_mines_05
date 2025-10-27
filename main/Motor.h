#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// MOTOR CONSTANTS
constexpr uint8_t DIRECTION_MOTOR_LEFT = 7;
constexpr uint8_t DIRECTION_MOTOR_RIGHT = 13;

void setMotorDutyCycle(float const& leftMotorDC, float const& rightMotorDC);
float sawtoothWave(float amplitude, float period, float phase, float x);
float unitSawtoothWave(float x);

#endif