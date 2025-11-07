#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// MOTOR CONSTANTS
constexpr uint8_t DIRECTION_MOTOR_LEFT = 8;
constexpr uint8_t DIRECTION_MOTOR_RIGHT = 12;

void setMotorDutyCycle(double const& leftMotorDC, double const& rightMotorDC);
float sawtoothWave(float amplitude, float period, float phase, float x);
float unitSawtoothWave(float x);

#endif