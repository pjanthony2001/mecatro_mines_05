#include "Motor.h"
#include <cmath>

void setMotorDutyCycle(float const& leftMotorDC, float const& rightMotorDC) {
    // Set direction
    digitalWrite(DIRECTION_MOTOR_RIGHT, rightMotorDC > 0);
    digitalWrite(DIRECTION_MOTOR_LEFT, leftMotorDC > 0);
    // Set duty cycle: the timer counts up to R_GPT2->GTPR.
    int counterValue = floor(abs(rightMotorDC) * R_GPT2->GTPR);
    if (counterValue > R_GPT2->GTPR)
        counterValue = R_GPT2->GTPR;
    R_GPT2->GTCCR[0] = counterValue;

    counterValue = floor(abs(leftMotorDC) * R_GPT7->GTPR);
    if (counterValue > R_GPT7->GTPR)
        counterValue = R_GPT7->GTPR;
    R_GPT7->GTCCR[1]  = counterValue;
}


float sawtoothWave(float amplitude, float period, float phase, float x) {
    float temp = (x / period + phase);
    return amplitude * (temp - std::floor(temp));
}

float unitSawtoothWave(float x) {
  return x - std::floor(x);
}