#ifndef CONTROL_H
#define CONTROL_H

#include <BasicLinearAlgebra.h>
#include "Parameters.h"
#include "Devices.h"

using namespace BLA;




struct ControlSample {
    unsigned long sampleTime;

    float leftMotorDutyCycle;
    float rightMotorDutyCycle;
    float err;
};


