#ifndef CONTROL_H
#define CONTROL_H

#include <BasicLinearAlgebra.h>
#include "Parameters.h"
#include "Devices.h"

using namespace BLA;


struct ControlState {

    ControlState(unsigned long startTime);

    unsigned long controlTime;

    Matrix<3, 1> W = {0.0, 0.0, 0.0};
    Matrix<2, 4> K;
    Matrix<3, 1> L;

    float leftMotorDutyCycle;
    float rightMotorDutyCycle;

    float err;

    void updateControlState(DEVICE_DATA& device_data, unsigned long timeNow);
};

struct ControlSample {
    unsigned long sampleTime;

    float leftMotorDutyCycle;
    float rightMotorDutyCycle;

    float err;
};

#endif


