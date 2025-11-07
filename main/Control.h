#ifndef CONTROL_H
#define CONTROL_H

#include <BasicLinearAlgebra.h>
#include "Parameters.h"
#include "Devices.h"

using namespace BLA;


struct ControlState {

    ControlState(unsigned long startTime);

    unsigned long controlTime;

    Matrix<3, 1, double> W = {0.0, 0.0, 0.0};
    Matrix<3, 1, double> W_dot = {0.0, 0.0, 0.0};
    Matrix<2, 4, double> K;
    Matrix<3, 1, double> L;

    float leftMotorDutyCycle;
    float rightMotorDutyCycle;

    double err;

    void updateControlState(MUX_TCA& mux, IMU_BMI270& imu, LeftAS5600& magSensorLeft, RightAS5600& magSensorRight, unsigned long timeNow);
};

struct ControlSample {
    unsigned long sampleTime;

    float leftMotorDutyCycle;
    float rightMotorDutyCycle;

    float err;
};

#endif


