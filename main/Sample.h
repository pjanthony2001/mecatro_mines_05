#ifndef SAMPLE_H
#define SAMPLE_H

#include <Arduino.h>
#include "Parameters.h"
#include "Utils.h"
#include "Devices.h"


struct Sample {
  uint8_t buffer[SAMPLE_BYTE_SIZE];
};

DEVICE_DATA constructSample(MUX_TCA& mux, IMU_BMI270& imu, LeftAS5600& magSensorLeft, RightAS5600& magSensorRight);


#endif