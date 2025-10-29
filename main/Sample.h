#ifndef SAMPLE_H
#define SAMPLE_H

#include <Arduino.h>
#include "Parameters.h"
#include "Utils.h"
#include "Devices.h"


struct Sample {
  uint8_t buffer[SAMPLE_BYTE_SIZE];
};

Sample constructSample(MUX_TCA& mux, IMU_BMI270& imu, AS5600& magSensorLeft, AS5600& magSensorRight);



#endif