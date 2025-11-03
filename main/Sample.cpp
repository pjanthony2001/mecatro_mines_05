#include "Sample.h"


DEVICE_DATA constructDeviceSample(MUX_TCA& mux, IMU_BMI270& imu, LeftAS5600& magSensorLeft, RightAS5600& magSensorRight, uint8_t& fmt) {
  DEVICE_DATA sample;
  
  if ((fmt >> 7) & 1) {
    sample.setSampleTime(millis());
  } 

  if (((fmt >> 6) & 1) && ((fmt >> 5) & 1)) {
    mux.setPort(0);
    if (!imu.readData(sample)) sample.isValid = false;
  } else if ((fmt >> 5) & 1) {
    mux.setPort(0);
    if (!imu.readAcc(sample)) sample.isValid = false;
  } else if ((fmt >> 6) & 1) {
    mux.setPort(0);
    if (!imu.readGyro(sample)) sample.isValid = false;
  }


  if ((fmt >> 4) & 1) {
    mux.setPort(1);
    if (!magSensorLeft.read(sample)) sample.isValid = false;
  }

  if ((fmt >> 3) & 1) {
    mux.setPort(2);
    if (!magSensorRight.read(sample)) sample.isValid = false;
  }

  return sample;
}