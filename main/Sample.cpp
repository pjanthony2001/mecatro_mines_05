#include "Sample.h"


DEVICE_DATA constructDeviceSample(MUX_TCA& mux, IMU_BMI270& imu, LeftAS5600& magSensorLeft, RightAS5600& magSensorRight, const uint8_t& fmt) {
  DEVICE_DATA sample;
  
  if ((fmt >> 7) & 1) {
    sample.setSampleTime(millis());
  } 

  if (((fmt >> 6) & 1) && ((fmt >> 5) & 1)) {
    mux.setPort(IMU_PORT);
    if (!imu.readData(sample)) sample.isValid = false;
  } else if ((fmt >> 5) & 1) {
    mux.setPort(IMU_PORT);
    if (!imu.readAcc(sample)) sample.isValid = false;
  } else if ((fmt >> 6) & 1) {
    mux.setPort(IMU_PORT);
    if (!imu.readGyro(sample)) sample.isValid = false;
  }


  if ((fmt >> 4) & 1) {
    mux.setPort(LEFT_ENCODER_PORT);
    if (!magSensorLeft.read(sample)) sample.isValid = false;
  }

  if ((fmt >> 3) & 1) {
    mux.setPort(RIGHT_ENCODER_PORT);
    if (!magSensorRight.read(sample)) sample.isValid = false;
  }

  return sample;
}