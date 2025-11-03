#include "Sample.h"


DEVICE_DATA constructDeviceSample(MUX_TCA& mux, IMU_BMI270& imu, LeftAS5600& magSensorLeft, RightAS5600& magSensorRight) {
  DEVICE_DATA sample;
  
  if ((DEVICE_SAMPLE_FMT >> 7) & 1) {
    sample.setSampleTime(millis());
  } 

  if (((DEVICE_SAMPLE_FMT >> 6) & 1) && ((DEVICE_SAMPLE_FMT >> 5) & 1)) {
    mux.setPort(0);
    if (!imu.readData(sample)) sample.isValid = false;
  } else if ((DEVICE_SAMPLE_FMT >> 5) & 1) {
    mux.setPort(0);
    if (!imu.readAcc(sample)) sample.isValid = false;
  } else if ((DEVICE_SAMPLE_FMT >> 6) & 1) {
    mux.setPort(0);
    if (!imu.readGyro(sample)) sample.isValid = false;
  }


  if ((DEVICE_SAMPLE_FMT >> 4) & 1) {
    if (!magSensorLeft.read(sample)) sample.isValid = false;
  }

  if ((DEVICE_SAMPLE_FMT >> 3) & 1) {
    if (!magSensorRight.read(sample)) sample.isValid = false;
  }

  return sample;
}