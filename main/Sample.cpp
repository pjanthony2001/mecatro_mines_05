#include "Sample.h"


DEVICE_DATA constructDeviceSample(MUX_TCA& mux, IMU_BMI270& imu, LeftAS5600& magSensorLeft, RightAS5600& magSensorRight) {
  DEVICE_DATA sample;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    switch (SAMPLE_STRUCTURE[i]) {
      case IMU_DATA : {
        mux.setPort(0);
        if (!imu.readData(sample)) sample.isValid = false;
      } break;

      case IMU_ACC : {
        mux.setPort(0);
        if (!imu.readAcc(sample)) sample.isValid = false;
      } break;


      case IMU_GYRO : {
        mux.setPort(0);
        if (!imu.readAcc(sample)) sample.isValid = false;
      } break;

      case LEFT_AS6500_DATA : {
        mux.setPort(1);
        if (!magSensorLeft.read(sample)) sample.isValid = false;
      } break;

      case RIGHT_AS6500_DATA : {
        mux.setPort(2);
        if (!magSensorRight.read(sample)) sample.isValid = false;
      } break;
    }
  }

  return sample;
}
