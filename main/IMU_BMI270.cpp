#include "IMU_BMI270.h"
#include "Parameters.h"
#include "Utils.h"
#include <Wire.h>


bool IMU_BMI270::init() {
  return imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) == BMI2_OK;
}

bool IMU_BMI270::readGyro(uint8_t* bufferPtr) {
  int status = imu.getSensorData();
  if (status != BMI2_OK) {
    return false;
  }

  printDebug(String("IMU SENSOR DATA STATUS: ") + (status == BMI2_OK));

  float data[3];
  data[0] = imu.data.gyroX;
  data[1] = imu.data.gyroY;
  data[2] = imu.data.gyroZ;

  printDebug(String("ASSERT: IMU_GYRO.byteSize == sizeof(data): ") + (IMU_ACC.byteSize == (sizeof(float) * 3)));// ASSERT byteSize == sizeof(data)
  memcpy(bufferPtr, data, IMU_GYRO.byteSize);
  return true;
}

bool IMU_BMI270::readAcc(uint8_t* bufferPtr) {
  int status = imu.getSensorData();
  if (status != BMI2_OK) {
    return false;
  }

  printDebug(String("IMU SENSOR DATA STATUS: ") + (status == BMI2_OK));

  float data[3];
  data[0] = imu.data.accelX;
  data[1] = imu.data.accelY;
  data[2] = imu.data.accelZ;

  printDebug(String("ASSERT: IMU_ACC.byteSize == sizeof(data): ") + (IMU_ACC.byteSize == sizeof(float) * 3));// ASSERT byteSize == sizeof(data)
  memcpy(bufferPtr, data, IMU_ACC.byteSize);
  return true;
}


bool IMU_BMI270::readData(uint8_t* bufferPtr) {
  int status = imu.getSensorData();
  if (status != BMI2_OK) {
    return false;
  }

  printDebug(String("IMU SENSOR DATA STATUS: ") + (status == BMI2_OK));

  float* fbuf = reinterpret_cast<float*>(bufferPtr);
  fbuf[0] = imu.data.gyroX;
  fbuf[1] = imu.data.gyroY;
  fbuf[2] = imu.data.gyroZ;
  fbuf[3] = imu.data.accelX;
  fbuf[4] = imu.data.accelY;
  fbuf[5] = imu.data.accelZ;

  printDebug(String("ASSERT: IMU_DATA.byteSize == sizeof(data): ") + (IMU_DATA.byteSize == sizeof(float) * 6));// ASSERT byteSize == sizeof(data)
  return true;
}