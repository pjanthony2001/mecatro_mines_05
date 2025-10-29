#include "Devices.h"
#include "Utils.h"
#include "Wire.h"
#include "IMU_BMI270.h"
#include "Parameters.h"
#include <SparkFun_I2C_Mux_Arduino_Library.h> 


bool AS5600::init() {
    if (!as5600.begin(AS5600_DEFAULT_ADDR, &Wire1)) {
        printDebug("Could not find AS5600 sensor, check wiring!");
        return false;
    }

    as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
    as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);

    as5600.setZPosition(0);
    as5600.setMPosition(4095);
    as5600.setMaxAngle(4095);

    printDebug(String("AS5600 Magnet detected: ") + as5600.isMagnetDetected());


    if (as5600.isAGCminGainOverflow()) {
        printDebug("AS5600 Magnet too strong");
    } else if (as5600.isAGCmaxGainOverflow()) {
        printDebug("AS5600 Magnet too weak");
    }

    return true;
};

uint16_t AS5600::read() {
    if (as5600.isAGCminGainOverflow()) {
        printDebug("AS5600 Magnet too strong");
    }
    uint16_t val = as5600.getAngle();
    printDebug(String(val));
    return val;
}

//Enables a specific port number
void MUX_TCA::setPort(uint8_t portNumber) {
  if (!mux.setPort(portNumber)) {
    printDebug("MUX FAILED TO SET PORT");
  }
}

//Disables a specific port number
uint8_t MUX_TCA::getPort() {
  return mux.getPort();
}

bool MUX_TCA::init() {
  if (!mux.begin(QWIIC_MUX_DEFAULT_ADDRESS, Wire1)) {
    printDebug("ERROR: Mux initialisation failed!");
    return false;
  }
  return true;
}



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