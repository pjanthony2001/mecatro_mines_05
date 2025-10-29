#include "Devices.h"
#include "Utils.h"
#include "Wire.h"
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

bool LeftAS5600::read(DEVICE_DATA& dataStruct) {
    if (as5600.isAGCminGainOverflow()) {
        printDebug("LEFT AS5600 Magnet too strong");
    }

    dataStruct.leftEncoderData = as5600.getAngle();
    return true;
}

bool RightAS5600::read(DEVICE_DATA& dataStruct) {
    if (as5600.isAGCminGainOverflow()) {
        printDebug("RIGHT AS5600 Magnet too strong");
    }

    dataStruct.rightEncoderData = as5600.getAngle();
    return true;
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

bool IMU_BMI270::getSensorData() {
  int status = imu.getSensorData();
  printDebug(String("IMU SENSOR DATA STATUS: ") + (status == BMI2_OK));
  return status == BMI2_OK;
}

bool IMU_BMI270::readGyro(DEVICE_DATA& dataStruct) {
  if (!getSensorData()) return false;

  dataStruct.gyroData[0] = imu.data.gyroX;
  dataStruct.gyroData[1] = imu.data.gyroY;
  dataStruct.gyroData[2] = imu.data.gyroZ;

  dataStruct.fmt |= 0b01000000; 
  return true;
}

bool IMU_BMI270::readAcc(DEVICE_DATA& dataStruct) {
  if (!getSensorData()) return false;

  dataStruct.accelData[0] = imu.data.accelX;
  dataStruct.accelData[1] = imu.data.accelY;
  dataStruct.accelData[2] = imu.data.accelZ;

  dataStruct.fmt |= 0b00100000; 
  return true;
}


bool IMU_BMI270::readData(DEVICE_DATA& dataStruct) {
  if (!getSensorData()) return false;

  dataStruct.gyroData[0] = imu.data.gyroX;
  dataStruct.gyroData[1] = imu.data.gyroY;
  dataStruct.gyroData[2] = imu.data.gyroZ;
  dataStruct.accelData[0] = imu.data.accelX;
  dataStruct.accelData[1] = imu.data.accelY;
  dataStruct.accelData[2] = imu.data.accelZ;

  dataStruct.fmt |= 0b01100000; 
  return true;
}



void DEVICE_DATA::writeBytes(uint8_t *dataBuffer) {


}

size_t DEVICE_DATA::computeDataSize() {

}
