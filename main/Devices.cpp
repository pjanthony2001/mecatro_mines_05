#include "Devices.h"
#include "Utils.h"
#include "Wire.h"
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
    as5600.setHysteresis(AS5600_HYSTERESIS_1LSB);

    if (as5600.isAGCminGainOverflow()) {
        printDebug("AS5600 Magnet too strong");
    } else if (as5600.isAGCmaxGainOverflow()) {
        printDebug("AS5600 Magnet too weak");
    }


    uint8_t agc = as5600.getAGC();
    uint16_t magnitude = as5600.getMagnitude();



    Serial.println("---- AS5600 Diagnostics ----");
    Serial.print("Magnet detected: ");
    Serial.println(as5600.isMagnetDetected() ? "YES" : "NO");

    Serial.print("AGC (Automatic Gain Control): ");
    Serial.println(agc);

    Serial.print("Magnitude (signal strength): ");
    Serial.println(magnitude);

    if (as5600.isAGCminGainOverflow()) Serial.println("⚠️ Magnet too strong");
    if (as5600.isAGCmaxGainOverflow()) Serial.println("⚠️ Magnet too weak");

    Serial.println("-----------------------------\n");


    lastAngle = as5600.getAngle();

    return true;
};

void AS5600::updateCumPosition() {
  uint16_t currAngle = as5600.getAngle();
  if ((lastAngle > 2048) && (currAngle < (lastAngle - 2048))) {
    currCumPosition += 4096 - lastAngle + currAngle;
  } else if ((currAngle > 2048) && (lastAngle < (currAngle - 2048))) {
    currCumPosition -= 4096 - currAngle + lastAngle;
  } else {
    currCumPosition += lastAngle;
    currCumPosition -= currAngle;
  }

  lastAngle = currAngle;
}

long AS5600::getCumAngle() {
  updateCumPosition();
  return currCumPosition;
}

uint16_t AS5600::getAngle() {
  updateCumPosition();
  return lastAngle;
}

bool LeftAS5600::read(DEVICE_DATA& dataStruct) {
    if (as5600.isAGCminGainOverflow()) {
        printDebug("LEFT AS5600 Magnet too strong");
    }

    dataStruct.leftEncoderData = as5600.getAngle();
    dataStruct.fmt |= 0b00010000;
    return true;
}

bool RightAS5600::read(DEVICE_DATA& dataStruct) {
    if (as5600.isAGCminGainOverflow()) {
        printDebug("RIGHT AS5600 Magnet too strong");
    }

    dataStruct.rightEncoderData = as5600.getAngle();
    dataStruct.fmt |= 0b00001000;
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
  uint8_t resp = imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1);
  // Serial.println("Running component retrim (gyro)...");
  // imu.performComponentRetrim(); // reduces gyro scale error
  int8_t err = BMI2_OK;

  //   // Set accelerometer config
    // bmi2_sens_config accelConfig;
    // accelConfig.type = BMI2_ACCEL;
    // accelConfig.cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    // accelConfig.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    // accelConfig.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    // accelConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
    // err = imu.setConfig(accelConfig);

  //   // Set gyroscope config
    // bmi2_sens_config gyroConfig;
    // gyroConfig.type = BMI2_GYRO;
    // gyroConfig.cfg.gyr.odr = BMI2_ACC_ODR_100HZ;
    // gyroConfig.cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    // gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    // gyroConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_250;
    // gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_125;
    // gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    // err = imu.setConfig(gyroConfig);
  return resp == BMI2_OK;
}

bool IMU_BMI270::getSensorData() {
  int status = imu.getSensorData();
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

BMI270_SensorData IMU_BMI270::getData() {
  if (!getSensorData()) {
    BMI270_SensorData data;
    return data;
  }
  return imu.data;
}



void DEVICE_DATA::writeBytes(uint8_t *dataBuffer) {
  size_t curr_idx = 0;

  if ((fmt >> 7) & 1) {
    memcpy(dataBuffer + curr_idx, &sampleTime, 4);
    curr_idx += 4;
  } 

  if ((fmt >> 6) & 1) {
    memcpy(dataBuffer + curr_idx, (uint8_t*) gyroData, 12);
    curr_idx += 12;
  }
  
  if ((fmt >> 5) & 1) {
    memcpy(dataBuffer + curr_idx, (uint8_t*) accelData, 12);
    curr_idx += 12;
  }

  if ((fmt >> 4) & 1) {
    memcpy(dataBuffer + curr_idx, &leftEncoderData, 4);
    curr_idx += 4;
  }

  if ((fmt >> 3) & 1) {
    memcpy(dataBuffer + curr_idx, &rightEncoderData, 4);
    curr_idx += 4;  
  }

  if ((fmt >> 2) & 1) {
    memcpy(dataBuffer + curr_idx, &float_1, 4);
    curr_idx += 4;  
  }
  
}

void DEVICE_DATA::setSampleTime(unsigned long sampleTime) {
  this->sampleTime = sampleTime;
  this->fmt |= 0b10000000; 
}




