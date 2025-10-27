#ifndef BMI270_H
#define BMI270_H

#include <Arduino.h>
#include "SparkFun_BMI270_Arduino_Library.h"

class IMU_BMI270 {
    // A wrapper for the BMI270 Adafruit class
    public:
        bool init();
        bool readGyro(uint8_t* bufferPtr);
        bool readAcc(uint8_t* bufferPtr);
        bool readData(uint8_t* bufferPtr);
    private:
        BMI270 imu;
};

#endif