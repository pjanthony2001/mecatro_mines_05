#ifndef DEVICES_H
#define DEVICES_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_AS5600.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <SparkFun_I2C_Mux_Arduino_Library.h>

constexpr float SENSOR_RAW_TO_RADS = (2 * M_PI) / ((2ul << 12)); // I am assuming that the [0, 2pi) range is stored as 12 bits in the ANGLE register


struct DEVICE_DATA {

    /// SAMPLE STRUCTURE : [sampleTime][gyroData][accelData][leftEncoder][rightEncoder][0][0][0]

    unsigned long sampleTime;

    float gyroData[3];
    float accelData[3];

    uint16_t leftEncoderData;
    uint16_t rightEncoderData;

    uint8_t fmt = 0; // FORMAT TO DECODE
    
    bool isValid = true;

    void writeBytes(uint8_t* dataBuffer);
    size_t computeDataSize();
    
};


class AS5600 {
    // A wrapper for the AS5600 Adafruit class
    public:
        bool init();

    protected:
        Adafruit_AS5600 as5600;
};


class LeftAS5600: public AS5600{
    public:
        bool read(DEVICE_DATA& dataStruct);
};

class RightAS5600: public AS5600{
    public:
        bool read(DEVICE_DATA& dataStruct);
};

class MUX_TCA { //Wrapper for Sparkfun Libraries Mux
    public:
        bool init();
        void setPort(uint8_t port);
        uint8_t getPort();

    private:
        QWIICMUX mux;
};


class IMU_BMI270 {
    // A wrapper for the BMI270 Adafruit class
    public:
        bool init();
        bool readGyro(DEVICE_DATA& dataStruct);
        bool readAcc(DEVICE_DATA& dataStruct);
        bool readData(DEVICE_DATA& dataStruct);

    private:
        BMI270 imu;
        bool getSensorData();
};



#endif