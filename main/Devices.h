#ifndef DEVICES_H
#define DEVICES_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_AS5600.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <SparkFun_I2C_Mux_Arduino_Library.h>

constexpr float SENSOR_RAW_TO_RADS = (2 * M_PI) / ((2ul << 12)); // I am assuming that the [0, 2pi) range is stored as 12 bits in the ANGLE register

class AS5600 {
    // A wrapper for the AS5600 Adafruit class
    public:
        bool init();
        uint16_t read();

    private:
        Adafruit_AS5600 as5600;
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
        bool readGyro(uint8_t* bufferPtr);
        bool readAcc(uint8_t* bufferPtr);
        bool readData(uint8_t* bufferPtr);
    private:
        BMI270 imu;
};

#endif