#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_AS5600.h>

constexpr float SENSOR_RAW_TO_RADS = (2 * M_PI) / ((2ul << 12)); // I am assuming that the [0, 2pi) range is stored as 12 bits in the ANGLE register

class AS5600 {
    // A wrapper for the AS5600 Adafruit class
    public:
        bool init();
        uint16_t read();

    private:
        Adafruit_AS5600 as5600;
};

#endif