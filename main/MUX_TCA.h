#ifndef MUX_TCA_H
#define MUX_TCA_H

#include <Arduino.h>
#include "Utils.h"
#include <SparkFun_I2C_Mux_Arduino_Library.h>

class MUX_TCA { //Wrapper for Sparkfun Libraries Mux
    public:
        bool init();
        void setPort(uint8_t port);
        uint8_t getPort();

    private:
        QWIICMUX mux;
};

#endif