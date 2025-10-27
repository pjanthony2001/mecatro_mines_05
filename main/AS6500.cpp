#include "AS6500.h"
#include "Utils.h"
#include "Wire.h"

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