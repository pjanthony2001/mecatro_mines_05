#include "Control.h"
#include "Parameters.h"
#include "Utils.h"

using namespace BLA;

constexpr float SENSOR_RAW_TO_RADS = (2 * M_PI) / ((1ul << 12)); // I am assuming that the [0, 2pi) range is stored as 12 bits in the ANGLE register
constexpr float GYRO_DEG_TO_RAD = (M_PI / 180);
constexpr float ACC_G_TO_MS_2 = 9.81;

ControlState::ControlState(unsigned long startTime)  : controlTime(startTime) {
    K = {-4.8045, -55.2805, -21.2007, -3.6603,
         -4.8045, -55.2805, -21.2007, -3.6603};

    L = {18.9276, 150, 318.5525};

}


void ControlState::updateControlState(MUX_TCA& mux, IMU_BMI270& imu, LeftAS5600& magSensorLeft, RightAS5600& magSensorRight, unsigned long timeNow) {
    
}
