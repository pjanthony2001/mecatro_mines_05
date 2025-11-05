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

    L = {23.75, 175.1154, 408.626};

}


void ControlState::updateControlState(DEVICE_DATA& device_data, unsigned long timeNow) {
    if (!device_data.isValid) {
        printDebug("CONTROL: DEVICE SAMPLING NOT VALID");
    }

    float timeDeltaFloat = ((float) timeNow - controlTime) / 1000;
    controlTime = timeNow;

    Matrix<2, 1> e_W = {GYRO_DEG_TO_RAD * device_data.gyroData[0], ACC_G_TO_MS_2 * device_data.accelData[1]}; // check these


    float y_alpha = SENSOR_RAW_TO_RADS * (device_data.leftEncoderData + device_data.rightEncoderData);

    err = y_alpha - (C_W * W)(0, 0);


    Matrix<3, 1> W_hat_dot = A_W * W + B_W * e_W + err * L;

    W += timeDeltaFloat * W_hat_dot;


    
    Matrix<4, 1> X_hat = {W(0, 0), W(1, 0) - d_i * e_W(0, 0), W(2, 0), e_W(0, 0)};
    Matrix<2, 1> e = - K * X_hat;
    printDebug(String(err));
    leftMotorDutyCycle = e(0) / 12.0;
    rightMotorDutyCycle = e(1) / 12.0;
}
