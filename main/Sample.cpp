#include "Sample.h"


Sample constructSample(MUX_TCA& mux, IMU_BMI270& imu, AS5600& magSensorLeft, AS5600& magSensorRight) {
  Sample sample;
  int currIdx = 0;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {

    switch (SAMPLE_STRUCTURE[i]) {
      case IMU_DATA : {

        mux.setPort(0);
        imu.readData(sample.buffer + currIdx);

        currIdx += IMU_DATA.byteSize;
      } break;

      case IMU_ACC : {
        mux.setPort(0);
        imu.readAcc(sample.buffer + currIdx);
        
        currIdx += IMU_ACC.byteSize;
      } break;


      case IMU_GYRO : {
        mux.setPort(0);
        imu.readGyro(sample.buffer + currIdx);
        
        currIdx += IMU_GYRO.byteSize;
      } break;

      case LEFT_AS6500_DATA : {
        mux.setPort(1);
        uint16_t data = magSensorLeft.read();

        memcpy(sample.buffer + currIdx, &data, LEFT_AS6500_DATA.byteSize);  // rawData is [ACC data (6 bytes), GYRO data (6 bytes)]
        currIdx += LEFT_AS6500_DATA.byteSize;

      } break;

      case RIGHT_AS6500_DATA : {
        mux.setPort(2);
        uint16_t data = magSensorRight.read();
        
        memcpy(sample.buffer + currIdx, &data, RIGHT_AS6500_DATA.byteSize);  // rawData is [ACC data (6 bytes), GYRO data (6 bytes)]
        currIdx += RIGHT_AS6500_DATA.byteSize;
      } break;

      case FLOAT_DATA : {
        float x = 19.000f; // TODO: REPLACE WITH FUNCTION CALL TO RETURN PROPER DATA

        memcpy(sample.buffer + currIdx, &x, RIGHT_AS6500_DATA.byteSize);  // rawData is [ACC data (6 bytes), GYRO data (6 bytes)]
        currIdx += FLOAT_DATA.byteSize;
        
      } break;
    }
  }

  return sample;
}
