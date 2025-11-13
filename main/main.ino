#include "WiFiUDP.h"
#include "Timers.h"
#include "Parameters.h"
#include "Sample.h"

#include "Devices.h"
#include "Motor.h"
#include "Control.h"

WiFiUDP_AP wifiAP = WiFiUDP_AP(SSID, PASSWORD);
MUX_TCA mux = MUX_TCA();
IMU_BMI270 imu = IMU_BMI270();
LeftAS5600 magSensorLeft = LeftAS5600();
RightAS5600 magSensorRight = RightAS5600();
CircularBuffer<DEVICE_DATA> deviceSampleBuffer = CircularBuffer<DEVICE_DATA>(BUFFER_SIZE);

//CONTROL STUFF IN A STRUCT !!!!
unsigned long controlTime;
constexpr float SENSOR_RAW_TO_RADS = (2 * M_PI) / ((1ul << 12)); // I am assuming that the [0, 2pi) range is stored as 12 bits in the ANGLE register
constexpr float GYRO_DEG_TO_RAD = (M_PI / 180);
constexpr float ACC_G_TO_MS_2 = 9.81;

Matrix<3, 1, double> W;
Matrix<3, 1, double> W_dot;
Matrix<2, 4, double> K = {-8, -50.0, -7, -0.05, 
                          -8, -50.0, -7, -0.05};


Matrix<3, 1, double> L = {18.9687, 154.1383, 318.5525};

double alpha_ = 0.2089;
double beta_ = 0.0244;

Matrix<4, 4, double> N = {alpha_, 0.0, beta_, 0.0,
                          0.0, alpha_, 0.0, beta_,
                          0.0, 0.0, 1.0, 0.0,
                          0.0, 0.0, 0.0, 1.0};



volatile unsigned long startTime; // IN MILLISECONDS SINCE LOOP STARTED


struct USBTelemetry {
  static void sendBatch(CircularBuffer<DEVICE_DATA>& buffer) {
    DEVICE_DATA batch[SAMPLE_BATCH_SIZE];
    uint8_t batchBuffer[SAMPLE_BATCH_SIZE * SAMPLE_BYTE_SIZE];
    if (buffer.available() >= SAMPLE_BATCH_SIZE) {
        buffer.popBatch(batch, SAMPLE_BATCH_SIZE);
        for (int i = 0; i < SAMPLE_BATCH_SIZE; i++) {
            batch[i].writeBytes(batchBuffer + i * SAMPLE_BYTE_SIZE);
        }
        
        printData((uint8_t*) batchBuffer, SAMPLE_BATCH_SIZE * SAMPLE_BYTE_SIZE);
    } 
  }
};

inline void configureDevices() {
  bool success = mux.init();
  printDebug(String("MUX Initialisation success: ") + success);
  
  mux.setPort(LEFT_ENCODER_PORT);
  success = magSensorLeft.init();
  printDebug(String("MagSensorLeft Initialisation success: ") + success);


  mux.setPort(RIGHT_ENCODER_PORT);
  success = magSensorRight.init();
  printDebug(String("MagSensorRight Initialisation success: ") + success);

  mux.setPort(IMU_PORT);
  success = imu.init();
  printDebug(String("IMU Initialisation success: ") + success);

}

void setup() {
  Serial.begin(115200);
  delay(1000);
  while(!Serial);;

  Serial.println("Starting setup...");
  Serial.flush();
  
  configureArduino();
  delay(300);

  configureDevices();
  delay(200);

  // I2C configuration
  // Set I2C clock speed to 400kHz (fast mode)
  // Note: this has to be done after starting the encoder, because their code reset the clock to 100kHz.
  Wire1.setClock(400000);
  Wire1.setTimeout(500);
  Wire1.begin();

  wifiAP.begin();
  delay(500);

  startTime = millis();
}


void loop() {

  // if (isSampleFlag()) {
  //   DEVICE_DATA s = constructDeviceSample(mux, imu, magSensorLeft, magSensorRight, DEVICE_SAMPLE_FMT);
  //   deviceSampleBuffer.push(s);
  //   resetSampleFlag();
  // }

  if (isControlFlag()) {
    
    unsigned long timeNow = millis() - startTime;
    int num_int = 15;
    double timeDelta = ((double) (timeNow - controlTime)) / (1000 * num_int);
    controlTime = timeNow;


    mux.setPort(LEFT_ENCODER_PORT);
    long alpha_l = magSensorLeft.getCumAngle();
    // mux.setPort(RIGHT_ENCODER_PORT);
    // long alpha_r = magSensorRight.getCumAngle();
    // mux.setPort(IMU_PORT);
    // BMI270_SensorData data = imu.getData();


    // Matrix<2, 1, double> e_W = {GYRO_DEG_TO_RAD * data.gyroY, -ACC_G_TO_MS_2 * data.accelX}; // check these

    // double y_alpha = SENSOR_RAW_TO_RADS * (alpha_l + alpha_r);

    // double err;
    // for (int i = 0; i < num_int; i++) {
    //   err = y_alpha - (C_W * W)(0, 0);
    //   Matrix<3,1,double> W_dot1 = (A_W * W) + (B_W * e_W) + (L * err);
    //   Matrix<3,1,double> W_temp = W + timeDelta * W_dot1;
    //   err = y_alpha - (C_W * W_temp)(0, 0);
    //   Matrix<3,1,double> W_dot2 = (A_W * W_temp) + (B_W * e_W) + (L * err);
    //   W += 0.5 * timeDelta * (W_dot1 + W_dot2);
    // }

    // Matrix<4, 1, double> X_hat = {W(0, 0), W(1, 0) - d_i * e_W(0, 0), W(2, 0), e_W(0, 0)};
    // X_hat = N * X_hat;
    // Matrix<2, 1, double> e = - K * X_hat;

    float freq = 1;
    double leftMotorDutyCycle = squareWave(timeNow, freq);
    double rightMotorDutyCycle = 0.0;

    DEVICE_DATA s;
    s.leftEncoderData = alpha_l;
    s.sampleTime = timeNow;
    s.float_1 = leftMotorDutyCycle;
    s.fmt = DEVICE_SAMPLE_FMT;
    deviceSampleBuffer.push(s);


    setMotorDutyCycle(-leftMotorDutyCycle, -rightMotorDutyCycle);
    // Serial.print(alpha_l * SENSOR_RAW_TO_RADS);
    // Serial.print("\t");
    // Serial.print(alpha_r * SENSOR_RAW_TO_RADS);

    resetControlFlag();
  }

  if (isTelemetryFlag()) {
    switch (commMode) {
      case CommMode::WIFI: {
        wifiAP.messageCheck();
        if (wifiAP.isConnected()) {
          wifiAP.sendBatch(deviceSampleBuffer);
        }
      } break;

      case CommMode::USB: {
        if (Serial) { // Serial Connected
          // Check for Messages?
          USBTelemetry::sendBatch(deviceSampleBuffer);
        }
      } break;
    }

    resetTelemetryFlag();

  }
}



