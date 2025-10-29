#include "WiFiUDP.h"
#include "Timers.h"
#include "Parameters.h"
#include "Sample.h"

#include "Devices.h"
#include "Motor.h"

WiFiUDP_AP wifiAP = WiFiUDP_AP(SSID, PASSWORD);
MUX_TCA mux = MUX_TCA();
IMU_BMI270 imu = IMU_BMI270();
AS5600 magSensorLeft = AS5600();
AS5600 magSensorRight = AS5600();
CircularBuffer<Sample> sampleBuffer = CircularBuffer<Sample>(BUFFER_SIZE);

volatile unsigned long startTime; // IN MILLISECONDS SINCE LOOP STARTED


struct USBTelemetry {
  static void sendBatch(CircularBuffer<Sample>& buffer) {
    Sample batch[SAMPLE_BATCH_SIZE];
    uint8_t batchBuffer[SAMPLE_BATCH_SIZE * SAMPLE_BYTE_SIZE];
    if (sampleBuffer.available() >= SAMPLE_BATCH_SIZE) {
        sampleBuffer.popBatch(batch, SAMPLE_BATCH_SIZE);

        for (int i = 0; i < SAMPLE_BATCH_SIZE; i++) {
            memcpy(batchBuffer + i * SAMPLE_BYTE_SIZE, batch[i].buffer, SAMPLE_BYTE_SIZE);
        }

        printData((uint8_t*) batchBuffer, sizeof(batch));
    } 
  }
};

void configureDevices() {
  bool success = mux.init();
  printDebug(String("MUX Initialisation success: ") + success);
  
  mux.setPort(0);
  success = imu.init();
  printDebug(String("IMU Initialisation success: ") + success);

  mux.setPort(1);
  success = magSensorLeft.init();
  printDebug(String("MagSensorLeft Initialisation success: ") + success);

  mux.setPort(2);
  success = magSensorRight.init();
  printDebug(String("MagSensorRight Initialisation success: ") + success);
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  while(!Serial);;

  configureArduino();
  delay(300);

  configureDevices();
  delay(200);

  wifiAP.begin();
  delay(500);


  startTime = millis();
}


void loop() {
  if (isSampleFlag()) {
    // Sample s = constructSample(mux, imu, magSensorLeft, magSensorRight);
    // Sample s;
    // float data[2];
    // data[0] = 10.0;
    // data[1] = -0.1;

    // memcpy(s.buffer, data, sizeof(float) * 2);
    // sampleBuffer.push(s);
    resetSampleFlag();
  }

  if (isControlFlag()) {
    float sawtoothValue = sawtoothWave(1, 400, 5, millis() - startTime);
    printDebug(String("SAW TOOTH: ") + sawtoothValue);
    printDebug(String("TIME NOW: ") + (millis() - startTime));

    setMotorDutyCycle(sawtoothValue, sawtoothValue);

    resetControlFlag();
  }

  if (isTelemetryFlag()) {
    switch (commMode) {
      case CommMode::WIFI: {
        wifiAP.messageCheck();
        if (wifiAP.isConnected()) {
          wifiAP.sendBatch(sampleBuffer);
        }
      } break;

      case CommMode::USB: {
        if (Serial) { // Serial Connected
          // Check for Messages?
          USBTelemetry::sendBatch(sampleBuffer);
        }
      } break;
    }

    resetTelemetryFlag();

  }
}



