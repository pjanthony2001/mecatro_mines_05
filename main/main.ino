#include "WiFiUDP.h"
#include "Timers.h"
#include "Parameters.h"
#include "Sample.h"

#include "Devices.h"
#include "Motor.h"

WiFiUDP_AP wifiAP = WiFiUDP_AP(SSID, PASSWORD);
MUX_TCA mux = MUX_TCA();
IMU_BMI270 imu = IMU_BMI270();
LeftAS5600 magSensorLeft = LeftAS5600();
RightAS5600 magSensorRight = RightAS5600();
CircularBuffer<DEVICE_DATA> deviceSampleBuffer = CircularBuffer<DEVICE_DATA>(BUFFER_SIZE);

volatile unsigned long startTime; // IN MILLISECONDS SINCE LOOP STARTED


struct USBTelemetry {
  static void sendBatch(CircularBuffer<DEVICE_DATA>& buffer) {
    DEVICE_DATA batch[SAMPLE_BATCH_SIZE];
    uint8_t batchBuffer[SAMPLE_BATCH_SIZE * SAMPLE_BYTE_SIZE];
    if (deviceSampleBuffer.available() >= SAMPLE_BATCH_SIZE) {
        deviceSampleBuffer.popBatch(batch, SAMPLE_BATCH_SIZE);
        for (int i = 0; i < SAMPLE_BATCH_SIZE; i++) {
            batch[i].writeBytes(batchBuffer + i * SAMPLE_BYTE_SIZE);
        }
        
        printData((uint8_t*) batchBuffer, SAMPLE_BATCH_SIZE * SAMPLE_BYTE_SIZE);
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
    DEVICE_DATA s = constructDeviceSample(mux, imu, magSensorLeft, magSensorRight, DEVICE_SAMPLE_FMT);
    printDebug(String(s.leftEncoderData));
    deviceSampleBuffer.push(s);
    resetSampleFlag();
  }

  if (isControlFlag()) {
    DEVICE_DATA s = constructDeviceSample(mux, imu, magSensorLeft, magSensorRight, 0xF); // sample everything
    

    float sawtoothValue = sawtoothWave(1, 400, 5, millis() - startTime);
    setMotorDutyCycle(sawtoothValue, sawtoothValue);

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



