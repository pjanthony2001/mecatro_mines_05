#ifndef TELEMETRY_PARAMETERS
#define TELEMETRY_PARAMETERS

#include <Arduino.h>
#include "Devices.h"

enum CommMode {
    USB,
    WIFI
};

enum DebugMode {
    DEBUG_OFF,
    DEBUG_ON
};

struct SampleDataType {
    uint8_t id;
    uint8_t byteSize;  // Various byte sizes of the whole Sample that is being constructed
    constexpr SampleDataType(uint8_t id, uint8_t byteSize) : id(id), byteSize(byteSize) {}
    constexpr operator uint8_t() const {
        return id;
    }
};



constexpr SampleDataType IMU_GYRO{0, 12}; // For now it is floats, but later change API to convert to 16bit signed integers, so 3 * 4bytes each
constexpr SampleDataType IMU_ACC{1, 12};  // For now it is floats, but later change API to convert to 16bit signed integers, so 3 * 4bytes each
constexpr SampleDataType IMU_DATA{2, 24}; // For now it is floats, but later change API to convert to 16bit signed integers, so 6 * 4bytes each

constexpr SampleDataType LEFT_AS6500_DATA{3, 2}; // For now it is 16 bit unsigned
constexpr SampleDataType RIGHT_AS6500_DATA{4, 2};

constexpr SampleDataType FLOAT_DATA{5, 4};


constexpr int computeDataSize(const uint8_t& fmt) {
  /// SAMPLE STRUCTURE : [sampleTime][gyroData][accelData][leftEncoder][rightEncoder][0][0][0]

  int total = 0;

  if ((fmt >> 3) & 1) {
    total += 2;
  }

  if ((fmt >> 4) & 1) {
    total += 2;
  }

  if ((fmt >> 5) & 1) {
    total += 12;
  }

  if ((fmt >> 6) & 1) {
    total += 12;
  }

  if ((fmt >> 7) & 1) {
    total += 4;
  }

  return total;
}

/// DEVICE SAMPLE STRUCTURE : [sampleTime][gyroData][accelData][leftEncoder][rightEncoder][0][0][0]

constexpr uint8_t DEVICE_SAMPLE_FMT = 0b00000000;
constexpr int SAMPLE_BYTE_SIZE = computeDataSize(DEVICE_SAMPLE_FMT);


constexpr char16_t SAMPLE_BATCH_SIZE = 5;
constexpr char16_t BUFFER_SIZE = 400;

constexpr  char* SSID = "Group5Mecatro";
constexpr  char* PASSWORD = "PASSWORD45678";

constexpr int LOCAL_UDP_PORT = 1234;
constexpr int CLIENT_UDP_PORT = 1234;

constexpr unsigned int MESSAGE_INTERVAL = 200; // in milliseconds, the interval to check messages, should be more than is sent by the client
constexpr unsigned int CONTROL_INTERVAL = 1; // in milliseconds, the interval to update control commands if necessary, and maybe sensor data is logged??
constexpr float SAMPLE_INTERVAL = 50; // in milliseconds, the interval to check my sampling devices (The highest ODR is 800Hz, but 400Hz is okay.)

constexpr unsigned long HEARTBEAT_TIMEOUT = 10000; // in milliseconds, the interval consider a connection is closed

constexpr bool PRINT_TELEMETRY = false;

constexpr int MAX_MESSAGE_DATA_SIZE = 32; // in bytes
constexpr int MAX_MESSAGE_SIZE = MAX_MESSAGE_DATA_SIZE + 2; //to account for the type byte and length byte

// COMMUNICATION MODE
constexpr CommMode commMode = CommMode::USB;




#endif