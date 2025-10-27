#ifndef TELEMETRY_PARAMETERS
#define TELEMETRY_PARAMETERS

#include <Arduino.h>

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


constexpr int computeSampleByteSize(const SampleDataType* sampleStructure, const size_t nSamples) {
  int sampleSize = 0;
  for (int i = 0; i < nSamples; i++) {
    sampleSize += sampleStructure[i].byteSize;
  }

  return sampleSize;
}

constexpr size_t NUM_SAMPLES = 1;
constexpr SampleDataType SAMPLE_STRUCTURE[NUM_SAMPLES] = {IMU_DATA}; // TODO: ADD A LOGGING BYTE6
// WARNING I HAVE NOT MAKE IT ADAPTIVE TO ANY PLACEMENT OF FLOAT DATA, for now add all float data to the front of the sample structure
constexpr int SAMPLE_BYTE_SIZE = computeSampleByteSize(SAMPLE_STRUCTURE, NUM_SAMPLES);


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