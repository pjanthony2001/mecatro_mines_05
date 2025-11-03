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

constexpr uint8_t DEVICE_SAMPLE_FMT = 0b10010000;
constexpr int SAMPLE_BYTE_SIZE = computeDataSize(DEVICE_SAMPLE_FMT);


constexpr char16_t SAMPLE_BATCH_SIZE = 5;
constexpr char16_t BUFFER_SIZE = 400;

constexpr  char* SSID = "Group5Mecatro";
constexpr  char* PASSWORD = "PASSWORD45678";

constexpr int LOCAL_UDP_PORT = 1234;
constexpr int CLIENT_UDP_PORT = 1234;

constexpr unsigned int MESSAGE_INTERVAL = 200; // in milliseconds, the interval to check messages, should be more than is sent by the client
constexpr unsigned int CONTROL_INTERVAL = 1; // in milliseconds, the interval to update control commands if necessary, and maybe sensor data is logged??
constexpr float SAMPLE_INTERVAL = 5; // in milliseconds, the interval to check my sampling devices (The highest ODR is 800Hz, but 400Hz is okay.)

constexpr unsigned long HEARTBEAT_TIMEOUT = 10000; // in milliseconds, the interval consider a connection is closed

constexpr bool PRINT_TELEMETRY = false;

constexpr int MAX_MESSAGE_DATA_SIZE = 32; // in bytes
constexpr int MAX_MESSAGE_SIZE = MAX_MESSAGE_DATA_SIZE + 2; //to account for the type byte and length byte

// COMMUNICATION MODE
constexpr CommMode commMode = CommMode::USB;

// CONTROL PARAMETERS


m_w = 0.41 ; %Masse d'une unite (roue + engrenage + rotor)
m_b = 1.8 ; %Masse du chassis du robot

I_x = 9.942 * 1e-3;
I_y = 5.31 * 1e-3;
I_z = 5.953 * 1e-3; %Moment d'inertie du chassis autour de chacun de ses axes, a C

Iw_x = 4.8886 * 1e-4;
Iw_y = 2.5233 * 1e-4;
Iw_z = 2.5307 * 1e-4; %Moment d'inertie de la roue autour de chacun de ses axes, a Cw

l = 0.171 ; %Distance entre les deux roues
d = 0.044 ; %Distance entre A et C
rho = 0.050 ; %position verticale(rayon du roue)
d_i = 0.100 ; %Distance verticale entre l'IMU et l'axe des roues

L = 0 ; %Inductance des moteurs 
R = 2.181818 ; %Resistance des moteurs
k = 0.3346 ; %Constante de couple

g = 9.81; %constante de gravitation


M = m_b + 2 * m_w;
I_psi = I_x + I_z + Iw_x + Iw_z + (m_w * l * l) / 2 + (Iw_y * l * l) / (2 * rho * rho);
Itot_y = I_y + 2 * Iw_y;

A_0 = M + (2 * Iw_y) / (rho * rho);
A_1 = m_b * d;
A_2 = -k / rho;
A_3 = m_b * d;
A_4 = Itot_y + m_b * d * d;
A_5 = -m_b * d * g;
A_6 = k;


B_0  = -2 * k / (rho * R);
B_1 = 2 * k / R;
B_2 = 1 / R;


A = inv(T) * Q_0;
B = inv(T) * Q_1;
 
A_W = [0 1 0
       0 0 g
       0 0 0];

B_W = [-d_i 0
        0 1
        1 0];


C_W = [2/rho 0 -2];



#endif