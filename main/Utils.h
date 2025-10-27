#ifndef TELEMETRY_UTILS
#define TELEMETRY_UTILS

#include <Arduino.h>
#include "Parameters.h"

inline void printDebug(const char* message) {
    Serial.print("DEBUG: ");
    Serial.println(message);
    Serial.flush();
}

inline void printDebug(String message) {
    Serial.print("DEBUG: ");
    Serial.println(message.c_str());
    Serial.flush();
}

inline void printDebugTelemetry(const char* message) {
  if (PRINT_TELEMETRY) {
    Serial.print("DEBUG TELEMETRY: ");
    Serial.println(message);
    Serial.flush();
  }
}

inline void printDebugTelemetry(String message) {
  if (PRINT_TELEMETRY) {
    Serial.print("DEBUG TELEMETRY: ");
    Serial.println(message.c_str());
    Serial.flush();
  }
}


inline void printData(uint8_t* buffer, size_t size) {
    Serial.print("DATA: ");
    for (size_t i = 0; i < size; i++) {
        if (buffer[i] < 16) Serial.print('0'); // pad single-digit hex
        Serial.print(buffer[i], HEX);
    }
    Serial.println();
    Serial.flush();
}

#endif