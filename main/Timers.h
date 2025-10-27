#ifndef TIMERS_H
#define TIMERS_H

void configureArduino();

bool isControlFlag();
bool isTelemetryFlag();
bool isSampleFlag();

bool resetControlFlag();
bool resetTelemetryFlag();
bool resetSampleFlag();

#endif