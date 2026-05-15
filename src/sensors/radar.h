#pragma once
#include <Arduino.h>

void radarInit(int rx_pin, int tx_pin);
void radarRead();
float radarGetPressure();
float radarGetStillNorm();
bool radarPresenceDetected();