// temperature.h
#pragma once
#include <Arduino.h>

void tempInit(int pin);
float tempGetCelsius();
float tempGetHeatNorm();
float tempGetDeltaNorm();
