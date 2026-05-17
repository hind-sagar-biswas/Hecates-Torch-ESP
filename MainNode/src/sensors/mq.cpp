#include "mq.h"
#include "config.h"
#include <Arduino.h>

// MQ sensors need ~30s warm-up after power-on.
// We do a blocking warm-up in mqInit() for demo reliability.
// In production you'd do this non-blocking, but for a demo
// having the node stall 30s at boot is acceptable.

static const uint32_t WARMUP_MS = 30000;

void mqInit()
{
    pinMode(MQ2_PIN, INPUT);
    pinMode(MQ135_PIN, INPUT);
    delay(WARMUP_MS);
}

float mqGetSmokeNorm()
{
    int raw = analogRead(MQ2_PIN);
    return constrain((float)raw / MQ2_MAX, 0.0f, 1.0f);
}

float mqGetAirNorm()
{
    int raw = analogRead(MQ135_PIN);
    return constrain((float)raw / MQ135_MAX, 0.0f, 1.0f);
}