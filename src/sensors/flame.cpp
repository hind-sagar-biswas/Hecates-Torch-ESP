#include "flame.h"
#include "config.h"
#include <Arduino.h>

// Flame sensor analog: lower raw value = more flame (inverse).
// The module outputs lower voltage as IR intensity increases.
// So we invert: norm = 1 - (raw / 4095)
// This way flame_norm = 1.0 means strong flame, 0.0 means none.
//
// Digital pin: LOW = flame detected (active low on most modules).
// We return true if flame detected (normalized for the rest of the system).

void flameInit()
{
    pinMode(FLAME_ANA_PIN, INPUT);
    pinMode(FLAME_DIG_PIN, INPUT);
}

float flameGetNorm()
{
    int raw = analogRead(FLAME_ANA_PIN); // 0–4095
    float norm = 1.0f - (raw / 4095.0f); // invert
    return constrain(norm, 0.0f, 1.0f);
}

bool flameGetDigital()
{
    return digitalRead(FLAME_DIG_PIN) == LOW; // active low
}