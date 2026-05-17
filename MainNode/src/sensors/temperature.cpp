// temperature.cpp
#include "temperature.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "config.h"

// ─── Static instances ─────────────────────────────────────────────────────────
// Pointers used so we can initialize with the correct pin at runtime
static OneWire *_ow = nullptr;
static DallasTemperature *_sensor = nullptr;

// ─── Read cache ───────────────────────────────────────────────────────────────
// DS18B20 blocks ~750ms per read — cache at 1Hz to avoid stalling the loop
static float _cached_temp = 25.0f;
static uint32_t _last_read = 0;
#define TEMP_READ_INTERVAL_MS 1000

// ─── Internal cached read ─────────────────────────────────────────────────────
static float _readTemp()
{
    uint32_t now = millis();

    // Return cached value if read too recently
    if (_sensor == nullptr)
        return _cached_temp;
    if ((now - _last_read) < TEMP_READ_INTERVAL_MS)
        return _cached_temp;

    _sensor->requestTemperatures();
    float t = _sensor->getTempCByIndex(0);

    if (t == DEVICE_DISCONNECTED_C)
    {
        Serial.println("Temp: DS18B20 disconnected — using cached value");
    }
    else
    {
        _cached_temp = t;
    }

    _last_read = now;
    return _cached_temp;
}

// ─── Public API ───────────────────────────────────────────────────────────────

void tempInit(int pin)
{
    _ow = new OneWire(pin);
    _sensor = new DallasTemperature(_ow);

    _sensor->begin();

    // Warm up — do one blocking read at boot so cache is valid immediately
    _sensor->requestTemperatures();
    float t = _sensor->getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C)
    {
        _cached_temp = t;
    }
    _last_read = millis();

    Serial.print("Temp: DS18B20 ready — initial reading: ");
    Serial.print(_cached_temp);
    Serial.println(" °C");
}

// Raw celsius — uses cache
float tempGetCelsius()
{
    return _readTemp();
}

// Heat penalty normalized to [0,1]
// 0 at 40°C, 1 at 55°C — matches HEAT_BASE and HEAT_RANGE in config.h
float tempGetHeatNorm()
{
    return constrain((_readTemp() - HEAT_BASE) / HEAT_RANGE, 0.0f, 1.0f);
}

// Rate of temperature rise normalized to [0,1]
// 0 = no rise, 1 = 5°C/s or faster (fire-level rise)
// Negative delta (cooling) clamped to 0 — only rising temps matter
// Uses cached read — no extra blocking
float tempGetDeltaNorm()
{
    static float prev_temp = 25.0f;
    static uint32_t prev_time = 0;

    float now_temp = _readTemp();
    uint32_t now = millis();
    float dt_sec = (now - prev_time) / 1000.0f;

    // Only compute after enough time has elapsed for a meaningful delta
    // Also avoids division by near-zero dt
    if (dt_sec < 0.5f)
        return 0.0f;

    float dT = now_temp - prev_temp;
    float rate = dT / dt_sec;                        // °C per second
    float norm = constrain(rate / 5.0f, 0.0f, 1.0f); // 5°C/s = max

    prev_temp = now_temp;
    prev_time = now;

    return norm;
}