#include "tof.h"
#include "config.h"
#include <Wire.h>
#include <VL53L1X.h>

// VL53L1X operates in mm. We convert to meters.
// If the sensor returns 0 or times out, we treat the path as blocked —
// "unknown" is safer than "clear" in an evacuation context.
// I2C is shared with OLED (both on SDA=21, SCL=22).
// VL53L1X default I2C address is 0x29 — no conflict with OLED at 0x3C.

static VL53L1X sensor;
static bool _initialized = false;

void tofInit()
{
    // Wire.begin() is called once in main setup() before tofInit().
    // We don't call it again here to avoid double-init with OLED.
    sensor.setTimeout(500);

    if (!sensor.init())
    {
        // Sensor not found — mark as failed, tofForwardBlocked() returns true
        _initialized = false;
        return;
    }

    _initialized = true;
    sensor.setDistanceMode(VL53L1X::Long);    // up to ~4m, suits corridors
    sensor.setMeasurementTimingBudget(50000); // 50ms — fast enough for 100ms tick
    sensor.startContinuous(50);               // reading every 50ms
}

float tofGetDistance()
{
    if (!_initialized)
        return -1.0f;

    // Non-blocking read — returns last completed measurement
    uint16_t mm = sensor.read(false);

    if (sensor.timeoutOccurred() || mm == 0)
        return -1.0f;

    return mm / 1000.0f; // mm → meters
}

bool tofForwardBlocked()
{
    float d = tofGetDistance();
    if (d < 0.0f)
        return true; // no reading = treat as blocked
    return d < D_MIN;
}