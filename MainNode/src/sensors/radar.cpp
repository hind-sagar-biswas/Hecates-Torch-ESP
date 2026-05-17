#include "radar.h"
#include <MyLD2410.h>
#include <HardwareSerial.h>

static HardwareSerial _serial(2);
static MyLD2410 sensor(_serial);

static float _pressure = 0.0f;
static float _stillNorm = 0.0f;
static bool _detected = false;

#define PRESSURE_WINDOW 5
static float _history[PRESSURE_WINDOW] = {0};
static int _histIdx = 0;

static float movingAvg(float v)
{
    _history[_histIdx % PRESSURE_WINDOW] = v;
    _histIdx++;
    float s = 0;
    for (int i = 0; i < PRESSURE_WINDOW; i++)
        s += _history[i];
    return s / PRESSURE_WINDOW;
}

void radarInit(int rx_pin, int tx_pin)
{
    _serial.begin(256000, SERIAL_8N1, rx_pin, tx_pin);
    delay(500);

    if (!sensor.begin())
    {
        Serial.println("Radar: LD2410C not responding");
        return;
    }

    // Enable enhanced mode to get per-gate signal arrays
    if (!sensor.enhancedMode())
    {
        Serial.println("Radar: enhanced mode failed — falling back to basic");
    }

    Serial.println("Radar: LD2410C ready");
}

void radarRead()
{
    if (!sensor.check())
        return; // no new frame

    _detected = sensor.presenceDetected();

    if (!_detected)
    {
        _pressure = movingAvg(0.0f);
        _stillNorm = 0.0f;
        return;
    }

    // ── Per-gate pressure index (enhanced mode) ────────────────────────────
    if (sensor.inEnhancedMode())
    {
        const MyLD2410::ValuesArray &moveSignals = sensor.getMovingSignals();
        const MyLD2410::ValuesArray &stillSignals = sensor.getStationarySignals();

        int activeGates = 0;
        float totalMove = 0;
        float totalStill = 0;
        float maxStill = 0;
        int N = moveSignals.N; // number of gates (typically 9)

        for (int g = 0; g < N; g++)
        {
            float mv = moveSignals.values[g]; // .values[] not []
            float st = stillSignals.values[g];

            totalMove += mv;
            totalStill += st;
            if (mv > 10 || st > 10)
                activeGates++;
            if (st > maxStill)
                maxStill = st;
        }

        // Density — how much of the detection space is occupied
        float density = (float)activeGates / N;

        // Mean energy across active gates — how intense the occupancy is
        float meanEnergy = (activeGates > 0)
                               ? (totalMove + totalStill) / (activeGates * 2.0f * 100.0f)
                               : 0.0f;

        // Stagnation — high still relative to move = crowd not flowing
        float totalCombined = totalMove + totalStill;
        float stagnation = (totalCombined > 0)
                               ? totalStill / totalCombined
                               : 0.0f;

        float rawPressure = constrain(
            density * 0.4f + meanEnergy * 0.3f + stagnation * 0.3f,
            0.0f, 1.0f);

        _pressure = movingAvg(rawPressure);
        _stillNorm = constrain(maxStill / 100.0f, 0.0f, 1.0f);
    }
    else
    {
        // ── Fallback: basic mode — aggregate signals only ──────────────────
        float moveAgg = sensor.movingTargetSignal() / 100.0f;
        float stillAgg = sensor.stationaryTargetSignal() / 100.0f;

        float stagnation = (moveAgg + stillAgg > 0)
                               ? stillAgg / (moveAgg + stillAgg)
                               : 0.0f;

        float rawPressure = constrain(
            stillAgg * 0.5f + stagnation * 0.3f + moveAgg * 0.2f,
            0.0f, 1.0f);

        _pressure = movingAvg(rawPressure);
        _stillNorm = stillAgg;
    }
}

float radarGetPressure() { return _pressure; }
float radarGetStillNorm() { return _stillNorm; }
bool radarPresenceDetected() { return _detected; }