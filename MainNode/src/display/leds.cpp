#include "leds.h"
#include "config.h"
#include <Arduino.h>

// Four discrete LEDs — not RGB. Each is wired to its own pin.
// GREEN=STBY, YELLOW=WARN, RED=ALERT, RED blink=EVAC, BLUE=RESCUE.
// Only one status LED is on at a time. Blue is fully independent — it
// can be on simultaneously with any status LED (e.g. EVAC + RESCUE).
//
// EVAC blink: 200ms on / 200ms off. We use millis() here so the blink
// is non-blocking — never delay() in an ISR-adjacent context.

static const uint32_t BLINK_PERIOD = 400; // full cycle ms
static const uint32_t BLINK_ON = 200;     // on portion ms

void ledsInit()
{
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_YELLOW_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);

    // All off at boot
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
}

void ledsUpdate(Mood mood, bool sos_active)
{
    // ── Status LED (mutually exclusive) ─────────────────────────────────────
    bool green = false;
    bool yellow = false;
    bool red = false;

    switch (mood)
    {
    case Mood::STBY:
        green = true;
        break;

    case Mood::WARN:
        yellow = true;
        break;

    case Mood::ALERT:
        red = true;
        break;

    case Mood::EVAC:
    {
        // Non-blocking blink on red
        uint32_t phase = millis() % BLINK_PERIOD;
        red = (phase < BLINK_ON);
        break;
    }

    case Mood::RESCUE:
        // RESCUE is an overlay — underlying mood LED still shows.
        // But RESCUE alone (without another mood) defaults to STBY green.
        green = true;
        break;
    }

    digitalWrite(LED_GREEN_PIN, green ? HIGH : LOW);
    digitalWrite(LED_YELLOW_PIN, yellow ? HIGH : LOW);
    digitalWrite(LED_RED_PIN, red ? HIGH : LOW);

    // ── Blue LED — fully independent ─────────────────────────────────────────
    digitalWrite(LED_BLUE_PIN, sos_active ? HIGH : LOW);
}