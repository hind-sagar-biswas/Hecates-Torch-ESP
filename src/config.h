#pragma once

// ─── Node Identity ────────────────────────────────────────────────────────────
#define NODE_ID 1
#define FLOOR_ID 0
#define IS_EXIT false

// ─── Pin Definitions ──────────────────────────────────────────────────────────

// LD2410C Radar (UART2)
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17

// VL53L1X ToF (I2C)
#define TOF_SDA_PIN 21
#define TOF_SCL_PIN 22

// OLED (I2C — shared bus with ToF)
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22
#define OLED_ADDR 0x3C

// DS18B20 Temperature (OneWire)
#define TEMP_PIN 4

// MQ Sensors (Analog)
#define MQ2_PIN 34
#define MQ135_PIN 35

// Flame Sensor
#define FLAME_ANA_PIN 26
#define FLAME_DIG_PIN 27

// LEDs
#define LED_GREEN_PIN 18
#define LED_YELLOW_PIN 19
#define LED_RED_PIN 23
#define LED_BLUE_PIN 5

// Demo knobs (potentiometers)
#define KNOB_PRESSURE_PIN 36
#define KNOB_FIRE_PIN 39
#define KNOB_AIR_PIN 34 // shared with MQ2 on Node 0 (sim node)

// Demo buttons
#define BTN_ALIVE_PIN 18
#define BTN_BLOCKED_PIN 19
#define BTN_EXIT_PIN 23 // button 3 — toggle exit node status

// ─── Algorithm Constants ──────────────────────────────────────────────────────
#define BASE_DISTANCE 1.0f
#define WEIGHT_ALPHA 2.0f // pressure
#define WEIGHT_BETA 3.0f  // fire
#define WEIGHT_ETA 2.5f   // smoke
#define WEIGHT_KAPPA 1.5f // heat

#define D_MIN 0.5f  // meters — blocked if forward < this
#define D_SAFE 0.3f // meters — hole if ground > this

#define MQ2_MAX 1000.0f
#define MQ135_MAX 1000.0f
#define HEAT_BASE 40.0f
#define HEAT_RANGE 15.0f

#define FLAME_ALERT_THR 500
#define SMOKE_ALERT_THR 700
#define AIR_ALERT_THR 700
#define PRESSURE_WARN_THR 0.6f
#define SMOKE_WARN_THR 0.5f
#define AIR_WARN_THR 0.5f

// ─── Timing ───────────────────────────────────────────────────────────────────
#define INTERVAL_STBY 2000
#define INTERVAL_WARN 500
#define INTERVAL_ALERT 200
#define INTERVAL_EVAC 100
#define STALENESS_MULT 3