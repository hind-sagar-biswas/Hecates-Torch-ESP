#pragma once
#include <Arduino.h>
#include "comms/packets.h"
#include "neighbor_table.h"
#include "config.h"

#define INF_COST 99999.0f
#define DIRECTION_SELF 255
#define DIRECTION_NONE 254

struct SensorReadings
{
    float pressure_norm = 0.0f;
    float flame_norm = 0.0f;
    float smoke_norm = 0.0f;
    float air_norm = 0.0f;
    float heat_norm = 0.0f;
    float still_norm = 0.0f;
    float delta_temp_norm = 0.0f; // rate of temperature rise, normalized [0,1]
    bool blocked = false;
    bool alive = true;
};

class NodeState
{
public:
    uint8_t node_id;
    uint8_t floor_id;
    float x, y;
    bool is_exit;

    Mood mood;
    float local_cost;
    float best_exit_cost;
    uint8_t direction; // neighbor node_id, DIRECTION_SELF, or DIRECTION_NONE

    bool sos_active;
    uint8_t sos_relays[MAX_NEIGHBORS];
    uint8_t sos_relay_count;
    float still_timer;

    float p_trapped = 0.0f;
    float p_history[5] = {0};
    uint8_t p_idx = 0;
    uint8_t sos_reason = 0; // 0=trapped, 1=fire_blocked, 2=toxic

    SensorReadings sensors;
    NeighborTable neighbors;
    uint32_t last_broadcast;

    // Derived sensor values — computed once per tick by updateDerivedSensors()
    // Used by both updateLocalCost() and updateMood() — never recomputed twice
    float cached_fire_value;
    float cached_h_air;

    NodeState(uint8_t id, uint8_t floor, float x, float y, bool exit);

    // Call first every tick — computes cached_fire_value and cached_h_air
    void updateDerivedSensors();

    void updateLocalCost();
    void updateMood();
    void updateNeighbors(uint32_t now);
    void updateRouting();
    void updateRescue(float dt);

    NodePacket buildPacket(uint32_t now);
    void receivePacket(const NodePacket &pkt, uint32_t now);

    HelloPacket buildHello();
    void receiveHello(const HelloPacket &pkt);

    uint32_t broadcastInterval() const;
    bool broadcastDue(uint32_t now) const;

private:
    bool hasSosRelay(uint8_t origin_id) const;
    void addSosRelay(uint8_t origin_id);
};