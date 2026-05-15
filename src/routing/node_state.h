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
    float pressure_norm;
    float flame_norm;
    float smoke_norm;
    float air_norm;
    float heat_norm;
    float still_norm;
    bool blocked;
    bool alive;
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
    uint8_t direction; // neighbor node_id, or DIRECTION_SELF/NONE

    bool sos_active;
    uint8_t sos_relays[MAX_NEIGHBORS];
    uint8_t sos_relay_count;
    float still_timer;

    SensorReadings sensors;
    NeighborTable neighbors;
    uint32_t last_broadcast;

    NodeState(uint8_t id, uint8_t floor, float x, float y, bool exit);

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