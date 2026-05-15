#pragma once
#include <Arduino.h>
#include "comms/packets.h"

struct NeighborEntry
{
    uint8_t node_id;
    uint8_t floor_id;
    float x, y;

    // Updated by NodePacket
    Mood mood;
    float best_exit_cost;
    NeighborStatus status;
    bool sos_flag;
    uint8_t sos_origin_id;
    uint32_t last_seen;
    uint32_t timestamp;

    // Populated from HelloPacket — never changes
    bool hello_received;
};

class NeighborTable
{
public:
    static const uint8_t MAX = 10;
    NeighborEntry entries[MAX];
    uint8_t count = 0;

    void addFromHello(const HelloPacket &pkt);
    void updateFromPacket(const NodePacket &pkt, uint32_t now);
    NeighborEntry *find(uint8_t node_id);
    bool exists(uint8_t node_id) const;
    void checkStaleness(uint32_t now);
};