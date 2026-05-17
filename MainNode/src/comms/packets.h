#pragma once
#include <Arduino.h>

#define MAX_NEIGHBORS 10
#define PACKET_VERSION 1

enum class Mood : uint8_t
{
    STBY = 0,
    WARN = 1,
    ALERT = 2,
    EVAC = 3,
    RESCUE = 4
};
enum class NeighborStatus : uint8_t
{
    OK = 0,
    WARN = 1,
    ALERT = 2,
    EVAC = 3,
    N_SIG = 4
};

struct __attribute__((packed)) HelloPacket
{
    uint8_t version; // always PACKET_VERSION
    uint8_t type;    // always 0
    uint8_t node_id;
    uint8_t floor_id;
    float x;
    float y;
};

struct __attribute__((packed)) NodePacket
{
    uint8_t version; // always PACKET_VERSION
    uint8_t type;    // always 1
    uint8_t node_id;
    uint8_t floor_id;
    uint8_t mood;
    float best_exit_cost;
    uint8_t sos_flag; // FIX: was bool
    uint8_t sos_origin_id;
    uint32_t timestamp;
};

// Compile-time size validation — catches accidental layout changes
static_assert(sizeof(HelloPacket) == 12, "HelloPacket size mismatch");
static_assert(sizeof(NodePacket) == 15, "NodePacket size mismatch");