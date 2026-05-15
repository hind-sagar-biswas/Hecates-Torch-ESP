#pragma once
#include <Arduino.h>

#define MAX_NEIGHBORS 10

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

// HelloPacket — broadcast once at boot
struct __attribute__((packed)) HelloPacket
{
    uint8_t node_id;
    uint8_t floor_id;
    float x;
    float y;
    uint8_t type; // 0 = HELLO identifier
};

// NodePacket — regular operational broadcast
struct __attribute__((packed)) NodePacket
{
    uint8_t node_id;
    uint8_t floor_id;
    uint8_t mood; // cast from Mood enum
    float best_exit_cost;
    bool sos_flag;
    uint8_t sos_origin_id;
    uint32_t timestamp;
    uint8_t type; // 1 = NODE identifier
};