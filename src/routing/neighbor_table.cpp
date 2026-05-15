#include "neighbor_table.h"
#include "config.h"
#include <Arduino.h>

// ─── Internal helpers ────────────────────────────────────────────────────────

// Staleness is relative to the neighbor's own broadcast interval.
// A WARN node broadcasts every 500ms, so 3× = 1500ms before we mark NSIG.
// This means a quiet STBY node (2000ms interval) gets 6s grace — appropriate.
static uint32_t intervalForMood(Mood m)
{
    switch (m)
    {
    case Mood::WARN:
        return INTERVAL_WARN;
    case Mood::ALERT:
        return INTERVAL_ALERT;
    case Mood::EVAC:
        return INTERVAL_EVAC;
    default:
        return INTERVAL_STBY;
    }
}

// ─── Public methods ──────────────────────────────────────────────────────────

void NeighborTable::addFromHello(const HelloPacket &pkt)
{
    // If already exists, just update position (shouldn't change, but be safe)
    NeighborEntry *existing = find(pkt.node_id);
    if (existing)
    {
        existing->x = pkt.x;
        existing->y = pkt.y;
        existing->floor_id = pkt.floor_id;
        existing->hello_received = true;
        return;
    }

    // No room — table full, silently drop
    if (count >= MAX)
        return;

    NeighborEntry &e = entries[count++];
    e.node_id = pkt.node_id;
    e.floor_id = pkt.floor_id;
    e.x = pkt.x;
    e.y = pkt.y;
    e.hello_received = true;

    // Routing fields — safe defaults until first NodePacket arrives
    e.mood = Mood::STBY;
    e.best_exit_cost = 99999.0f;     // INF_COST — don't use this neighbor yet
    e.status = NeighborStatus::N_SIG; // not confirmed until packet received
    e.sos_flag = false;
    e.sos_origin_id = 0;
    e.last_seen = 0;
    e.timestamp = 0;
}

void NeighborTable::updateFromPacket(const NodePacket &pkt, uint32_t now)
{
    NeighborEntry *e = find(pkt.node_id);

    // Packet from unknown node — only add if we got a HELLO first.
    // Without x,y we can't compute direction, so we skip.
    if (!e)
        return;
    if (!e->hello_received)
        return;

    e->mood = static_cast<Mood>(pkt.mood);
    e->best_exit_cost = pkt.best_exit_cost;
    e->sos_flag = pkt.sos_flag;
    e->sos_origin_id = pkt.sos_origin_id;
    e->last_seen = now;
    e->timestamp = pkt.timestamp;

    // Map mood to NeighborStatus
    switch (e->mood)
    {
    case Mood::STBY:
        e->status = NeighborStatus::OK;
        break;
    case Mood::WARN:
        e->status = NeighborStatus::WARN;
        break;
    case Mood::ALERT:
        e->status = NeighborStatus::ALERT;
        break;
    case Mood::EVAC:
        e->status = NeighborStatus::EVAC;
        break;
    default:
        e->status = NeighborStatus::OK;
        break;
        // RESCUE maps to OK for routing — it's a separate overlay, not a cost state
    }
}

NeighborEntry *NeighborTable::find(uint8_t node_id)
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (entries[i].node_id == node_id)
            return &entries[i];
    }
    return nullptr;
}

bool NeighborTable::exists(uint8_t node_id) const
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (entries[i].node_id == node_id)
            return true;
    }
    return false;
}

void NeighborTable::checkStaleness(uint32_t now)
{
    for (uint8_t i = 0; i < count; i++)
    {
        NeighborEntry &e = entries[i];

        // Never confirmed by a real packet yet — always NSIG
        if (e.last_seen == 0)
        {
            e.status = NeighborStatus::N_SIG;
            e.best_exit_cost = 99999.0f;
            continue;
        }

        uint32_t age = now - e.last_seen;
        uint32_t deadline = STALENESS_MULT * intervalForMood(e.mood);

        if (age > deadline)
        {
            e.status = NeighborStatus::N_SIG;
            e.best_exit_cost = 99999.0f;
            // mood and position retained — SOS relay logic needs to survive NSIG
        }
    }
}