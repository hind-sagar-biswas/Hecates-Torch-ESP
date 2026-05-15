#include "node_state.h"
#include <math.h>

NodeState::NodeState(uint8_t id, uint8_t floor, float px, float py, bool exit)
    : node_id(id), floor_id(floor), x(px), y(py), is_exit(exit),
      mood(Mood::STBY), local_cost(BASE_DISTANCE),
      best_exit_cost(exit ? BASE_DISTANCE : INF_COST),
      direction(exit ? DIRECTION_SELF : DIRECTION_NONE),
      sos_active(false), sos_relay_count(0),
      still_timer(0), last_broadcast(0)
{
    sensors = {0, 0, 0, 0, 0, 0, false, true};
    memset(sos_relays, 0, sizeof(sos_relays));
}

// TODO: UPDATE THE COST CALCULATION TO MATCH THE PAPER — CURRENTLY SIMPLIFIED FOR DEMO PURPOSES
void NodeState::updateLocalCost()
{
    if (!sensors.alive || sensors.blocked)
    {
        local_cost = INF_COST;
        return;
    }
    local_cost = BASE_DISTANCE + WEIGHT_ALPHA * sensors.pressure_norm + WEIGHT_BETA * sensors.flame_norm + WEIGHT_ETA * sensors.smoke_norm + WEIGHT_KAPPA * sensors.heat_norm;
}

void NodeState::updateMood()
{
    if (local_cost >= INF_COST)
    {
        mood = Mood::EVAC;
        return;
    }

    if (sensors.flame_norm > (FLAME_ALERT_THR / 1000.0f) ||
        sensors.smoke_norm > (SMOKE_ALERT_THR / MQ2_MAX) ||
        sensors.air_norm > (AIR_ALERT_THR / MQ135_MAX))
    {
        mood = Mood::ALERT;
        return;
    }

    if (sensors.pressure_norm > PRESSURE_WARN_THR ||
        sensors.smoke_norm > SMOKE_WARN_THR ||
        sensors.air_norm > AIR_WARN_THR)
    {
        mood = Mood::WARN;
        return;
    }

    mood = Mood::STBY;
}

void NodeState::updateNeighbors(uint32_t now)
{
    neighbors.checkStaleness(now);
}

void NodeState::updateRouting()
{
    if (is_exit)
    {
        best_exit_cost = local_cost;
        direction = DIRECTION_SELF;
        return;
    }

    float best = INF_COST;
    uint8_t best_id = DIRECTION_NONE;

    for (uint8_t i = 0; i < neighbors.count; i++)
    {
        NeighborEntry &n = neighbors.entries[i];
        if (n.status == NeighborStatus::N_SIG ||
            n.status == NeighborStatus::EVAC)
            continue;

        float candidate = local_cost + n.best_exit_cost;
        if (candidate < best)
        {
            best = candidate;
            best_id = n.node_id;
        }
    }

    best_exit_cost = best;
    direction = best_id;

    if (best >= INF_COST && mood != Mood::EVAC)
    {
        mood = Mood::EVAC;
    }
}

void NodeState::updateRescue(float dt)
{
    bool trapped = sensors.still_norm > 0.7f && sensors.pressure_norm < 0.2f;

    if (trapped)
    {
        still_timer += dt;
        // T_CONFIRM varies by mood — simplified for firmware
        float confirm = (mood == Mood::EVAC) ? 3.0f : (mood == Mood::ALERT) ? 5.0f
                                                  : (mood == Mood::WARN)    ? 15.0f
                                                                            : 20.0f;
        if (still_timer > confirm && sensors.still_norm > 0.7f)
        {
            sos_active = true;
        }
    }
    else
    {
        still_timer = 0;
        sos_active = false;
    }
}

NodePacket NodeState::buildPacket(uint32_t now)
{
    last_broadcast = now;
    NodePacket pkt;
    pkt.node_id = node_id;
    pkt.floor_id = floor_id;
    pkt.mood = (uint8_t)mood;
    pkt.best_exit_cost = best_exit_cost;
    pkt.sos_flag = sos_active;
    pkt.sos_origin_id = sos_active ? node_id : 0;
    pkt.timestamp = now;
    pkt.type = 1;
    return pkt;
}

void NodeState::receivePacket(const NodePacket &pkt, uint32_t now)
{
    if (pkt.node_id == node_id)
        return;
    neighbors.updateFromPacket(pkt, now);

    // SOS relay
    if (pkt.sos_flag && pkt.sos_origin_id != node_id)
    {
        addSosRelay(pkt.sos_origin_id);
    }
}

HelloPacket NodeState::buildHello()
{
    HelloPacket pkt;
    pkt.node_id = node_id;
    pkt.floor_id = floor_id;
    pkt.x = x;
    pkt.y = y;
    pkt.type = 0;
    return pkt;
}

void NodeState::receiveHello(const HelloPacket &pkt)
{
    if (pkt.node_id == node_id)
        return;

    // Distance gate — NEIGHBOR_RANGE check
    float dx = x - pkt.x;
    float dy = y - pkt.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist > 3.5f)
        return; // NEIGHBOR_RANGE

    neighbors.addFromHello(pkt);
}

uint32_t NodeState::broadcastInterval() const
{
    switch (mood)
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

bool NodeState::broadcastDue(uint32_t now) const
{
    return (now - last_broadcast) >= broadcastInterval();
}

bool NodeState::hasSosRelay(uint8_t origin_id) const
{
    for (uint8_t i = 0; i < sos_relay_count; i++)
        if (sos_relays[i] == origin_id)
            return true;
    return false;
}

void NodeState::addSosRelay(uint8_t origin_id)
{
    if (hasSosRelay(origin_id))
        return;
    if (sos_relay_count < MAX_NEIGHBORS)
        sos_relays[sos_relay_count++] = origin_id;
}