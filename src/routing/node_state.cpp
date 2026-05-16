#include "node_state.h"
#include <math.h>

NodeState::NodeState(uint8_t id, uint8_t floor, float px, float py, bool exit)
    : node_id(id), floor_id(floor), x(px), y(py), is_exit(exit),
      mood(Mood::STBY),
      local_cost(BASE_DISTANCE),
      best_exit_cost(exit ? BASE_DISTANCE : INF_COST),
      direction(exit ? DIRECTION_SELF : DIRECTION_NONE),
      sos_active(false), sos_relay_count(0),
      still_timer(0), last_broadcast(0),
      cached_fire_value(0.0f), cached_h_air(0.0f)
{
    // SensorReadings uses in-class default initializers — no manual zeroing needed
    memset(sos_relays, 0, sizeof(sos_relays));
    last_broadcast = millis();
}

// ─── Derived sensor fusion ────────────────────────────────────────────────────
// Called once per tick before updateLocalCost() and updateMood()
// Prevents computing fire_value and h_air twice per tick

void NodeState::updateDerivedSensors()
{
    // Fused fire hazard — flame analog + rate of temp rise + smoke confirmation
    cached_fire_value = constrain(W1 * sensors.flame_norm + W2 * sensors.delta_temp_norm + W3 * sensors.smoke_norm, 0.0f, 1.0f);

    // Weighted air quality index — smoke weighted higher (immediate visibility)
    // Normalized so result stays in [0,1]
    cached_h_air = (ETA_TOX * sensors.air_norm + BETA_SMOKE * sensors.smoke_norm) / (ETA_TOX + BETA_SMOKE);
}

// ─── Local cost ───────────────────────────────────────────────────────────────

void NodeState::updateLocalCost()
{
    // Output sensors values
    // Serial.printf("Node %d sensors — Pressure: %.2f, Fire: %.2f, H_air: %.2f, Heat: %.2f\n",
    //               node_id, sensors.pressure_norm, cached_fire_value, cached_h_air, sensors.heat_norm);
    // Output Blockage status
    // Serial.printf("Node %d blockage status: %s\n", node_id, sensors.blocked ? "BLOCKED" : "CLEAR");
    if (!sensors.alive || sensors.blocked)
    {
        local_cost = INF_COST;
        return;
    }

    local_cost = BASE_DISTANCE + WEIGHT_ALPHA * sensors.pressure_norm + WEIGHT_BETA * cached_fire_value + WEIGHT_ETA * cached_h_air + WEIGHT_KAPPA * sensors.heat_norm;
}

// ─── Mood elevation ───────────────────────────────────────────────────────────

void NodeState::updateMood()
{
    if (local_cost >= INF_COST)
    {
        mood = Mood::EVAC;
        return;
    }

    if (cached_fire_value > FLAME_ALERT_THR || cached_h_air > SMOKE_ALERT_THR)
    {
        mood = Mood::ALERT;
        return;
    }

    if (sensors.pressure_norm > PRESSURE_WARN_THR || cached_h_air > SMOKE_WARN_THR || sensors.air_norm > AIR_WARN_THR)
    {
        mood = Mood::WARN;
        return;
    }

    mood = Mood::STBY;
}

// ─── Neighbor staleness ───────────────────────────────────────────────────────

void NodeState::updateNeighbors(uint32_t now)
{
    neighbors.checkStaleness(now);
}

// ─── Bellman-Ford routing step ────────────────────────────────────────────────

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
        if (n.status == NeighborStatus::N_SIG || n.status == NeighborStatus::EVAC)
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

// ─── RESCUE / SOS detection ───────────────────────────────────────────────────
// ─── Internal helpers ────────────────────────────────────────────────────────

static float smoothProb(float *history, uint8_t &idx, float newVal)
{
    history[idx % 5] = newVal;
    idx++;
    float sum = 0;
    for (int i = 0; i < 5; i++)
        sum += history[i];
    return sum / 5.0f;
}

// ─── Main rescue update ──────────────────────────────────────────────────────
void NodeState::updateRescue(float dt)
{

    bool trapped_signature = sensors.still_norm > 0.35f && sensors.pressure_norm < 0.20f && sensors.still_norm > (sensors.pressure_norm * 1.5f);

    if (trapped_signature)
    {
        still_timer += dt;

        // Human presence probability — radar only
        // These are the only signals reliable through smoke and fire
        float p_still = sensors.still_norm;
        float p_isolated = 1.0f - constrain(sensors.pressure_norm * 3.0f, 0.0f, 1.0f);

        float p_raw;
        bool fire_confirmed = cached_fire_value > 0.5f;

        if (fire_confirmed)
        {
            // Fire: only radar trusted, thermal evidence is meaningless
            p_raw = (p_still * 0.70f) +
                    (p_isolated * 0.30f);
        }
        else
        {
            // No fire: weak body heat signal from DS18B20 is usable
            float p_body_heat = 0.0f;
            if (sensors.heat_norm > 0.0f && sensors.heat_norm < 0.25f)
                p_body_heat = sensors.heat_norm / 0.25f;

            p_raw = (p_still * 0.60f) + (p_isolated * 0.25f) + (p_body_heat * 0.15f);
        }

        p_trapped = smoothProb(p_history, p_idx, p_raw);

        // Confirm time: dangerous environment shortens wait
        // not because we're more certain, but because we can't afford to wait
        float confirm =
            (mood == Mood::EVAC) ? 3.0f : (mood == Mood::ALERT) ? 5.0f
                                      : (mood == Mood::WARN)    ? 15.0f
                                                                : 20.0f;

        if (still_timer > confirm && p_trapped > 0.65f)
        {
            sos_active = true;

            // Reason tells responders what they're walking into
            // not who or how many
            if (cached_fire_value > 0.7f)
                sos_reason = 1; // FIRE_BLOCKED
            else if (sensors.air_norm > 0.6f && sensors.smoke_norm > 0.5f)
                sos_reason = 2; // TOXIC
            else
                sos_reason = 0; // TRAPPED
        }
    }
    else
    {
        still_timer = 0.0f;
        p_trapped = smoothProb(p_history, p_idx, 0.0f);

        // Hysteresis — don't kill SOS on a single missed reading
        if (p_trapped < 0.20f)
        {
            sos_active = false;
            sos_reason = 0;
        }
    }
}

// ─── Packet building ──────────────────────────────────────────────────────────

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

    if (pkt.sos_flag && pkt.sos_origin_id != node_id)
    {
        addSosRelay(pkt.sos_origin_id);
    }
}

// ─── HELLO ────────────────────────────────────────────────────────────────────

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

    float dx = x - pkt.x;
    float dy = y - pkt.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist > 3.5f)
        return; // NEIGHBOR_RANGE — matches values.js NEIGHBOR_RANGE

    neighbors.addFromHello(pkt);
}

// ─── Timing ───────────────────────────────────────────────────────────────────

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

// ─── SOS relay helpers ────────────────────────────────────────────────────────

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