#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

#include "config.h"
#include "comms/packets.h"
#include "comms/espnow.h"
#include "routing/node_state.h"
// #include "display/oled.h"
#include "display/leds.h"

enum class BootPhase
{
  HELLO,
  OPERATIONAL
};

// ─── Global state ─────────────────────────────────────────────────────────────

static Preferences prefs;
static NodeState *node = nullptr;
static bool nodeAlive = true;

static BootPhase bootPhase = BootPhase::HELLO;
static uint32_t helloPhaseStart = 0;
static uint32_t lastHelloBroadcast = 0;

// ─── Button state ─────────────────────────────────────────────────────────────

static bool lastAliveBtn = false;
static bool lastBlockedBtn = false;
static bool btnBlocked = false;

// ─── ESP-NOW receive callback ─────────────────────────────────────────────────

void onPacketReceived(const uint8_t *data, int len, uint32_t now)
{
  // FIX: dispatch on FIRST byte (type), not last
  // FIX: check minimum length before touching data
  if (len < 2)
    return;

  uint8_t version = data[0];
  uint8_t type = data[1];

  if (version != PACKET_VERSION)
    return;

  if (type == 0 && len >= (int)sizeof(HelloPacket))
  {
    // Accept HELLO packets in BOTH phases —
    // a node that reboots mid-session needs to re-announce itself
    HelloPacket pkt;
    memcpy(&pkt, data, sizeof(HelloPacket));
    node->receiveHello(pkt); // neighbor_table deduplicates by node_id
  }
  else if (type == 1 && len >= (int)sizeof(NodePacket))
  {
    // NodePackets only meaningful in OPERATIONAL phase
    // During HELLO phase we don't have costs yet — ignore
    if (bootPhase == BootPhase::OPERATIONAL)
    {
      NodePacket pkt;
      memcpy(&pkt, data, sizeof(NodePacket));
      node->receivePacket(pkt, now);
    }
  }
}

// ─── Exit toggle ──────────────────────────────────────────────────────────────

void checkExitToggle(uint32_t now)
{
  static bool lastState = false;
  static uint32_t lastMs = 0;

  bool pressed = (digitalRead(BTN_EXIT_PIN) == HIGH);

  if (pressed && !lastState && (now - lastMs) > 200)
  {
    node->is_exit = !node->is_exit;
    node->best_exit_cost = node->is_exit ? node->local_cost : INF_COST;
    node->direction = node->is_exit ? DIRECTION_SELF : DIRECTION_NONE;
    prefs.putBool("is_exit", node->is_exit);
    lastMs = now;
  }
  lastState = pressed;
}

// ─── Alive toggle ─────────────────────────────────────────────────────────────

void checkAliveToggle(uint32_t now)
{
  static uint32_t lastMs = 0;

  bool pressed = (digitalRead(BTN_ALIVE_PIN) == HIGH);

  if (pressed && !lastAliveBtn && (now - lastMs) > 200)
  {
    nodeAlive = !nodeAlive;

    if (nodeAlive)
    {
      node->sensors.alive = true;
      node->mood = Mood::STBY;
      node->best_exit_cost = INF_COST;
      node->direction = DIRECTION_NONE;
      node->sos_active = false;
      node->still_timer = 0.0f;
      Serial.println("Node online — mood reset.");
    }
    else
    {
      node->sensors.alive = false;
      Serial.println("Node offline.");
    }

    lastMs = now;
  }
  lastAliveBtn = pressed;
}

// ─── Blocked toggle ───────────────────────────────────────────────────────────

void checkBlockedToggle(uint32_t now)
{
  static uint32_t lastMs = 0;

  bool pressed = (digitalRead(BTN_BLOCKED_PIN) == HIGH);

  if (pressed && !lastBlockedBtn && (now - lastMs) > 200)
  {
    btnBlocked = !btnBlocked;
    lastMs = now;
  }
  lastBlockedBtn = pressed;
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup()
{
  Serial.begin(115200);


  // FIX: buttons first, LEDs second — no pin overlap, but ordering is explicit
  // Button pins
  // pinMode(BTN_ALIVE_PIN, INPUT_PULLDOWN);
  pinMode(BTN_BLOCKED_PIN, INPUT_PULLDOWN);
  pinMode(BTN_EXIT_PIN, INPUT_PULLDOWN);

  // NVS
  prefs.begin("hecate", false);
  bool is_exit = prefs.getBool("is_exit", IS_EXIT); // default to config value if not set

  // Node
  node = new NodeState(NODE_ID, FLOOR_ID, NODE_X, NODE_Y, is_exit);


  // FIX: LEDs init after buttons — ledsInit() only touches LED pins
  ledsInit();
  // oledInit();

  // Comms
  espnowInit(onPacketReceived);

  helloPhaseStart = millis();
  bootPhase = BootPhase::HELLO;

  Serial.println("Hecate node ready.");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop()
{
  uint32_t now = millis();

  static uint32_t lastTick = 0;
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;

  // ── HELLO phase ───────────────────────────────────────────────────────────
  if (bootPhase == BootPhase::HELLO)
  {
    // Broadcast HELLO every 500ms so all neighbors hear us
    // even if they booted after us
    if (now - lastHelloBroadcast > HELLO_BROADCAST_INTERVAL_MS)
    {
      HelloPacket hello = node->buildHello();
      espnowSendHello((uint8_t *)&hello, sizeof(HelloPacket));
      lastHelloBroadcast = now;

      Serial.printf("[N%d] HELLO phase — %lu ms remaining\n", node->node_id, HELLO_PHASE_DURATION_MS - (now - helloPhaseStart));
    }

    // Still run LED and OLED so node isn't visually dead during boot
    ledsUpdate(node->mood, node->sos_active);
    // oledUpdate(*node); // will show STBY, 0 neighbors, no cost — that's fine

    // Phase transition check
    if (now - helloPhaseStart >= HELLO_PHASE_DURATION_MS)
    {
      bootPhase = BootPhase::OPERATIONAL;
      node->last_broadcast = millis(); // reset so first OP broadcast isn't immediate
      Serial.printf("[N%d] HELLO phase done — %d neighbor(s) found. Entering OPERATIONAL.\n", node->node_id, node->neighbors.count);
    }

    return; // don't run operational logic during HELLO phase
  }

  // ── OPERATIONAL phase ─────────────────────────────────────────────────────

  // ── 0. Buttons ────────────────────────────────────────────────────────────
  // checkAliveToggle(now);
  checkBlockedToggle(now);
  checkExitToggle(now);

  // ── Offline path ──────────────────────────────────────────────────────────
  if (!nodeAlive)
  {
    ledsUpdate(node->mood, node->sos_active);
    // oledUpdate(*node);
    return;
  }

  // ── 1. Sensors ────────────────────────────────────────────────────────────
  // radarRead();

  node->sensors.heat_norm = analogRead(KNOB_FIRE_PIN) / 4095.0f;
  node->sensors.delta_temp_norm = 0.0f;
  node->sensors.pressure_norm = analogRead(KNOB_PRESSURE_PIN) / 4095.0f;
  node->sensors.still_norm = 0.0f;
  node->sensors.smoke_norm = 0.0f;
  node->sensors.air_norm = analogRead(KNOB_AIR_PIN) / 4095.0f;
  node->sensors.flame_norm = 0.0f;

  // // Hardware blocked OR button blocked
  node->sensors.blocked = btnBlocked;

  // ── 2. Algorithm tick ─────────────────────────────────────────────────────
  node->updateDerivedSensors();
  node->updateLocalCost();
  node->updateMood();
  node->updateNeighbors(now);
  node->updateRouting();
  node->updateRescue(dt);

  // ── 3. Broadcast ──────────────────────────────────────────────────────────
  if (node->broadcastDue(now))
  {
    NodePacket pkt = node->buildPacket(now);
    espnowBroadcast((uint8_t *)&pkt, sizeof(NodePacket));
  }

  // ── 4. Outputs ────────────────────────────────────────────────────────────
  ledsUpdate(node->mood, node->sos_active);
  // oledUpdate(*node);

  static uint32_t lastPrint = 0;
  if (now - lastPrint > 2000)
  {
    Serial.printf("[N%d] [%s] mood=%d cost=%.2f dir=%d sos=%d p=%.2f\n",
                  node->node_id,
                  node->is_exit ? "EXIT" : "NODE",
                  (int)node->mood,
                  node->best_exit_cost,
                  node->direction,
                  node->sos_active,
                  node->p_trapped);
    lastPrint = now;
  }
}