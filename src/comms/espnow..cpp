#include "espnow.h"
#include "comms/packets.h"
#include <esp_now.h>
#include <WiFi.h>

// ESP-NOW requires WiFi to be initialized first, but we don't join any network.
// We use WiFi in STA mode purely as the radio layer for ESP-NOW.
// Broadcast address FF:FF:FF:FF:FF:FF reaches all ESP-NOW peers in range
// without needing to register each one individually — which suits dynamic
// neighbor discovery perfectly.

static uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static void (*_userCallback)(const uint8_t *data, int len, uint32_t now) = nullptr;

// ─── Internal receive callback (ESP-NOW format) ───────────────────────────────
static void _onReceive(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    if (_userCallback && data && len > 0)
    {
        _userCallback(data, len, (uint32_t)millis());
    }
}

// ─── Public ──────────────────────────────────────────────────────────────────
void espnowInit(void (*onPacketReceived)(const uint8_t *data, int len, uint32_t now))
{
    _userCallback = onPacketReceived;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // ensure no AP association

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed — restarting.");
        delay(1000);
        ESP.restart(); // cleaner than infinite spin
    }

    esp_now_register_recv_cb(_onReceive);

    // Register broadcast peer — required before sending to that address
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
    peer.channel = 0; // 0 = current channel
    peer.encrypt = false;

    // FIX: check esp_now_add_peer return value
    if (esp_now_add_peer(&peer) != ESP_OK)
    {
        Serial.println("ESP-NOW add peer failed — restarting.");
        delay(1000);
        ESP.restart();
    }
}

void espnowBroadcast(const uint8_t *data, size_t len)
{
    esp_now_send(BROADCAST_ADDR, data, len);
}

// HELLO uses the same broadcast — separate function for call-site clarity
void espnowSendHello(const uint8_t *data, size_t len)
{
    esp_now_send(BROADCAST_ADDR, data, len);
}