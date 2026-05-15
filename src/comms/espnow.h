#pragma once
#include <Arduino.h>

void espnowInit(void (*onPacketReceived)(const uint8_t *data, int len, uint32_t now));
void espnowBroadcast(const uint8_t *data, size_t len);
void espnowSendHello(const uint8_t *data, size_t len);