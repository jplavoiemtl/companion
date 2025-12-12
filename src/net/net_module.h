#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

struct NetTopics {
  const char* image;
  const char* power;
  const char* energy;
};

struct NetConfig {
  const char* server1;
  uint16_t serverPort1;
  const char* server2;
  uint16_t serverPort2;
  const char* caCert;
  PubSubClient* mqttClient;
  WiFiClient* wifiClient;
  WiFiClientSecure* secureClient;
  void (*mqttCallback)(char*, byte*, unsigned int);
  NetTopics topics;
};

// Initialize module with static configuration (servers, clients, topics)
void netInit(const NetConfig& cfg);

// Configure MQTT client transport/server based on connection index (1 or 2)
void netConfigureMqttClient(int connection);

// MQTT reconnect handler; respects internal rate limiting unless bypassRateLimit=true
void netCheckMqtt(bool bypassRateLimit = false);

// Accessors
bool netIsMqttConnected();
bool netHasInitialMqttSuccess();

