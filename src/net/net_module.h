#pragma once

#include <Arduino.h>

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
  void (*mqttCallback)(char*, byte*, unsigned int);
  NetTopics topics;
  const char* motionTopic;
  const char* imuTopic;
};

// Initialize once with immutable configuration (servers and fixed topics)
void netInit(const NetConfig& cfg);

// Select desired profile (1 or 2), invalidate previous epoch; no client access.
void netConfigureMqttClient(int connection);

// MQTT reconnect handler; respects internal rate limiting unless bypassRateLimit=true
void netCheckMqtt(bool bypassRateLimit = false);

// Accessors
bool netIsMqttConnected();
uint16_t netGetActivePort();

// Temporary serial-controlled MQTT outage test; call once per main-loop iteration.
void netBenchLoop();
// Main-only policy diagnostic; reads snapshots, never sockets.
void netObserveRetryPolicy(bool mediaBusy);

// Main-task facade. Success means queued, never wire acceptance.
bool netPublish(const char* topic,const char* payload,const char* category="imu",const char* trigger="periodic");
int netMqttState();
void netMainTick();
bool netMqttBusy();
bool netMqttFaulted();
bool netMqttLeaseHeld();
void netShutdown();
void netLinkEvent(bool up); // WiFi callback, fixed metadata only
void netShowReconnectNotice(); // main/UI only
