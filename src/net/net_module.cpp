#include "net_module.h"
#if defined(__has_include) && __has_include("secrets_private.h")
#include "secrets_private.h"
#else
#include "secrets.h"
#endif
#include "calibration.h"
#include "HWCDC.h"

extern HWCDC USBSerial;

namespace {

NetConfig cfg{};

// MQTT reconnect state
unsigned long lastMqttAttempt = 0;
constexpr unsigned long MQTT_RECONNECT_INTERVAL = 15000;  // 15s between reconnection attempts
static uint16_t activePort = 0;

// Give-up state. If MQTT never connects this boot, stop retrying after a budget
// of failed attempts so the UI stays responsive. A power-cycle resets this; the
// trade-off is that we won't recover mid-session if the broker comes back online,
// which is acceptable for this device's frequent power-cycle pattern.
constexpr uint8_t MAX_INITIAL_FAILURES = 5;
uint8_t failureCount = 0;
bool everConnected = false;
bool giveUp = false;

// Helper to decide if port is secure
bool isSecurePort(uint16_t port) {
  return port == 9735 || port == 8883;
}

}  // namespace

void netInit(const NetConfig& c) {
  cfg = c;
  if (cfg.mqttClient) {
    // mqttClient is owned by the sketch; we just configure it here.
    cfg.mqttClient->setBufferSize(512);
    if (cfg.mqttCallback) {
      cfg.mqttClient->setCallback(cfg.mqttCallback);
    }
  }
}

void netConfigureMqttClient(int connection) {
  if (!cfg.mqttClient || !cfg.wifiClient || !cfg.secureClient) return;

  if (connection == 1) {
    activePort = cfg.serverPort1;
                     
    if (isSecurePort(cfg.serverPort1)) {
      cfg.secureClient->setCACert(cfg.caCert);
      cfg.mqttClient->setClient(*cfg.secureClient);
    } else {
      cfg.mqttClient->setClient(*cfg.wifiClient);
    }
    cfg.mqttClient->setServer(cfg.server1, cfg.serverPort1);
  } else {
    activePort = cfg.serverPort2;

    if (isSecurePort(cfg.serverPort2)) {
      cfg.secureClient->setCACert(cfg.caCert);
      cfg.mqttClient->setClient(*cfg.secureClient);
    } else {
      cfg.mqttClient->setClient(*cfg.wifiClient);
    }
    cfg.mqttClient->setServer(cfg.server2, cfg.serverPort2);
  }
}

void netCheckMqtt(bool bypassRateLimit) {
  if (!cfg.mqttClient) return;
  if (giveUp) return;

  if (!cfg.mqttClient->connected()) {
    unsigned long currentTime = millis();
    if (!bypassRateLimit && currentTime - lastMqttAttempt < MQTT_RECONNECT_INTERVAL) {
      return;
    }

    cfg.mqttClient->disconnect();  // clean stale state
    delay(100);

    // Tighten timeouts so a failed attempt unblocks the main loop quickly.
    // Defaults are catastrophic for UI responsiveness: WiFiClientSecure has a
    // 30 s TCP connect timeout and 120 s TLS handshake timeout, and PubSubClient
    // busy-waits (no yield) up to its socketTimeout for CONNACK.
    if (cfg.secureClient) {
      cfg.secureClient->setTimeout(5000);       // TCP connect, ms
      cfg.secureClient->setHandshakeTimeout(5); // TLS handshake, seconds
    }
    if (cfg.wifiClient) {
      cfg.wifiClient->setTimeout(5);            // WiFiClient, seconds
    }
    cfg.mqttClient->setSocketTimeout(5);        // CONNACK wait, seconds

    bool ok = cfg.mqttClient->connect(CLIENT_ID, USERNAME, KEY);

    // Stamp the attempt time AFTER it returns. If we stamped before, a
    // long-blocking attempt would already have exceeded MQTT_RECONNECT_INTERVAL
    // by the time it failed, defeating the rate limit and starving LVGL.
    lastMqttAttempt = millis();

    if (ok) {
      everConnected = true;
      failureCount = 0;
      // Subscriptions
      if (cfg.topics.image) {
        cfg.mqttClient->subscribe(cfg.topics.image, 1);
      }
      if (cfg.topics.power) {
        cfg.mqttClient->subscribe(cfg.topics.power, 1);
      }
      if (cfg.topics.energy) {
        cfg.mqttClient->subscribe(cfg.topics.energy, 1);
      }
      calibReportStatus();
    } else {
      failureCount++;
      if (!everConnected && failureCount >= MAX_INITIAL_FAILURES) {
        giveUp = true;
        USBSerial.println("MQTT unreachable after initial attempts; giving up until next reboot.");
      }
    }
  }
}

bool netIsMqttConnected() {
  return cfg.mqttClient && cfg.mqttClient->connected();
}


uint16_t netGetActivePort() {
  return activePort;
}

