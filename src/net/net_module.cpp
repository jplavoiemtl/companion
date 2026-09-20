#include "net_module.h"
#include "../diagnostics/diagnostics_probes.h"
#include "../diagnostics/sd_diagnostics.h"
#include "../diagnostics/diagnostics_network.h"
#if defined(__has_include) && __has_include("secrets_private.h")
#include "secrets_private.h"
#else
#include "secrets.h"
#endif
#include "calibration.h"
#include "HWCDC.h"
#include <WiFi.h>
#include <string.h>

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

// Retained serial bench controls; inactive until an explicit off command.
constexpr bool MQTT_BENCH_ENABLED = true;
constexpr unsigned long BENCH_FIRST_ATTEMPT_MS = 5000;
// RFC 5737 documentation address, not a production service. The network may
// reject it quickly instead of silently dropping it; measure every attempt.
const IPAddress BENCH_ADDRESS(192, 0, 2, 1);
enum class BenchPhase { Idle, Outage, Restoring };
BenchPhase benchPhase = BenchPhase::Idle;
unsigned long benchStarted = 0;
unsigned int benchAttempt = 0;
int configuredConnection = 0;
char benchCommand[24] = {};
size_t benchCommandLength = 0;
bool benchCommandOverflow = false;
bool observedConnected = false;
bool mediaDeferred = false;
int observedAssociation = -1;

bool isSecurePort(uint16_t port);

bool observeMqtt() {
  const bool connected = cfg.mqttClient && cfg.mqttClient->connected();
  if (observedConnected && !connected) {
    // PubSubClient may set LOST/TIMEOUT in loop()/connected(); capture before cleanup.
    diagnet::mqttLoss(cfg.mqttClient->state(), benchPhase == BenchPhase::Outage ? "test" : "real",
                      configuredConnection, activePort, isSecurePort(activePort) ? cfg.secureClient : nullptr);
  }
  observedConnected = connected;
  return connected;
}
void intentionalDisconnect(const char* reason) {
  observeMqtt(); // Preserve a real loss already visible before this requested cleanup.
  diagnet::event("MQTT_DISCONNECT", "reason=%s state_before=%d", reason, cfg.mqttClient->state());
  cfg.mqttClient->disconnect();
  observedConnected = false;
}

void printBenchStatus() {
  const char* phase = benchPhase == BenchPhase::Outage ? "OFF" :
                      benchPhase == BenchPhase::Restoring ? "RESTORING" : "ON";
  const unsigned long elapsed = benchPhase == BenchPhase::Idle ? 0 : millis() - benchStarted;
  USBSerial.printf("[TEST] %s | WiFi=%s | MQTT=%s | time since off=%lu ms | uptime=%lu ms\n",
                   phase, WiFi.status() == WL_CONNECTED ? "CONNECTED" : "OFFLINE",
                   observeMqtt() ? "CONNECTED" : "DISCONNECTED",
                   elapsed, millis());
}

void restoreBenchMqtt(const char* reason) {
  if (benchPhase != BenchPhase::Outage) return;
  benchPhase = BenchPhase::Restoring;
  intentionalDisconnect("bench_restore");
  netConfigureMqttClient(configuredConnection);
  diagnet::event("BENCH_APPLIED", "command=on target=real reason=%s",
                 !strcmp(reason, "serial on") ? "serial_on" : "unexpected_test_connection");
  // Make the real broker eligible immediately; the normal image-busy guard remains.
  lastMqttAttempt = millis() - MQTT_RECONNECT_INTERVAL;
  USBSerial.printf("[TEST] ON: restoring real broker (%s), %lu ms after off.\n",
                   reason, millis() - benchStarted);
}

void handleBenchCommand(char* command) {
  // Accept surrounding spaces, but require an exact command, never a prefix.
  while (*command == ' ' || *command == '\t') ++command;
  size_t length = strlen(command);
  while (length && (command[length - 1] == ' ' || command[length - 1] == '\t')) {
    command[--length] = '\0';
  }
  if (!length) return;

  if (diagnosticsCommand(command)) return;
  if (!strcmp(command, "off") || !strcmp(command, "on") || !strcmp(command, "status"))
    diagnet::event("BENCH_REQUEST", "command=%s target=%s", command, benchPhase == BenchPhase::Outage ? "test" : "real");
  if (strcmp(command, "status") == 0) {
    printBenchStatus();
    diagnosticsPrintStatus();
  } else if (strcmp(command, "on") == 0) {
    if (benchPhase == BenchPhase::Outage) {
      restoreBenchMqtt("serial on");
    } else {
      diagnet::event("BENCH_APPLIED", "command=on result=already_real");
      USBSerial.println("[TEST] ON: real broker already selected.");
      printBenchStatus();
    }
  } else if (strcmp(command, "off") == 0) {
    if (benchPhase == BenchPhase::Outage) {
      diagnet::event("BENCH_APPLIED", "command=off result=already_test");
      USBSerial.println("[TEST] Already OFF; send on to restore the real broker.");
      return;
    }
    if (benchPhase == BenchPhase::Restoring) {
      diagnet::event("BENCH_APPLIED", "command=off result=refused reason=restoring");
      USBSerial.println("[TEST] Restoring; wait for the real broker to reconnect before another off.");
      return;
    }
    if (!configuredConnection || !cfg.mqttClient || WiFi.status() != WL_CONNECTED ||
        !observeMqtt()) {
      diagnet::event("BENCH_APPLIED", "command=off result=refused reason=not_connected");
      USBSerial.println("[TEST] Not started: wait for WiFi and the real MQTT broker to connect.");
      return;
    }
    intentionalDisconnect("bench_off");
    benchPhase = BenchPhase::Outage;
    benchStarted = millis();
    benchAttempt = 0;
    cfg.mqttClient->setServer(BENCH_ADDRESS, activePort);
    diagnet::event("BENCH_APPLIED", "command=off target=test port=%u", activePort);
    // Leave time to see orange status and prepare a button press before the first attempt.
    lastMqttAttempt = millis() - MQTT_RECONNECT_INTERVAL + BENCH_FIRST_ATTEMPT_MS;
    USBSerial.printf("[TEST] OFF: MQTT -> 192.0.2.1:%u until you send on; WiFi/HTTPS unchanged.\n",
                     activePort);
    USBSerial.printf("[TEST] First attempt in %lu ms; later attempts use the existing 15-second retry interval.\n",
                     BENCH_FIRST_ATTEMPT_MS);
    USBSerial.println("[TEST] No automatic restore. Commands wait while a connection attempt blocks.");
  } else {
    USBSerial.println("[TEST] Unknown command. Use: off, on, status (then Enter).");
  }
}

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

  configuredConnection = connection;

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
  if (benchPhase == BenchPhase::Outage) {
    cfg.mqttClient->setServer(BENCH_ADDRESS, activePort);
  }
  diagnet::event("MQTT_CONFIG", "connection=%d target=%s port=%u tls=%u",
                 configuredConnection, benchPhase == BenchPhase::Outage ? "test" : "real",
                 activePort, isSecurePort(activePort));
}

void netCheckMqtt(bool bypassRateLimit) {
  if (!cfg.mqttClient) return;
  if (giveUp) return;

  if (!observeMqtt()) {
    unsigned long currentTime = millis();
    if (!bypassRateLimit && currentTime - lastMqttAttempt < MQTT_RECONNECT_INTERVAL) {
      return;
    }

    cfg.mqttClient->disconnect();  // clean stale state (loss captured above)
    delay(100);

    // Bound TCP and TLS separately; connection attempts are still synchronous.
    // In ESP32 core 3.1.3, setTimeout() affects Stream reads, while
    // setConnectionTimeout() sets the TCP timeout used by connect().
    if (cfg.secureClient) {
      cfg.secureClient->setConnectionTimeout(5000); // TCP connect, ms
      cfg.secureClient->setHandshakeTimeout(5); // TLS handshake, seconds
    }
    if (cfg.wifiClient) {
      cfg.wifiClient->setConnectionTimeout(5000);   // TCP connect, ms
    }
    cfg.mqttClient->setSocketTimeout(5);        // CONNACK wait, seconds

    const bool testAttempt = benchPhase == BenchPhase::Outage;
    const bool logAttempt = benchPhase != BenchPhase::Idle;
    const unsigned long attemptStarted = millis();
    if (logAttempt) {
      USBSerial.printf("[TEST] MQTT attempt %u BEGIN -> %s | uptime=%lu ms\n",
                       ++benchAttempt, testAttempt ? "TEST endpoint" : "REAL broker", attemptStarted);
    }
    // Never send production credentials or the production client ID to the test address.
    char context[144];
    snprintf(context, sizeof(context), "target=%s connection=%d wifi_connection=%d port=%u tls=%u",
             testAttempt ? "test" : "real", configuredConnection,
             diagnet::associatedConnection(), activePort, isSecurePort(activePort));
    diagnet::Span attempt("mqtt_connect", diag::Phase::MqttConnect, context);
    diagnosticsProbeBegin(ProbeWindow::MqttConnect);
    bool ok = testAttempt ? cfg.mqttClient->connect("companion-bench-test")
                          : cfg.mqttClient->connect(CLIENT_ID, USERNAME, KEY);
    // Snapshot state and TLS error before probes, subscriptions or cleanup can reuse them.
    attempt.end(ok, cfg.mqttClient->state(), isSecurePort(activePort) ? cfg.secureClient : nullptr);
    observedConnected = ok;
    diagnosticsProbeEnd(ProbeWindow::MqttConnect);
    if (logAttempt) {
      USBSerial.printf("[TEST] MQTT attempt %u END -> %s | %s | elapsed=%lu ms | state=%d\n",
                       benchAttempt, testAttempt ? "TEST endpoint" : "REAL broker",
                       ok ? "CONNECTED" : "FAILED", millis() - attemptStarted, cfg.mqttClient->state());
    }

    // Stamp the attempt time AFTER it returns. If we stamped before, a
    // long-blocking attempt would already have exceeded MQTT_RECONNECT_INTERVAL
    // by the time it failed, defeating the rate limit and starving LVGL.
    lastMqttAttempt = millis();

    if (testAttempt) {
      // A simulated failure must not consume the production initial-failure budget.
      if (ok) {
        USBSerial.println("[TEST] Unexpected test connection; aborting test and restoring real broker.");
        restoreBenchMqtt("unexpected test connection");
      }
      return;
    }

    if (ok) {
      diagnet::event("MQTT_CONNECTED", "recovery=%u target=real connection=%d port=%u state=%d",
                     everConnected || benchPhase == BenchPhase::Restoring,
                     configuredConnection, activePort, cfg.mqttClient->state());
      everConnected = true;
      failureCount = 0;
      // Subscriptions
      if (cfg.topics.image) {
        const bool accepted = cfg.mqttClient->subscribe(cfg.topics.image, 1);
        diagnet::event("MQTT_SUBSCRIBE", "category=image qos=1 accepted=%u ack=unobserved", accepted);
      }
      if (cfg.topics.power) {
        const bool accepted = cfg.mqttClient->subscribe(cfg.topics.power, 1);
        diagnet::event("MQTT_SUBSCRIBE", "category=power qos=1 accepted=%u ack=unobserved", accepted);
      }
      if (cfg.topics.energy) {
        const bool accepted = cfg.mqttClient->subscribe(cfg.topics.energy, 1);
        diagnet::event("MQTT_SUBSCRIBE", "category=energy qos=1 accepted=%u ack=unobserved", accepted);
      }
      calibReportStatus();
      if (benchPhase == BenchPhase::Restoring) {
        benchPhase = BenchPhase::Idle;
        USBSerial.println("[TEST] Real broker connected. Ready for another off.");
      }
    } else {
      failureCount++;
      if (!everConnected && failureCount >= MAX_INITIAL_FAILURES) {
        giveUp = true;
        diagnet::event("MQTT_BUDGET", "result=exhausted failures=%u retry=until_reboot", failureCount);
        USBSerial.println("MQTT unreachable after initial attempts; giving up until next reboot.");
      }
    }
  }
}

bool netIsMqttConnected() {
  return observeMqtt();
}


void netObserveRetryPolicy(bool mediaBusy) {
#if DIAG_ENABLED
  const bool connected = observeMqtt();
  const int association = diagnet::associatedConnection();
  if (association != observedAssociation) {
    if (association && configuredConnection)
      diagnet::event("MQTT_WIFI_PROFILE", "wifi_connection=%d mqtt_connection=%d mismatch=%u",
                     association, configuredConnection, association != configuredConnection);
    observedAssociation = association;
  }
  const bool deferred = cfg.mqttClient && !connected && !giveUp &&
                        WiFi.status() == WL_CONNECTED && mediaBusy;
  if (deferred != mediaDeferred) {
    diagnet::event("MQTT_RETRY_POLICY", "result=%s reason=%s state=%d",
                   deferred ? "deferred" : "released",
                   deferred ? "media" : connected ? "connected" : WiFi.status() != WL_CONNECTED ? "wifi_offline" : "media_clear",
                   cfg.mqttClient ? cfg.mqttClient->state() : -1);
    mediaDeferred = deferred;
  }
#endif
}

uint16_t netGetActivePort() {
  return activePort;
}

// Poll from loop(), including while WiFi is offline. No blocking readString/readBytes.
void netBenchLoop() {
  if (!MQTT_BENCH_ENABLED) return;
  static bool announced = false;
  if (!announced) {
    USBSerial.println("[TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.");
    announced = true;
  }
  // Bound work per loop so a flood of serial input cannot starve touch/network processing.
  for (unsigned int count = 0; count < 64 && USBSerial.available() > 0; ++count) {
    const int value = USBSerial.read();
    if (value < 0) break;
    const char ch = static_cast<char>(value);
    if (ch == '\r' || ch == '\n') {
      if (benchCommandOverflow) {
        USBSerial.println("[TEST] Command too long; discarded. Use: off, on, status.");
      } else if (benchCommandLength) {
        benchCommand[benchCommandLength] = '\0';
        handleBenchCommand(benchCommand);
      }
      benchCommandLength = 0;
      benchCommandOverflow = false;
    } else if (!benchCommandOverflow) {
      if (ch == '\b' || ch == 127) {
        if (benchCommandLength) --benchCommandLength;
      } else if ((ch >= 32 && ch <= 126) || ch == '\t') {
        if (benchCommandLength < sizeof(benchCommand) - 1) {
          benchCommand[benchCommandLength++] = ch;
        } else {
          benchCommandOverflow = true;
        }
      }
    }
  }
}
