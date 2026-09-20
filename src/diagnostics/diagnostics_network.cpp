#include "diagnostics_network.h"
#include <WiFi.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace diagnet {
namespace {
uint64_t nowMs() { return uint64_t(esp_timer_get_time()) / 1000; }
#if DIAG_ENABLED
const char* primarySsid = nullptr;
const char* secondarySsid = nullptr;
int primaryNumber = 0, secondaryNumber = 0;
portMUX_TYPE wifiMux = portMUX_INITIALIZER_UNLOCKED;
int association = 0;
int lastRssi = 0;
bool rssiValid = false;
uint32_t suppressedWifi = 0;
// Callback-only bounded per-reason/profile gates, including alternating driver reasons.
struct DisconnectGate { bool used = false; int reason = 0, profile = 0; uint64_t last = 0; };
DisconnectGate disconnectGates[4];
uint32_t repeatedDisconnects = 0;
bool validRssi(int rssi) { return rssi > -128 && rssi < 0; }
bool allowDisconnect(int reason, int profile, uint64_t now) {
  int candidate = 0;
  for (int i = 0; i < 4; ++i) {
    DisconnectGate& slot = disconnectGates[i];
    if (slot.used && slot.reason == reason && slot.profile == profile) {
      if (now - slot.last < 5000) return false;
      slot.last = now;
      return true;
    }
    if (!slot.used || (disconnectGates[candidate].used && slot.last < disconnectGates[candidate].last)) candidate = i;
  }
  DisconnectGate& slot = disconnectGates[candidate];
  slot.used = true; slot.reason = reason; slot.profile = profile; slot.last = now;
  return true;
}
// Main-task-only application counters and span IDs.
uint64_t operation = 0, inboundAt = 0, inboundCount = 0, powerCount = 0, energyCount = 0;
uint32_t suppressedImage = 0;

int match(const uint8_t* ssid, size_t length) {
  if (primarySsid && strlen(primarySsid) == length && !memcmp(ssid, primarySsid, length)) return primaryNumber;
  if (secondarySsid && strlen(secondarySsid) == length && !memcmp(ssid, secondarySsid, length)) return secondaryNumber;
  return 0;
}
const char* label(int connection) {
  return connection && connection == primaryNumber ? "primary" :
         connection && connection == secondaryNumber ? "secondary" : "unknown";
}
const char* reasonName(unsigned code) {
  switch (code) {
    case 1: return "unspecified";
    case 2: return "auth_expired";
    case 3: return "auth_leave";
    case 4: return "assoc_expired";
    case 8: return "assoc_leave";
    case 15: return "four_way_timeout";
    case 23: return "auth_8021x_failed";
    case 36: return "sta_leaving";
    case 200: return "beacon_timeout";
    case 201: return "no_ap_found";
    case 202: return "auth_fail";
    case 203: return "assoc_fail";
    case 204: return "handshake_timeout";
    case 205: return "connection_fail";
    default: return "other"; // Numeric SDK reason is authoritative.
  }
}
void wifiEvent(WiFiEvent_t id, WiFiEventInfo_t info) {
  // No String construction, driver queries, SD, serial, NVS or UI here.
  if (id == ARDUINO_EVENT_WIFI_SCAN_DONE) {
    event("WIFI_SCAN_DONE", "source=driver status=%lu count=%u scan_id=%u",
          static_cast<unsigned long>(info.wifi_scan_done.status), info.wifi_scan_done.number, info.wifi_scan_done.scan_id);
  } else if (id == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    const auto& connected = info.wifi_sta_connected;
    const int p = match(connected.ssid, connected.ssid_len);
    portENTER_CRITICAL(&wifiMux); association = p; portEXIT_CRITICAL(&wifiMux);
    event("WIFI_ASSOC", "source=driver profile=%s connection=%d channel=%u suppressed_disconnects=%lu",
          label(p), p, connected.channel, static_cast<unsigned long>(repeatedDisconnects));
    for (auto& slot : disconnectGates) slot.used = false;
    repeatedDisconnects = 0; // Fresh loss after association is never suppressed.
  } else if (id == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    const auto& disconnected = info.wifi_sta_disconnected;
    const int p = match(disconnected.ssid, disconnected.ssid_len);
    const uint64_t now = nowMs();
    portENTER_CRITICAL(&wifiMux);
    association = 0;
    // Reject the int8 lower bound observed on failed scans, and nonnegative values.
    const bool rawValid = validRssi(disconnected.rssi);
    if (rawValid) { lastRssi = disconnected.rssi; rssiValid = true; }
    const int rssi = lastRssi; const bool valid = rssiValid;
    portEXIT_CRITICAL(&wifiMux);
    if (!allowDisconnect(disconnected.reason, p, now)) {
      ++repeatedDisconnects;
      portENTER_CRITICAL(&wifiMux); ++suppressedWifi; portEXIT_CRITICAL(&wifiMux);
      return;
    }
    event("WIFI_DISCONNECT", "source=driver profile=%s connection=%d reason=%u label=%s raw_rssi=%d raw_valid=%u rssi=%d rssi_valid=%u rssi_source=%s suppressed_any=%lu",
          label(p), p, disconnected.reason, reasonName(disconnected.reason), disconnected.rssi, rawValid, rssi, valid,
          rawValid ? "event" : valid ? "last_valid" : "unknown", static_cast<unsigned long>(repeatedDisconnects));
    repeatedDisconnects = 0;
  } else if (id == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    const uint32_t ip = info.got_ip.ip_info.ip.addr;
    const int p = associatedConnection();
    event("WIFI_GOT_IP", "source=driver profile=%s connection=%d ip=%u.%u.%u.%u changed=%u",
          label(p), p, unsigned(ip & 255), unsigned((ip >> 8) & 255),
          unsigned((ip >> 16) & 255), unsigned((ip >> 24) & 255), info.got_ip.ip_changed);
  } else if (id == ARDUINO_EVENT_WIFI_STA_LOST_IP) {
    event("WIFI_LOST_IP", "source=driver");
  }
}
#endif
}
void event(const char* name, const char* format, ...) {
#if DIAG_ENABLED
  char fields[456];
  va_list args; va_start(args, format);
  const int length = vsnprintf(fields, sizeof(fields), format, args);
  va_end(args);
  if (length < 0 || size_t(length) >= sizeof(fields)) {
    // Never store a truncated record or an unbounded original input.
    diag::record("NET_FORMAT_ERROR", "reason=field_capacity", true);
    return;
  }
  const bool routine = !strcmp(name, "MQTT_PUBLISH") || !strcmp(name, "MQTT_IMAGE") ||
                       !strcmp(name, "BENCH_REQUEST");
  diag::record(name, fields, !routine);
#endif
}
void init(const char* primary, int pn, const char* secondary, int sn) {
#if DIAG_ENABLED
  static bool installed = false;
  if (installed) return;
  primarySsid = primary; secondarySsid = secondary;
  primaryNumber = pn; secondaryNumber = sn;
  // Publish immutable profile pointers before registering the task callback.
  WiFi.onEvent(wifiEvent);
  installed = true;
#endif
}
const char* profile(const char* ssid) {
#if DIAG_ENABLED
  return ssid ? label(match(reinterpret_cast<const uint8_t*>(ssid), strlen(ssid))) : "unknown";
#else
  return "unknown";
#endif
}
int associatedConnection() {
#if DIAG_ENABLED
  portENTER_CRITICAL(&wifiMux); const int p = association; portEXIT_CRITICAL(&wifiMux);
  return p;
#else
  return 0;
#endif
}
void inbound(const char* category) {
#if DIAG_ENABLED
  inboundAt = nowMs(); ++inboundCount;
  if (!strcmp(category, "power")) ++powerCount;
  else if (!strcmp(category, "energy")) ++energyCount;
#endif
}
void health(DiagnosticsHealth& h) {
#if DIAG_ENABLED
  portENTER_CRITICAL(&wifiMux);
  if (h.wifi && validRssi(h.rssi)) { lastRssi = h.rssi; rssiValid = true; }
  h.lastRssi = lastRssi; h.rssiValid = rssiValid;
  h.wifiConnection = association; h.wifiSuppressed = suppressedWifi;
  portEXIT_CRITICAL(&wifiMux);
  h.mqttInbound = inboundCount; h.mqttPower = powerCount; h.mqttEnergy = energyCount;
  h.mqttInboundAt = inboundAt; h.mqttInboundKnown = inboundCount != 0;
  h.mqttImageSuppressed = suppressedImage;
#endif
}
void imageNotification(const char* result) {
#if DIAG_ENABLED
  static const char* last = "";
  static uint64_t lastAt = 0;
  static uint32_t suppressed = 0;
  const uint64_t now = nowMs();
  if (strcmp(result, "accepted") && !strcmp(result, last) && now - lastAt < 5000) {
    ++suppressed; ++suppressedImage; return;
  }
  event("MQTT_IMAGE", "result=%s suppressed=%lu", result, static_cast<unsigned long>(suppressed));
  last = result; lastAt = now; suppressed = 0;
#endif
}
void publish(const char* category, const char* trigger, bool accepted) {
  event("MQTT_PUBLISH", "category=%s trigger=%s accepted=%u ack=unobserved", category, trigger, accepted);
}
void mqttLoss(int state, const char* target, int connection, uint16_t port, WiFiClientSecure* secure) {
#if DIAG_ENABLED
  char error[96] = {};
  const int tlsCode = secure ? secure->lastError(error, sizeof(error)) : 0;
  error[sizeof(error)-1] = 0;
  for (char* c = error; *c; ++c)
    if (*c < 32 || *c > 126 || *c == '"' || *c == '\\') *c = '_';
  event("MQTT_LOST", "source=main_observation state=%d target=%s connection=%d port=%u tls_queried=%u tls_code=%d tls_fresh=unknown tls_text=\"%s\"",
        state, target, connection, port, secure != nullptr, tlsCode, error);
#endif
}
Span::Span(const char* kind, diag::Phase phase, const char* context) : kind_(kind) {
#if DIAG_ENABLED
  start_ = nowMs(); id_ = ++operation;
  previous_ = diag::mainBreadcrumb();
  diag::breadcrumb(false, phase, id_);
  event(!strcmp(kind_, "mqtt_connect") ? "MQTT_CONNECT_BEGIN" : "NET_BEGIN",
        "id=%llu kind=%s %s", static_cast<unsigned long long>(id_), kind_, context);
#endif
}
void Span::end(bool ok, int code, WiFiClientSecure* secure) {
#if DIAG_ENABLED
  if (done_) return;
  // Capture before any stop/disconnect/end or client reuse. lastError may be stale
  // after a DNS failure; freshness is deliberately never inferred from its value.
  char error[96] = {};
  const bool queried = !ok && secure;
  const int tlsCode = queried ? secure->lastError(error, sizeof(error)) : 0;
  error[sizeof(error)-1] = 0;
  for (char* c = error; *c; ++c) {
    if (*c < 32 || *c > 126 || *c == '"' || *c == '\\') *c = '_';
  }
  const uint64_t elapsed = nowMs() - start_;
  event(!strcmp(kind_, "mqtt_connect") ? "MQTT_CONNECT_END" : "NET_END",
        "id=%llu kind=%s result=%s elapsed_ms=%llu %s=%d tls_queried=%u tls_code=%d tls_fresh=unknown tls_text=\"%s\"",
        static_cast<unsigned long long>(id_), kind_, ok ? "ok" : "failed",
        static_cast<unsigned long long>(elapsed), !strcmp(kind_, "mqtt_connect") ? "state" : "code",
        code, queried, tlsCode, error);
  diag::breadcrumb(false, static_cast<diag::Phase>(previous_.phase), previous_.operation);
#endif
  done_ = true;
}
Span::~Span() { if (!done_) end(false, -1); }
}
