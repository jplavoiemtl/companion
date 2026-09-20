#pragma once
#include "diagnostics_internal.h"
#include <WiFiClientSecure.h>

// Bounded task-context observers. Labels/formats are code literals, never user data.
namespace diagnet {
void event(const char* name, const char* format, ...) __attribute__((format(printf, 2, 3)));
void init(const char* primary, int primaryNumber, const char* secondary, int secondaryNumber);
const char* profile(const char* ssid);
int associatedConnection();
void inbound(const char* category); // main-task MQTT callback; no payload/topic stored
void health(DiagnosticsHealth& health); // main-task snapshot; writer never calls WiFi
void imageNotification(const char* result);
void publish(const char* category, const char* trigger, bool accepted);
void mqttLoss(int state, const char* target, int connection, uint16_t port, WiFiClientSecure* secure);

// Main task only. Restores the enclosing breadcrumb (including operation ID).
// These are observable blocking spans, not claims of isolated handshake timing.
class Span {
 public:
  Span(const char* kind, diag::Phase phase, const char* context = "");
  ~Span();
  void end(bool ok, int code = 0, WiFiClientSecure* secure = nullptr);
  Span(const Span&) = delete;
  Span& operator=(const Span&) = delete;
 private:
  const char* kind_;
  uint64_t id_ = 0, start_ = 0;
  diag::Crumb previous_{};
  bool done_ = false;
};
}
