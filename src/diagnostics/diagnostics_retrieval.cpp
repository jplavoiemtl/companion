#include "diagnostics_retrieval.h"
#include "sd_diagnostics.h"
#include "diagnostics_usb.h"
#include "diagnostics_network.h"
#include "../image/image_fetcher.h"
#include "../video/video_stream.h"
#include <esp_timer.h>
#include <WiFi.h>
#include "HWCDC.h"
#include <string.h>
extern HWCDC USBSerial;
extern bool vbusPresent;

namespace {
enum class Mode : uint8_t { Off, Starting, Active, Stopping };
Mode mode = Mode::Off;
constexpr uint64_t IDLE_MS = 300000;
// Observation threshold, not permission to free a buffer or a close deadline.
constexpr uint64_t RELEASE_WARN_MS = 10000;
uint64_t stoppingAt = 0;
bool releaseWarned = false;
uint64_t activityAt = 0;
bool linkUp = false;
const char* lastReason = "boot";
const char* exitReason = "none";
uint64_t nowMs() { return esp_timer_get_time() / 1000; }
const char* stateName() {
  switch (mode) {
    case Mode::Starting: return "STARTING";
    case Mode::Active: return "ACTIVE";
    case Mode::Stopping: return "STOPPING";
    default: return "OFF";
  }
}
void report(const char* result) {
  USBSerial.printf("[LOG RETRIEVAL] state=%s link=%s result=%s reason=%s idle_ms=%llu release_stuck=%u server=absent\n",
    stateName(), linkUp ? "up" : "down", result, lastReason,
    (unsigned long long)(mode == Mode::Off ? 0 : nowMs()-activityAt), unsigned(releaseWarned));
}
const char* entryRefusal() {
  if (mode != Mode::Off) return "not_off";
  if (!vbusPresent) return "usb_power_required";
  if (diagnosticsStorageClosing()) return "logger_closing";
  if (!diagnosticsStorageReady()) return "logger_unavailable";
  if (WiFi.status() != WL_CONNECTED) return "wifi_offline";
  if (imageFetcherIsBusy()) return "image_busy";
  if (videoStreamActive()) return "live_busy";
  if (diagnosticsUsbBusy()) return "retrieval_busy";
  if (imageFetcherHasPendingDisplay()) return "display_pending";
  return nullptr;
}
void enter() {
  if (const char* reason = entryRefusal()) {
    lastReason = reason;
    diagnet::event("RETRIEVAL_MODE", "action=enter trigger=usb reason=%s result=refused", reason);
    report("refused"); return;
  }
  mode = Mode::Starting;
  activityAt = nowMs(); linkUp = true; lastReason = "requested"; releaseWarned = false;
  // No startup resource or failure point until increment 3. Keep STARTING explicit.
  mode = Mode::Active;
  diagnet::event("RETRIEVAL_MODE", "action=enter trigger=usb reason=requested result=ok");
  report("ok");
}
} // namespace
bool logRetrievalActive() { return mode != Mode::Off; }
void logRetrievalExit(const char* reason) {
  if (mode == Mode::Off || mode == Mode::Stopping) return;
  mode = Mode::Stopping; lastReason = exitReason = reason;
  stoppingAt = nowMs(); releaseWarned = false;
  // Existing USB adapter owns cleanup/release on the writer. Never wait for it here.
  if (diagnosticsUsbBusy()) diagnosticsUsbCommand("log abort");
  diagnet::event("RETRIEVAL_MODE", "action=exit trigger=%s reason=%s result=stopping", reason, reason);
}
void logRetrievalTouch() {
  if (mode == Mode::Active) activityAt = nowMs();
}
void logRetrievalTick() {
  if (mode == Mode::Off) return;
  if (mode == Mode::Active || mode == Mode::Starting) {
    // Power/logger exits precede idle. Link loss does not relinquish media exclusion.
    if (!vbusPresent) logRetrievalExit("usb_power_lost");
    else if (diagnosticsStorageClosing()) logRetrievalExit("logger_closing");
    else if (!diagnosticsStorageReady()) logRetrievalExit("logger_unavailable");
    else if (nowMs()-activityAt >= IDLE_MS) logRetrievalExit("idle_timeout");
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (connected != linkUp) {
      linkUp = connected;
      diagnet::event("RETRIEVAL_LINK", "state=%s", linkUp ? "up" : "down");
    }
  }
  if (mode == Mode::Stopping && diagnosticsUsbBusy() &&
      !releaseWarned && nowMs()-stoppingAt >= RELEASE_WARN_MS) {
    releaseWarned = true;
    // One bounded queued record plus operator output. Retain storage and exclusion;
    // a late release can recover naturally, otherwise an operator reboot is needed.
    diagnet::event("RETRIEVAL_STUCK", "reason=release_timeout exit=%s recovery=await_release_or_reboot", exitReason);
    report("release_timeout");
  }
  if (mode == Mode::Stopping && !diagnosticsUsbBusy()) {
    mode = Mode::Off; lastReason = exitReason;
    diagnet::event("RETRIEVAL_MODE", "action=exit trigger=%s reason=%s result=ok", lastReason, lastReason);
  }
}
bool logRetrievalCommand(const char* command) {
  if (!strcmp(command,"log mode on")) { logRetrievalTick(); enter(); return true; }
  if (!strcmp(command,"log mode off")) {
    const bool alreadyOff = mode == Mode::Off;
    const bool alreadyStopping = mode == Mode::Stopping;
    logRetrievalExit("usb_command");
    report(alreadyOff ? "already_off" : alreadyStopping ? "already_stopping" : "stopping");
    return true;
  }
  if (!strcmp(command,"log mode status")) { logRetrievalTick(); report("status"); return true; }
  if (!strncmp(command,"log mode",8) && (command[8] == 0 || command[8] == ' ')) {
    report("invalid_command"); return true;
  }
  return false;
}
