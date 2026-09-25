#include "net_module.h"
#include "net_worker.h"
#include "../image/image_fetcher.h"
#include "../video/video_stream.h"
#include "../diagnostics/diagnostics_retrieval.h"
#include <lvgl.h>
#include <esp_timer.h>
#include "../diagnostics/diagnostics_probes.h"
#include "../diagnostics/sd_diagnostics.h"
#include "../diagnostics/diagnostics_network.h"
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

enum class BenchPhase { Idle, Outage, Restoring };
BenchPhase benchPhase = BenchPhase::Idle;
unsigned long benchStarted = 0;
unsigned int benchAttempt = 0;
int configuredConnection = 0;
char benchCommand[24] = {};
size_t benchCommandLength = 0;
bool benchCommandOverflow = false;
bool observedConnected = false;
bool benchFirstPending=false,restorePending=false;
bool mediaDeferred = false;
int observedAssociation = -1;

bool isSecurePort(uint16_t port);
uint32_t attemptId=0;
bool initialized=false;
lv_obj_t* reconnectNotice=nullptr;
uint32_t noticeAt=0;
bool faultReported=false,allocReported=false;
const char* lastResult="none";

bool observeMqtt() { return mqttowner::view().connected; }
void intentionalDisconnect(const char* reason) {
  diagnet::event("MQTT_DISCONNECT", "reason=%s state_before=%d",reason,netMqttState());
  mqttowner::invalidate(); observedConnected=false;
}

void printBenchStatus() {
  const char* phase = benchPhase == BenchPhase::Outage ? "OFF" :
                      benchPhase == BenchPhase::Restoring ? "RESTORING" : "ON";
  const unsigned long elapsed = benchPhase == BenchPhase::Idle ? 0 : millis() - benchStarted;
  USBSerial.printf("[TEST] %s | WiFi=%s | MQTT=%s | time since off=%lu ms | uptime=%lu ms\n",
                   phase, WiFi.status() == WL_CONNECTED ? "CONNECTED" : "OFFLINE",
                   observeMqtt() ? "CONNECTED" : "DISCONNECTED",
                   elapsed, millis());
  const auto state=mqttowner::view();
  USBSerial.printf("[MQTT OWNER] phase=%s epoch=%lu attempt_epoch=%lu id=%lu age_ms=%llu lease=%u connected=%u stack_min=%lu stack_external=%u tcb_internal=%u internal_min=%lu largest_min=%lu dma_min=%lu rx_drops=%lu tx_drops=%lu completion_drops=%lu lease_timeouts=%lu cancelled=%lu stuck=%lu tx_size=%lu tx_ok=%lu tx_failed=%lu packet_drops=%lu result=%s\n",
    mqttowner::phaseName(state.phase),(unsigned long)state.epoch,(unsigned long)state.attemptEpoch,(unsigned long)state.id,
    (unsigned long long)(state.busy ? esp_timer_get_time()/1000-state.started : 0),state.lease,state.connected,
    (unsigned long)state.stackMin,state.stackExternal,state.tcbInternal,(unsigned long)state.internalMin,
    (unsigned long)state.largestMin,(unsigned long)state.dmaMin,(unsigned long)state.rxDrops,
    (unsigned long)state.txDrops,(unsigned long)state.completionDrops,(unsigned long)state.leaseTimeouts,
    (unsigned long)state.cancelled,(unsigned long)state.stuck,(unsigned long)state.txOversize,
    (unsigned long)state.txAccepted,(unsigned long)state.txRejected,(unsigned long)state.rxPacketDrops,lastResult);
}

void restoreBenchMqtt(const char* reason) {
  if (benchPhase != BenchPhase::Outage) return;
  benchPhase = BenchPhase::Restoring; restorePending=true; benchFirstPending=false;
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
    if (!configuredConnection || WiFi.status() != WL_CONNECTED ||
        !observeMqtt()) {
      diagnet::event("BENCH_APPLIED", "command=off result=refused reason=not_connected");
      USBSerial.println("[TEST] Not started: wait for WiFi and the real MQTT broker to connect.");
      return;
    }
    intentionalDisconnect("bench_off");
    benchPhase = BenchPhase::Outage; benchFirstPending=true;
    benchStarted = millis();
    benchAttempt = 0;

    diagnet::event("BENCH_APPLIED", "command=off target=test port=%u", activePort);
    // Leave time to see orange status and prepare a button press before the first attempt.
    lastMqttAttempt = millis() - MQTT_RECONNECT_INTERVAL + BENCH_FIRST_ATTEMPT_MS;
    USBSerial.printf("[TEST] OFF: MQTT -> 192.0.2.1:%u until you send on; WiFi/HTTPS unchanged.\n",
                     activePort);
    USBSerial.printf("[TEST] First attempt in %lu ms; later attempts use the existing 15-second retry interval.\n",
                     BENCH_FIRST_ATTEMPT_MS);
    USBSerial.println("[TEST] No automatic restore. Commands remain responsive during reconnection.");
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
  cfg=c; mqttowner::init(c); initialized=true;
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t) {
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      case ARDUINO_EVENT_WIFI_STA_STOP: netLinkEvent(false); break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP: netLinkEvent(true); break;
      default: break;
    }
  });
}
void netLinkEvent(bool up) { mqttowner::linkEvent(up); }
void netConfigureMqttClient(int connection) {
  configuredConnection=connection;
  activePort=connection==1 ? cfg.serverPort1 : cfg.serverPort2;
  mqttowner::invalidate();
  diagnet::event("MQTT_CONFIG", "connection=%d target=%s port=%u tls=%u",
    connection,benchPhase==BenchPhase::Outage ? "test" : "real",activePort,isSecurePort(activePort));
}
void netCheckMqtt(bool bypassRateLimit) {
  if(!initialized || !configuredConnection || giveUp || netIsMqttConnected() || netMqttBusy()) return;
  if(!bypassRateLimit && millis()-lastMqttAttempt<MQTT_RECONNECT_INTERVAL) return;
  const bool testAttempt=benchPhase==BenchPhase::Outage;
  if(!mqttowner::request(attemptId+1,configuredConnection,testAttempt)) return;
  ++attemptId; benchFirstPending=false; restorePending=false;
  diagnosticsProbeBegin(ProbeWindow::MqttConnect);
  diagnet::event("MQTT_CONNECT_BEGIN", "execution=worker id=%lu target=%s connection=%d wifi_connection=%d port=%u tls=%u",
    (unsigned long)attemptId,testAttempt ? "test" : "real",configuredConnection,
    diagnet::associatedConnection(),activePort,isSecurePort(activePort));
  if(benchPhase!=BenchPhase::Idle) USBSerial.printf("[TEST] MQTT attempt %u BEGIN -> %s | uptime=%lu ms\n",
    ++benchAttempt,testAttempt ? "TEST endpoint" : "REAL broker",millis());
}
bool netIsMqttConnected() { return observeMqtt(); }
int netMqttState() { return mqttowner::view().state; }
bool netMqttBusy() { const auto state=mqttowner::view(); return state.busy || state.lease; }
bool netMqttFaulted() { return mqttowner::view().phase==mqttowner::Phase::Fault; }
bool netMqttLeaseHeld() { return mqttowner::view().lease; }
void netShutdown() { mqttowner::invalidate(true); }
bool netPublish(const char* topic,const char* payload,const char* category,const char* trigger) {
  if(!topic) return false;
  int which=-1;
  if(cfg.motionTopic && !strcmp(topic,cfg.motionTopic)) which=0;
  else if(cfg.imuTopic && !strcmp(topic,cfg.imuTopic)) which=1;
  else if(!strcmp(topic,"companion/calibration")) which=2;
  return which>=0 && mqttowner::publish(which,payload,category,trigger);
}
void netShowReconnectNotice() {
  if(!reconnectNotice) {
    reconnectNotice=lv_label_create(lv_layer_top()); LV_ASSERT_MALLOC(reconnectNotice);
    lv_obj_clear_flag(reconnectNotice,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_width(reconnectNotice,320); lv_obj_align(reconnectNotice,LV_ALIGN_TOP_MID,0,66);
    lv_obj_set_style_text_color(reconnectNotice,lv_color_white(),0);
    lv_obj_set_style_bg_color(reconnectNotice,lv_color_black(),0);
    lv_obj_set_style_bg_opa(reconnectNotice,LV_OPA_COVER,0);
  }
  lv_label_set_text(reconnectNotice,"Reconnecting. Try again.");
  lv_obj_clear_flag(reconnectNotice,LV_OBJ_FLAG_HIDDEN); noticeAt=millis();
}
void netMainTick() {
  if(!initialized) return;
  static bool inTick=false;
  if(inTick) return; inTick=true;
  mqttowner::arbitrate(!imageFetcherIsBusy() && !videoStreamActive() &&
    !imageFetcherHasPendingDisplay() && !logRetrievalActive());
  static uint32_t sampledAt=0;
  if(mqttowner::view().busy && millis()-sampledAt>=20) { mqttowner::sample(); sampledAt=millis(); }
  auto state=mqttowner::view();
  if(state.phase==mqttowner::Phase::Fault && !faultReported) {
    faultReported=true;
    diagnet::event("MQTT_WORKER_FAULT","result=worker_stuck id=%lu lease=%u recovery=late_cleanup_or_restart",(unsigned long)state.id,state.lease);
    netShowReconnectNotice(); lv_label_set_text(reconnectNotice,"MQTT stalled. Wait or restart.");
  } else if(state.phase!=mqttowner::Phase::Fault && faultReported) {
    faultReported=false; diagnet::event("MQTT_WORKER_FAULT","result=late_cleanup");
    if(reconnectNotice) lv_obj_add_flag(reconnectNotice,LV_OBJ_FLAG_HIDDEN);
  }
  if(state.allocationFailed && !allocReported) {
    allocReported=true; lastResult="worker_alloc"; diagnet::event("MQTT_WORKER_FAULT","result=worker_alloc");
  }
  if(reconnectNotice && !faultReported && millis()-noticeAt>=3000) lv_obj_add_flag(reconnectNotice,LV_OBJ_FLAG_HIDDEN);
  mqttowner::Result result{};
  if(mqttowner::takeResult(result)) {
    // Original completion stamp, not delayed main-task delivery time.
    const bool stale=result.epoch!=mqttowner::view().epoch;
    if(stale) { result.ok=false; result.counted=false; result.reason="cancelled"; }
    lastResult=result.reason;
    char fields[456];
    snprintf(fields,sizeof(fields),"execution=worker id=%lu epoch=%lu target=%s result=%s valid=%u dns_ms=%lu tcp_setup_ms=%lu tls_ms=%lu mqtt_exchange_ms=%lu total_ms=%llu failed_phase=%s state=%d error=%d error_fresh=%u dns=%s cancelled=%u",
      (unsigned long)result.id,(unsigned long)result.epoch,result.test ? "test" : "real",result.reason,result.valid,
      (unsigned long)result.dnsMs,(unsigned long)result.tcpMs,(unsigned long)result.tlsMs,(unsigned long)result.mqttMs,
      (unsigned long long)(result.ended-result.started),result.ok ? "none" : mqttowner::phaseName(result.failedPhase),result.state,
      result.error,result.errorFresh,result.dnsResult,!strcmp(result.reason,"cancelled"));
    diag::recordAt(result.when,"MQTT_CONNECT_END",fields,true);
    diagnet::event("MQTT_CONNECT_MEM","id=%lu stack_min=%lu stack_external=%u tcb_internal=%u internal_min=%lu largest_min=%lu dma_min=%lu",
      (unsigned long)result.id,(unsigned long)state.stackMin,state.stackExternal,state.tcbInternal,
      (unsigned long)state.internalMin,(unsigned long)state.largestMin,(unsigned long)state.dmaMin);
    diagnosticsProbeEnd(ProbeWindow::MqttConnect);
    // Backoff begins after owner completion, never at dispatch or repeated WiFi events.
    lastMqttAttempt = millis();
    if(restorePending) lastMqttAttempt-=MQTT_RECONNECT_INTERVAL;
    if(benchPhase!=BenchPhase::Idle) USBSerial.printf("[TEST] MQTT attempt %u END -> %s | %s | elapsed=%llu ms | state=%d\n",
      benchAttempt,result.test ? "TEST endpoint" : "REAL broker",result.ok ? "CONNECTED" : "FAILED",
      (unsigned long long)(result.ended-result.started),result.state);
    if(result.ok) {
      mqttowner::acknowledgeReady(result.epoch);
      if(result.test) restoreBenchMqtt("unexpected test connection");
      else if(netIsMqttConnected()) {
        diagnet::event("MQTT_CONNECTED","recovery=%u target=real connection=%d port=%u state=%d",everConnected || benchPhase==BenchPhase::Restoring,configuredConnection,activePort,result.state);
        everConnected=true; failureCount=0; observedConnected=true;
        const char* categories[]={"image","power","energy"};
        for(unsigned i=0;i<3;++i) diagnet::event("MQTT_SUBSCRIBE","category=%s qos=1 accepted=%u ack=unobserved",categories[i],(result.subscriptions>>i)&1);
        calibReportStatus(); // main only, enqueues publication
        if(benchPhase==BenchPhase::Restoring) { benchPhase=BenchPhase::Idle; USBSerial.println("[TEST] Real broker connected. Ready for another off."); }
      }
    } else if(result.counted && !result.test) {
      ++failureCount;
      if(!everConnected && failureCount>=MAX_INITIAL_FAILURES) {
        giveUp=true; diagnet::event("MQTT_BUDGET","result=exhausted failures=%u retry=until_reboot",failureCount);
      }
    }
  }
  int lostState;
  if(mqttowner::takeLoss(lostState)) {
    lastMqttAttempt = millis();
    if(benchFirstPending) lastMqttAttempt-=MQTT_RECONNECT_INTERVAL-BENCH_FIRST_ATTEMPT_MS;
    if(restorePending) lastMqttAttempt-=MQTT_RECONNECT_INTERVAL;
    if(observedConnected) diagnet::mqttLoss(lostState,benchPhase==BenchPhase::Outage ? "test" : "real",configuredConnection,activePort,nullptr);
    else diagnet::event("MQTT_CLEANUP","execution=worker state=%d result=closed",lostState);
    observedConnected=false;
  }
  mqttowner::Sent completion{};
  for(unsigned i=0;i<4 && mqttowner::takeSent(completion);++i) {
    // Preserve the old quiet IMU telemetry policy; owner totals still count it.
    if(!strcmp(completion.category,"imu")) continue;
    char fields[144];
    snprintf(fields,sizeof(fields),"category=%s trigger=%s accepted=%u ack=unobserved epoch=%lu",
      completion.category,completion.trigger,completion.accepted,(unsigned long)completion.epoch);
    diag::recordAt(completion.when,"MQTT_PUBLISH",fields,false);
  }
  const uint64_t dispatchAt=esp_timer_get_time();
  mqttowner::Rx message{};
  for(unsigned i=0;i<2 && esp_timer_get_time()-dispatchAt<2000 && mqttowner::takeRx(message);++i) {
    const char* names[]={cfg.topics.image,cfg.topics.power,cfg.topics.energy};
    if(cfg.mqttCallback && message.topic<3 && names[message.topic]) {
      // Application callback never mutates its topic. Bounded payload lives through call.
      cfg.mqttCallback(const_cast<char*>(names[message.topic]),reinterpret_cast<byte*>(message.payload),message.length);
    }
  }
  inTick=false;
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
  const bool deferred = initialized && !connected && !giveUp &&
                        WiFi.status() == WL_CONNECTED && mediaBusy;
  if (deferred != mediaDeferred) {
    diagnet::event("MQTT_RETRY_POLICY", "result=%s reason=%s state=%d",
                   deferred ? "deferred" : "released",
                   deferred ? "media" : connected ? "connected" : WiFi.status() != WL_CONNECTED ? "wifi_offline" : "media_clear",
                   netMqttState());
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
