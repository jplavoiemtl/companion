#include "diagnostics_mqtt_service.h"
#include "diagnostics_internal.h"
#include <esp_timer.h>
#include <stdio.h>
namespace diagmqtt {
namespace {
struct Metric { uint32_t gap=0, cost=0, count=0, over=0, depth=0; uint64_t entered=0, last=0; };
Metric metrics[3];
bool active=false;
uint32_t attempt=0, contexts=0, spanMs=0;
uint64_t started=0;
const char* span="none";
uint64_t now() { return esp_timer_get_time()/1000ULL; }
uint32_t bounded(uint64_t value) { return value>UINT32_MAX ? UINT32_MAX : uint32_t(value); }
void context() {
 const uint32_t phase=diag::mainBreadcrumb().phase;
 if(phase<32) contexts |= uint32_t(1)<<phase;
}
void gap(Metric& m,uint64_t at) {
 const uint32_t duration=bounded(at-m.last);
 if(duration>m.gap) m.gap=duration;
 if(duration>100 && m.over<UINT32_MAX) ++m.over;
 m.last=at;
}
void cost(Metric& m,uint64_t at) {
 const uint64_t from=m.entered>started ? m.entered : started;
 const uint32_t duration=bounded(at-from);
 if(duration>m.cost) m.cost=duration;
}
}
void begin(uint32_t id,uint64_t requestedMs) {
#if DIAG_ENABLED
 attempt=id; started=requestedMs; contexts=spanMs=0; span="none"; active=true;
 for(auto& m:metrics) { m.gap=m.cost=m.over=0; m.count=m.depth ? 1 : 0; m.last=started; }
 context();
#endif
}
void enter(Service service) {
 auto& m=metrics[uint8_t(service)];
 if(m.depth++!=0) return; // Collapse nested LVGL calls, preserving the outer duration.
 m.entered=now();
 if(!active) return;
 gap(m,m.entered); if(m.count<UINT32_MAX) ++m.count; context();
}
void leave(Service service) {
 auto& m=metrics[uint8_t(service)];
 if(!m.depth || --m.depth) return;
 if(active) { cost(m,now()); context(); }
}
void block(const char* name,uint64_t start,uint64_t end) {
 if(!active || end<started) return;
 const uint64_t from=start>started ? start : started;
 const uint32_t duration=bounded(end-from);
 if(duration>spanMs) { spanMs=duration; span=name; }
}
void finish(uint32_t id) {
#if DIAG_ENABLED
 if(!active || id!=attempt) return;
 const uint64_t end=now(); context();
 for(auto& m:metrics) { gap(m,end); if(m.depth) cost(m,end); }
 active=false; // Exactly one record; includes result adoption and unfinished tail gaps.
 char fields[456];
 const auto& u=metrics[0]; const auto& i=metrics[1]; const auto& l=metrics[2];
 snprintf(fields,sizeof(fields),"id=%lu window_ms=%llu ui_gap_ms=%lu ui_call_ms=%lu ui_n=%lu ui_over100=%lu imu_gap_ms=%lu imu_call_ms=%lu imu_n=%lu imu_over100=%lu loop_gap_ms=%lu loop_call_ms=%lu loop_n=%lu loop_over100=%lu contexts=%lu span=%s span_ms=%lu",
  (unsigned long)attempt,(unsigned long long)(end-started),
  (unsigned long)u.gap,(unsigned long)u.cost,(unsigned long)u.count,(unsigned long)u.over,
  (unsigned long)i.gap,(unsigned long)i.cost,(unsigned long)i.count,(unsigned long)i.over,
  (unsigned long)l.gap,(unsigned long)l.cost,(unsigned long)l.count,(unsigned long)l.over,
  (unsigned long)contexts,span,(unsigned long)spanMs);
 diag::record("MQTT_CONNECT_SERVICE",fields,true); // Main adoption time, not worker END.
#endif
}
}
