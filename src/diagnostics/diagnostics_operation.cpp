#include "diagnostics_operation.h"
#include "diagnostics_network.h"
#include <esp_timer.h>
namespace diagop {
namespace {
uint64_t now() { return esp_timer_get_time() / 1000ULL; }
uint32_t next = 0;
bool inLoop = false, observed = false, loopSeen = false;
uint64_t loopStart = 0, longest = 0, longestId = 0, maximum = 0;
uint64_t gapCount = 0, lastReport = 0;
uint32_t suppressed = 0;
const char* longestName = "unmeasured";
uint8_t lastScreen = 0;
bool lastUsb = false, lastMoving = false;
}
uint32_t nextId() {
#if DIAG_ENABLED
 return ++next;
#else
 return 0;
#endif
}
void block(const char* name, uint64_t id, uint64_t start, uint64_t end) {
#if DIAG_ENABLED
 // Keep only the longest measured span: nested spans are never added together.
 if (inLoop && start >= loopStart && end - start > longest) {
  longest = end - start; longestName = name; longestId = id;
 }
#endif
}
namespace {
void reportGap(uint64_t end) {
 const uint64_t elapsed = end - loopStart;
 if (elapsed > maximum) maximum = elapsed;
 if (elapsed <= 1000) return;
 ++gapCount;
 if (lastReport && end - lastReport < 5000) { ++suppressed; return; }
 diagnet::event("LOOP_GAP", "elapsed_ms=%llu observed_span=%s span_id=%llu span_ms=%llu other_ms=%llu suppressed=%lu attribution=observation",
  (unsigned long long)elapsed, longestName, (unsigned long long)longestId,
  (unsigned long long)longest, (unsigned long long)(elapsed > longest ? elapsed-longest : 0), (unsigned long)suppressed);
 lastReport = end; suppressed = 0;
}
}
Loop::Loop() {
#if DIAG_ENABLED
 const uint64_t started = now();
 // Entry-to-entry includes scheduler delay between loop invocations. No boot gap.
 if (loopSeen) reportGap(started);
 loopSeen = true; loopStart = started; longest = 0; longestId = 0;
 longestName = "unmeasured"; inLoop = true;
#endif
}
Loop::~Loop() { inLoop = false; }
Block::Block(const char* name, uint64_t id) : name_(name), id_(id), start_(0) {
#if DIAG_ENABLED
 start_ = now();
#endif
}
Block::~Block() {
#if DIAG_ENABLED
 block(name_, id_, start_, now());
#endif
}
void observe(uint8_t screen, bool usb, bool moving) {
#if DIAG_ENABLED
 if (!observed || screen != lastScreen)
  diagnet::event("UI_SCREEN", "from=%u to=%u initial=%u source=main_observation", lastScreen, screen, !observed);
 if (!observed || usb != lastUsb)
  diagnet::event("POWER_USB", "present=%u initial=%u source=pmic_snapshot", usb, !observed);
 if (!observed || moving != lastMoving)
  diagnet::event("MOTION", "moving=%u initial=%u source=main_observation", moving, !observed);
 lastScreen = screen; lastUsb = usb; lastMoving = moving; observed = true;
#endif
}
void health(DiagnosticsHealth& h) {
#if DIAG_ENABLED
 h.loopMaxMs = maximum; h.loopGaps = gapCount; h.loopSuppressed = suppressed;
#endif
}
}
