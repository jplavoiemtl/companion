#include "diagnostics_internal.h"
#include <Arduino.h>
#include <Preferences.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_random.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <esp_sntp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <sys/time.h>
#include <time.h>
#include <stddef.h>
#include <string.h>

namespace diag {
Identity identity{};
namespace {
constexpr uint32_t MAGIC = 0x44494147;
constexpr uint32_t VERSION = 1;
constexpr char TZ_RULE[] = "EST5EDT,M3.2.0,M11.1.0";
struct ClockRetention { uint32_t magic, version, valid, checksum; };
RTC_NOINIT_ATTR Crumb mainCrumb;
RTC_NOINIT_ATTR Crumb writerCrumb;
RTC_NOINIT_ATTR ClockRetention retainedClock;
portMUX_TYPE rtcMux = portMUX_INITIALIZER_UNLOCKED;
Quality quality = Quality::Unknown;
bool testClock = false;
bool clockStarted = false;
int64_t lastSync = -1, anchorEpoch = 0;
uint64_t anchorUp = 0;
uint32_t revision = 0;

uint32_t hashBytes(const void* ptr, size_t size) {
  uint32_t result = 2166136261u;
  const uint8_t* p = static_cast<const uint8_t*>(ptr);
  while (size--) result = (result ^ *p++) * 16777619u;
  return result;
}
bool validCrumb(const Crumb& c) {
  return c.magic == MAGIC && c.version == VERSION &&
         c.phase <= static_cast<uint32_t>(Phase::LiveTls) &&
         c.checksum == hashBytes(&c, offsetof(Crumb, checksum));
}
void retainClock(bool valid) { // rtcMux held
  retainedClock = {MAGIC, VERSION, valid ? 1u : 0u, 0};
  retainedClock.checksum = hashBytes(&retainedClock, offsetof(ClockRetention, checksum));
}
int64_t epochNow() {
  timeval tv{};
  gettimeofday(&tv, nullptr);
  return int64_t(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
}
bool plausible(int64_t value) {
  return value >= 1704067200000LL && value < 4102444800000LL; // 2024..2099
}
void synchronized(timeval* tv) {
  const uint64_t up = esp_timer_get_time() / 1000;
  const int64_t epoch = int64_t(tv->tv_sec) * 1000 + tv->tv_usec / 1000;
  int64_t correction;
  bool prior;
  portENTER_CRITICAL(&rtcMux);
  if (testClock || !plausible(epoch)) { portEXIT_CRITICAL(&rtcMux); return; }
  prior = quality != Quality::Unknown;
  correction = prior ? epoch - (anchorEpoch + int64_t(up - anchorUp)) : 0;
  quality = Quality::Synced;
  lastSync = up;
  anchorUp = up;
  anchorEpoch = epoch;
  ++revision;
  retainClock(true);
  portEXIT_CRITICAL(&rtcMux);
  char fields[128];
  snprintf(fields, sizeof(fields), "source=sntp prior_known=%u correction_ms=%lld",
           prior, static_cast<long long>(correction));
  record("CLOCK_SYNC", fields, true); // bounded queue, no file or serial I/O
}
} // namespace

const char* phaseName(uint32_t phase) {
  static const char* const names[] = {"setup","idle","sd_mount","sd_write","sd_flush",
    "sd_rotate","sd_prune","sd_close","sleep","shutdown","test_panic","test_watchdog",
    "wifi_setup","mqtt_setup","mqtt_connect","image_https","live_tls"};
  return phase < sizeof(names)/sizeof(names[0]) ? names[phase] : "invalid";
}
const char* resetName(int reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "other";
  }
}
Crumb mainBreadcrumb() {
  portENTER_CRITICAL(&rtcMux); Crumb copy = mainCrumb; portEXIT_CRITICAL(&rtcMux);
  return copy;
}
void breadcrumb(bool writer, Phase phase, uint64_t operation) {
  portENTER_CRITICAL(&rtcMux);
  Crumb& slot = writer ? writerCrumb : mainCrumb;
  const uint32_t next = slot.sequence + 1;
  // Zero the whole object, including padding, for a reproducible checksum.
  memset(&slot, 0, sizeof(slot));
  slot.magic = MAGIC; slot.version = VERSION; slot.sequence = next;
  slot.phase = static_cast<uint32_t>(phase); slot.boot = identity.boot;
  slot.operation = operation; slot.upMs = esp_timer_get_time() / 1000;
  slot.checksum = hashBytes(&slot, offsetof(Crumb, checksum));
  portEXIT_CRITICAL(&rtcMux);
}
void initializeIdentityClock() {
  identity.reset = esp_reset_reason();
  identity.wake = esp_sleep_get_wakeup_cause();
  const bool retained = identity.reset != ESP_RST_POWERON;
  identity.mainValid = retained && validCrumb(mainCrumb);
  identity.writerValid = retained && validCrumb(writerCrumb);
  if (identity.mainValid) identity.previousMain = mainCrumb;
  if (identity.writerValid) identity.previousWriter = writerCrumb;
  const bool provenance = retained && retainedClock.magic == MAGIC &&
    retainedClock.version == VERSION && retainedClock.valid == 1 &&
    retainedClock.checksum == hashBytes(&retainedClock, offsetof(ClockRetention, checksum));
  // One checked boot-counter write, on the internal-stack setup task only.
  Preferences prefs;
  if (prefs.begin("diagnostics", false)) {
    const bool present = prefs.isKey("boot");
    const uint64_t old = prefs.getULong64("boot", UINT64_MAX);
    if ((!present || old != UINT64_MAX) && (!present || old < UINT64_MAX - 1)) {
      const uint64_t next = present ? old + 1 : 1;
      if (prefs.putULong64("boot", next) == sizeof(next) &&
          prefs.getULong64("boot", 0) == next) {
        identity.boot = next;
        identity.persistent = true;
      }
    }
    prefs.end();
  }
  if (identity.persistent)
    snprintf(identity.session, sizeof(identity.session), "boot-%llu",
             static_cast<unsigned long long>(identity.boot));
  else
    snprintf(identity.session, sizeof(identity.session), "volatile-%08lx",
             static_cast<unsigned long>(esp_random()));
  setenv("TZ", TZ_RULE, 1);
  tzset();
  const int64_t epoch = epochNow();
  quality = provenance && plausible(epoch) ? Quality::Approx : Quality::Unknown;
  anchorEpoch = epoch;
  anchorUp = esp_timer_get_time() / 1000;
  retainClock(quality != Quality::Unknown);
  // Read before replacement; invalid slot sequence is reset rather than trusted.
  if (!identity.mainValid) memset(&mainCrumb, 0, sizeof(mainCrumb));
  if (!identity.writerValid) memset(&writerCrumb, 0, sizeof(writerCrumb));
  breadcrumb(false, Phase::Setup);
  breadcrumb(true, Phase::Idle);
}
void startClock() {
  if (clockStarted) return;
  clockStarted = true;
  esp_sntp_set_time_sync_notification_cb(synchronized);
  configTzTime(TZ_RULE, "pool.ntp.org", "time.nist.gov");
}
Stamp stamp() {
  Stamp s{};
  // Clock revision prevents mixing a just-completed synchronization with old epoch.
  uint32_t before;
  do {
    portENTER_CRITICAL(&rtcMux);
    before = revision;
    portEXIT_CRITICAL(&rtcMux);
    s.upMs = esp_timer_get_time() / 1000;
    s.epochMs = epochNow();
    portENTER_CRITICAL(&rtcMux);
    s.quality = quality; s.test = testClock; s.clockRevision = revision;
    bool consistent = revision == before;
    portEXIT_CRITICAL(&rtcMux);
    if (consistent) break;
  } while (true);
  return s;
}
const char* qualityName(Quality q) {
  return q == Quality::Synced ? "synced" : q == Quality::Approx ? "approx" : "unknown";
}
void localTime(const Stamp& s, char* out, size_t capacity) {
  if (s.quality == Quality::Unknown || !plausible(s.epochMs)) {
    snprintf(out, capacity, "unknown"); return;
  }
  time_t seconds = s.epochMs / 1000;
  tm local{};
  localtime_r(&seconds, &local);
  char base[24], offset[8];
  strftime(base, sizeof(base), "%Y-%m-%dT%H:%M:%S", &local);
  strftime(offset, sizeof(offset), "%z", &local);
  snprintf(out, capacity, "%s.%03d%.3s:%.2s", base, int(s.epochMs % 1000), offset, offset + 3);
}
int64_t syncAgeMs() {
  portENTER_CRITICAL(&rtcMux);
  int64_t last = lastSync;
  portEXIT_CRITICAL(&rtcMux);
  return last < 0 ? -1 : int64_t(esp_timer_get_time() / 1000) - last;
}
void clockPoll() {
  const Stamp s = stamp();
  int64_t drift = 0;
  bool changed = false;
  portENTER_CRITICAL(&rtcMux);
  if (s.clockRevision != revision) { portEXIT_CRITICAL(&rtcMux); return; }
  if (s.quality != Quality::Unknown && s.upMs >= anchorUp) {
    drift = s.epochMs - (anchorEpoch + int64_t(s.upMs - anchorUp));
    if (llabs(drift) > 2000) {
      quality = plausible(s.epochMs) ? Quality::Approx : Quality::Unknown;
      lastSync = -1; ++revision;
      retainClock(!testClock && quality != Quality::Unknown);
      changed = true;
    }
  }
  anchorUp = s.upMs; anchorEpoch = s.epochMs;
  portEXIT_CRITICAL(&rtcMux);
  if (s.quality != Quality::Unknown) {
    static char lastOffset[8] = {};
    const time_t seconds = s.epochMs / 1000;
    tm local{};
    localtime_r(&seconds, &local);
    char offset[8] = {};
    strftime(offset, sizeof(offset), "%z", &local);
    if (strcmp(offset, lastOffset)) {
      char fields[96];
      snprintf(fields, sizeof(fields), "from=%s to=%s source=%s",
               *lastOffset ? lastOffset : "unknown", offset, s.test ? "test" : "clock");
      memcpy(lastOffset, offset, sizeof(lastOffset));
      record("CLOCK_OFFSET", fields, true);
    }
  }
  if (changed) {
    char fields[96];
    snprintf(fields, sizeof(fields), "source=%s correction_ms=%lld",
             s.test ? "test" : "unannounced", static_cast<long long>(drift));
    record("CLOCK_CHANGE", fields, true);
  }
}
#if DIAG_TEST_HOOKS
bool clockTest(const char* command) {
  int64_t epoch = 0;
  // Ten seconds before Montreal's 2026 changes; UTC epochs, independent of host TZ.
  if (!strcmp(command, "log test spring")) epoch = 1772953190LL;
  else if (!strcmp(command, "log test autumn")) epoch = 1793512790LL;
  else if (!strcmp(command, "log test sync")) {
    portENTER_CRITICAL(&rtcMux);
    testClock = false; quality = Quality::Unknown; lastSync = -1; ++revision;
    retainClock(false);
    portEXIT_CRITICAL(&rtcMux);
    configTzTime(TZ_RULE, "pool.ntp.org", "time.nist.gov");
    record("CLOCK_TEST_END", "source=test awaiting=sntp", true);
    return true;
  } else return false;
  esp_sntp_stop();
  portENTER_CRITICAL(&rtcMux);
  testClock = true; quality = Quality::Approx; lastSync = -1; ++revision;
  retainClock(false);
  portEXIT_CRITICAL(&rtcMux);
  timeval tv{static_cast<time_t>(epoch), 0};
  settimeofday(&tv, nullptr);
  portENTER_CRITICAL(&rtcMux);
  anchorEpoch = epoch * 1000; anchorUp = esp_timer_get_time() / 1000;
  portEXIT_CRITICAL(&rtcMux);
  record("CLOCK_TEST", "source=test sync=paused", true);
  return true;
}
#endif
} // namespace diag
