#pragma once
#include "sd_diagnostics.h"
#include <stddef.h>

namespace diag {
enum class Quality : uint8_t { Unknown, Approx, Synced };
enum class Phase : uint32_t { Setup, Idle, SdMount, SdWrite, SdFlush, SdRotate,
                             SdPrune, SdClose, Sleep, Shutdown, TestPanic, TestWatchdog,
                             WifiSetup, MqttSetup, MqttConnect, ImageHttps, LiveTls };
struct Stamp {
  uint64_t upMs;
  int64_t epochMs;
  Quality quality;
  bool test;
  uint32_t clockRevision;
};
struct Crumb {
  uint32_t magic, version, sequence, phase;
  uint64_t boot, operation, upMs;
  uint32_t checksum;
};
struct Identity {
  uint64_t boot;
  bool persistent;
  char session[32];
  int reset, wake;
  Crumb previousMain, previousWriter;
  bool mainValid, writerValid;
};
extern Identity identity;
void initializeIdentityClock();
void startClock();
Stamp stamp();
const char* qualityName(Quality quality);
void localTime(const Stamp& stamp, char* out, size_t capacity);
void breadcrumb(bool writer, Phase phase, uint64_t operation = 0);
Crumb mainBreadcrumb(); // Locked main-task context snapshot for nested phase restoration.
const char* phaseName(uint32_t phase);
const char* resetName(int reason);
void clockPoll(); // writer, detects unannounced clock changes
int64_t syncAgeMs();
bool record(const char* event, const char* fields, bool important = false);
#if DIAG_TEST_HOOKS
bool clockTest(const char* command);
#endif
} // namespace diag
