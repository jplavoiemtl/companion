#include "diagnostics_inventory.h"
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

namespace diaginventory {
namespace {
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
struct Slot { diagreader::FileEntry* entries = nullptr; size_t count = 0; unsigned pins = 0; uint64_t at = 0; };
Slot slots[2];
bool online = false, wanted = false, quiet = true, valid = false, stale = true;
uint64_t changed = 0, activeGeneration = 0, quietGeneration = 0;
int published = -1;
const char* error = "pending";
// Only writer touches scan state. quiet is false for the entire enabled interval.
DIR* directory = nullptr;
int staging = -1;
size_t seen = 0, used = 0;
uint64_t nextScan = 0, scanChanged = 0;
uint64_t nowMs() { return esp_timer_get_time()/1000; }
void endScan() {
  if (directory) { closedir(directory); directory = nullptr; }
  staging = -1; seen = used = 0;
}
void fail(const char* reason) {
  endScan();
  portENTER_CRITICAL(&mux); stale = true; error = reason; portEXIT_CRITICAL(&mux);
  nextScan = nowMs()+5000;
}
} // namespace
bool start(uint64_t generation) {
  auto* a = static_cast<diagreader::FileEntry*>(heap_caps_calloc(diagreader::MAX_ENTRIES, sizeof(diagreader::FileEntry), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  auto* b = static_cast<diagreader::FileEntry*>(heap_caps_calloc(diagreader::MAX_ENTRIES, sizeof(diagreader::FileEntry), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!a || !b) { heap_caps_free(a); heap_caps_free(b); return false; }
  portENTER_CRITICAL(&mux);
  const bool ok = online && quiet && !slots[0].entries && !slots[1].entries;
  if (ok) {
    slots[0] = {a,0,0,0}; slots[1] = {b,0,0,0};
    published = -1; valid = false; stale = true; error = "pending";
    ++changed; activeGeneration = generation; quietGeneration = 0; wanted = true; quiet = false;
  }
  portEXIT_CRITICAL(&mux);
  if (!ok) { heap_caps_free(a); heap_caps_free(b); }
  return ok;
}
void stop() { portENTER_CRITICAL(&mux); wanted = false; portEXIT_CRITICAL(&mux); }
bool dispose(uint64_t generation) {
  portENTER_CRITICAL(&mux);
  const bool noAllocation = !slots[0].entries && !slots[1].entries;
  const bool ok = quiet && !wanted && !slots[0].pins && !slots[1].pins &&
    (noAllocation || (activeGeneration == generation && quietGeneration == generation));
  auto* a = ok ? slots[0].entries : nullptr;
  auto* b = ok ? slots[1].entries : nullptr;
  if (ok) { slots[0] = {}; slots[1] = {}; published = -1; valid = false; }
  portEXIT_CRITICAL(&mux);
  heap_caps_free(a); heap_caps_free(b); return ok;
}
View pin(uint64_t now) {
  View v;
  portENTER_CRITICAL(&mux);
  v.valid = valid; v.stale = stale; v.error = error;
  if (valid && published >= 0) {
    Slot& s = slots[published]; ++s.pins;
    v.entries = s.entries; v.count = s.count; v.at = s.at; v.slot = published;
    v.stale = stale || now-s.at > 5000;
  }
  portEXIT_CRITICAL(&mux);
  return v;
}
void unpin(const View& v) {
  if (v.slot < 0) return;
  portENTER_CRITICAL(&mux); if (slots[v.slot].pins) --slots[v.slot].pins; portEXIT_CRITICAL(&mux);
}
void writerOnline() { portENTER_CRITICAL(&mux); online = true; portEXIT_CRITICAL(&mux); }
void writerOffline() {
  endScan(); nextScan = 0;
  portENTER_CRITICAL(&mux); online = false; wanted = false; quiet = true; quietGeneration = activeGeneration; portEXIT_CRITICAL(&mux);
}
void writerPreempt() { endScan(); nextScan = 0; }
void writerChanged() { portENTER_CRITICAL(&mux); ++changed; stale = true; portEXIT_CRITICAL(&mux); }
void writerTick(bool storageReady) {
  portENTER_CRITICAL(&mux);
  const bool run = wanted && storageReady;
  const uint64_t revision = changed;
  portEXIT_CRITICAL(&mux);
  if (!run) {
    endScan(); nextScan = 0;
    portENTER_CRITICAL(&mux); if (!wanted) { quiet = true; quietGeneration = activeGeneration; } portEXIT_CRITICAL(&mux);
    return;
  }
  // A shared USB reservation wins before its writer adapter gets a turn.
  if (diagreader::busy()) { endScan(); nextScan = 0; return; }
  if (directory && revision != scanChanged) { endScan(); nextScan = 0; }
  if (!directory) {
    if (nowMs() < nextScan && revision == scanChanged) return;
    portENTER_CRITICAL(&mux);
    const int candidate = published == 0 ? 1 : 0;
    staging = slots[candidate].pins == 0 ? candidate : -1;
    portEXIT_CRITICAL(&mux);
    if (staging < 0) return;
    directory = opendir("/sdcard/logs");
    if (!directory) { fail("directory_open"); return; }
    seen = used = 0; scanChanged = revision;
  }
  for (unsigned batch = 0; batch < 8; ++batch) {
    portENTER_CRITICAL(&mux); const bool cancelled = !wanted; portEXIT_CRITICAL(&mux);
    if (cancelled || diagreader::busy()) { endScan(); return; }
    errno = 0; dirent* item = readdir(directory);
    if (!item) {
      if (errno) { fail("directory_read"); return; }
      const int completed = staging;
      if (closedir(directory)) { directory = nullptr; fail("directory_close"); return; }
      directory = nullptr;
      portENTER_CRITICAL(&mux);
      if (wanted && scanChanged == changed) {
        slots[completed].count = used; slots[completed].at = nowMs();
        published = completed; valid = true; stale = false; error = "none";
      }
      portEXIT_CRITICAL(&mux);
      staging = -1; nextScan = nowMs()+5000; return;
    }
    if (++seen > diagreader::MAX_ENTRIES) { fail("directory_limit"); return; }
    uint32_t number = 0;
    const bool current = !strcmp(item->d_name,"current.log");
    if (!current && !diagreader::archiveName(item->d_name,number)) continue;
    char path[64]; snprintf(path,sizeof(path),"/sdcard/logs/%s",item->d_name);
    struct stat info{};
    if (stat(path,&info) || info.st_size < 0) { fail("stat"); return; }
    if (S_ISREG(info.st_mode)) slots[staging].entries[used++] = {uint64_t(info.st_size),number,current};
  }
}
} // namespace diaginventory
