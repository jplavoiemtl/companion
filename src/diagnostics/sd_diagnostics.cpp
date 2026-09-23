#include "diagnostics_log_time.h"
#include "diagnostics_http_transfer.h"
#include "diagnostics_inventory.h"
#include "diagnostics_retrieval.h"
#include "sd_diagnostics.h"
#include "diagnostics_usb.h"
#include "diagnostics_internal.h"
#include "../../pin_config.h"
#include <Arduino.h>
#include "HWCDC.h"
#include <SD_MMC.h>
#include <esp_vfs_fat.h>
#include <esp_heap_caps.h>
#include <esp_memory_utils.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>
#include <freertos/portmacro.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#if DIAG_TEST_HOOKS
#include <Preferences.h>
#endif

extern HWCDC USBSerial;

namespace {
constexpr char ROOT[] = "/sdcard/logs";
constexpr char CURRENT[] = "/sdcard/logs/current.log";
constexpr uint32_t FILE_LIMIT = 2 * 1024 * 1024;
constexpr uint32_t ARCHIVE_LIMIT = 30;
constexpr uint64_t RESERVE = 16 * 1024 * 1024;
constexpr uint32_t MAX_GENERATION = 99999999;
constexpr size_t ENTRY_LIMIT = 256;
constexpr size_t LINE_CAPACITY = 1024;
// ESP-IDF stack sizes and high-water marks are in bytes.
#if DIAG_WRITER_STACK_PSRAM
constexpr size_t WRITER_STACK = 8192;
constexpr const char* STACK_MODE = "psram";
static StaticTask_t writerTcb; // Internal .bss; never heap-allocated or freed.
#else
constexpr size_t WRITER_STACK = 6144;
constexpr const char* STACK_MODE = "internal";
#endif
enum class WriterLifecycle : uint8_t { Off, Starting, Active, Parked, CreateFailed, Deleted };
const char* lifecycleName(WriterLifecycle value) {
  switch (value) {
    case WriterLifecycle::Off: return "off";
    case WriterLifecycle::Starting: return "starting";
    case WriterLifecycle::Active: return "active";
    case WriterLifecycle::Parked: return "parked";
    case WriterLifecycle::CreateFailed: return "create_failed";
    case WriterLifecycle::Deleted: return "deleted";
  }
  return "invalid";
}
bool writerRunning(WriterLifecycle value) {
  return value == WriterLifecycle::Starting || value == WriterLifecycle::Active;
}
constexpr uint64_t FLUSH_MS = 2000, HEALTH_MS = 60000, SLOW_US = 100000;
enum class State : uint8_t { Off, Starting, Ready, Disabled, Closing, Closed };
const char* stateName(State s) {
  switch (s) {
    case State::Off: return "off"; case State::Starting: return "starting";
    case State::Ready: return "ready"; case State::Disabled: return "disabled";
    case State::Closing: return "closing"; case State::Closed: return "closed";
  }
  return "invalid";
}
struct Event {
  diag::Stamp when;
  char event[24];
  char fields[456];
  bool important;
};
static_assert(sizeof(Event) <= 512, "Keep event queue within 8 KiB");
constexpr uint32_t QUEUE_COUNT = 8192 / sizeof(Event);
constexpr uint32_t IMPORTANT_RESERVE = 4;
struct Snapshot {
  State state = State::Off;
  char error[40] = {};
  int errorCode = 0;
  uint64_t size = 0, freeBytes = 0, cardBytes = 0;
  uint64_t writeMaxUs = 0, flushMaxUs = 0, sdMaxUs = 0;
  uint32_t generation = 0, newest = 0, archives = 0, oversized = 0;
  uint32_t drops = 0, routineDrops = 0, truncated = 0, highWater = 0;
  uint32_t slowWrites = 0, writes = 0, rotations = 0, pruned = 0;
  uint32_t stackMin = UINT32_MAX;
  uint32_t stackFinalMargin = UINT32_MAX;
  uintptr_t stackStart = 0;
  int writerCore = -1;
  WriterLifecycle writerLifecycle = WriterLifecycle::Off;
  bool placementValid = false, stackExternal = false, stackLocalExternal = false;
  bool tcbInternal = false;
  uint32_t internalMin = UINT32_MAX, internalLargest = UINT32_MAX;
  uint32_t dmaMin = UINT32_MAX, dmaLargest = UINT32_MAX;
  uint64_t healthUp = 0;
  DiagnosticsHealth health;
};
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Snapshot snapshot;
// Temporary startup allocation evidence, retained until reboot for late USB attachment.
// Fixed internal storage: no heap allocation or serial output while capturing.
enum class StartupPoint : uint8_t {
  BeforeClock, AfterClock, BeforeWriter, WriterEntry, AfterFormatter,
  BeforeMount, AfterMount, BeforeCurrentOpen, AfterCurrentOpen, StorageDone, Count
};
struct StartupMemory {
  uint64_t upUs = 0;
  uint32_t free = 0, largest = 0, minimum = 0;
  bool captured = false;
};
StartupMemory startupMemory[static_cast<size_t>(StartupPoint::Count)];
static_assert(sizeof(startupMemory) <= 240, "Bound startup probe internal RAM");
constexpr const char* STARTUP_NAMES[] = {
  "before_clock", "after_clock", "before_writer", "writer_entry", "after_formatter",
  "before_mount", "after_mount", "before_current_open", "after_current_open", "storage_done"
};
static_assert(sizeof(STARTUP_NAMES) / sizeof(STARTUP_NAMES[0]) ==
              static_cast<size_t>(StartupPoint::Count), "Name every startup snapshot");
Event* queue = nullptr;
uint32_t head = 0, count = 0;
bool initialized = false, started = false;
bool accepting = false, closeRequested = false, closeDone = false;
bool sleepRequested = false;
diag::Stamp bootStamp{}, closeStamp{};
uint64_t nextHealthCapture = 0;
bool setupFinished = false;

#if DIAG_TEST_HOOKS
enum class Test : uint8_t { None, Small, Normal, Rotate, Rename, Header, Partial, Space, Full };
Test pendingTest = Test::None;
Test pauseAt = Test::None;
bool fullWrites = false, fakeSpace = false;
// Stress state is shared only through mux. Preferences and nextNvsWrite are main-task only.
struct NvsStress {
  bool active = false, summaryPending = false, cleanupOk = false;
  uint64_t startMs = 0, endMs = 0, firstNvsMs = 0, lastNvsMs = 0;
  uint64_t firstSdMs = 0, lastSdMs = 0;
  uint32_t nvsWrites = 0, nvsErrors = 0, sdRecords = 0;
};
NvsStress nvsStress;
Preferences stressPreferences;
bool stressPreferencesOpen = false;
uint64_t nextNvsWrite = 0;
constexpr uint64_t NVS_STRESS_MS = 30000, NVS_INTERVAL_MS = 100;
constexpr uint32_t NVS_WRITE_LIMIT = 300;
#endif

// Everything below here that uses SD or an fd runs only on the writer task.
int fd = -1;
bool mounted = false, dirty = false;
uint64_t sizeBytes = 0, lastFlush = 0, nextHealth = 0, nextClock = 0;
uint64_t sequence = 0;
uint32_t generation = 0, archiveFloor = 0, lastDropReport = 0, lastSlowReport = 0;
uint32_t fileLimit = FILE_LIMIT, archiveLimit = ARCHIVE_LIMIT;
uint64_t reserveBytes = RESERVE;
char* line = nullptr; // PSRAM formatter; SD may internally need a DMA bounce buffer.
Event writerEvent{}; // Single internal staging copy, never per-frame.
struct Inventory {
  uint32_t count = 0, newest = 0, oldest = MAX_GENERATION;
  uint64_t bytes = 0;
  bool any = false;
};

#if DIAG_USB_TEST_FIXTURE
constexpr char FIXTURE_TEMP[] = "/sdcard/logs/usb-fixture.tmp";
static_assert(LINE_CAPACITY % 64 == 0 && FILE_LIMIT % LINE_CAPACITY == 0,
              "Fixture batches and records must exactly fill 2 MiB");
// Only busy/request and status fields cross tasks, always under mux.
struct FixtureStatus {
  bool busy = false, requested = false, deleting = false;
  uint32_t bytes = 0, number = 0;
  const char* result = "none";
  int error = 0;
} fixture;
enum class UsbGate : uint8_t { None, Queue, Prune };
struct UsbGateStatus {
  UsbGate armed = UsbGate::None, active = UsbGate::None;
  uint32_t target = 0, added = 0, queued = 0;
  bool fired = false;
  const char* result = "none";
  const char* outcome = "none";
} usbGate; // Cross-task accesses use mux; filesystem actions are writer-only.
int fixtureFd = -1; // Writer only.
bool fixtureOwnsTemp = false, fixtureDeleting = false;
void fixtureFinish(const char* result, int code = 0);
uint64_t fixtureStarted = 0;
uint32_t fixtureWritten = 0, fixtureNumber = 0;
bool fixtureBusy() {
  portENTER_CRITICAL(&mux); bool busy = fixture.busy; portEXIT_CRITICAL(&mux);
  return busy;
}
#endif

uint64_t nowMs() { return esp_timer_get_time() / 1000; }
Snapshot readSnapshot() {
  portENTER_CRITICAL(&mux); Snapshot copy = snapshot; portEXIT_CRITICAL(&mux);
  return copy;
}
bool closing() {
  portENTER_CRITICAL(&mux); bool result = closeRequested; portEXIT_CRITICAL(&mux);
  return result;
}
void setState(State state, const char* error = "", int code = 0) {
  portENTER_CRITICAL(&mux);
  snapshot.state = state;
  if (*error) { snprintf(snapshot.error, sizeof(snapshot.error), "%s", error); snapshot.errorCode = code; }
  portEXIT_CRITICAL(&mux);
}
void captureStartup(StartupPoint point) {
  const size_t index = static_cast<size_t>(point);
  portENTER_CRITICAL(&mux);
  const bool captured = startupMemory[index].captured;
  portEXIT_CRITICAL(&mux);
  if (captured) return; // Keep the first open, not later rotations.
  StartupMemory sample;
  sample.upUs = esp_timer_get_time();
  sample.free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  sample.largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  sample.minimum = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  sample.captured = true;
  portENTER_CRITICAL(&mux);
  if (!startupMemory[index].captured) startupMemory[index] = sample;
  portEXIT_CRITICAL(&mux);
}
int openCurrentForWrite(int flags) {
  captureStartup(StartupPoint::BeforeCurrentOpen);
  const int result = open(CURRENT, flags, 0666);
  const int code = errno;
  captureStartup(StartupPoint::AfterCurrentOpen);
  errno = code; // Measurement calls must not replace the filesystem error.
  return result;
}
void printStartupMemory() {
  USBSerial.printf("[LOG MEM] retained=boot snapshot_bytes=%u values=bytes timestamps=us\n",
                   unsigned(sizeof(startupMemory)));
  for (size_t i = 0; i < static_cast<size_t>(StartupPoint::Count); ++i) {
    portENTER_CRITICAL(&mux);
    const StartupMemory sample = startupMemory[i];
    portEXIT_CRITICAL(&mux);
    if (!sample.captured) {
      USBSerial.printf("[LOG MEM] phase=%s captured=0\n", STARTUP_NAMES[i]);
      continue;
    }
    USBSerial.printf("[LOG MEM] phase=%s up_us=%llu free=%u largest=%u heap_min_boot=%u\n",
      STARTUP_NAMES[i], static_cast<unsigned long long>(sample.upUs),
      sample.free, sample.largest, sample.minimum);
  }
}
void memorySample(uint64_t duration = 0) {
  uint32_t internal = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  uint32_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  uint32_t dma = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  uint32_t dmaLargest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  uint32_t stack = uxTaskGetStackHighWaterMark(nullptr);
  portENTER_CRITICAL(&mux);
  snapshot.internalMin = min(snapshot.internalMin, internal);
  snapshot.internalLargest = min(snapshot.internalLargest, largest);
  snapshot.dmaMin = min(snapshot.dmaMin, dma);
  snapshot.dmaLargest = min(snapshot.dmaLargest, dmaLargest);
  snapshot.stackMin = min(snapshot.stackMin, stack);
  snapshot.sdMaxUs = max(snapshot.sdMaxUs, duration);
  portEXIT_CRITICAL(&mux);
}
void disable(const char* reason, int code) {
  setState(State::Disabled, reason, code);
  portENTER_CRITICAL(&mux);
  accepting = false;
  snapshot.drops += count; count = 0;
  portEXIT_CRITICAL(&mux);
  // Single terminal failure per boot; never queue logger errors recursively.
  USBSerial.printf("[LOG] disabled reason=%s errno=%d; companion continues\n", reason, code);
}
bool good() { return readSnapshot().state != State::Disabled; }
void archivePath(uint32_t value, char* out, size_t capacity) {
  snprintf(out, capacity, "%s/archive-%08lu.log", ROOT, static_cast<unsigned long>(value));
}
bool archiveNumber(const char* name, uint32_t& value) {
  // Exact basename only; never accept a directory, path, suffix, or case variant.
  if (strlen(name) != 20 || strncmp(name, "archive-", 8) || strcmp(name + 16, ".log")) return false;
  uint32_t n = 0;
  for (int i = 8; i < 16; ++i) {
    if (name[i] < '0' || name[i] > '9') return false;
    n = n * 10 + name[i] - '0';
  }
  value = n; return true;
}
bool inventory(Inventory& result) {
  result = Inventory{};
  DIR* dir = opendir(ROOT);
  if (!dir) { disable("directory_open", errno); return false; }
  size_t entries = 0;
  bool ok = true;
  for (;;) {
    errno = 0;
    dirent* entry = readdir(dir);
    if (!entry) { if (errno) { disable("directory_read", errno); ok = false; } break; }
    if (++entries > ENTRY_LIMIT) { disable("directory_limit", EOVERFLOW); ok = false; break; }
    uint32_t n;
    if (archiveNumber(entry->d_name, n)) {
      char path[80]; archivePath(n, path, sizeof(path));
      struct stat info{};
      if (stat(path, &info)) { disable("archive_stat", errno); ok = false; break; }
      if (S_ISREG(info.st_mode)) {
        if (info.st_size < 0) { disable("archive_size", EIO); ok = false; break; }
        // Preserve oversized evidence; account for its bytes in normal retention.
        if (uint64_t(info.st_size) > FILE_LIMIT) {
          portENTER_CRITICAL(&mux); snapshot.oversized = 1; portEXIT_CRITICAL(&mux);
        }
        result.any = true; ++result.count;
        result.newest = max(result.newest, n); result.oldest = min(result.oldest, n);
        result.bytes += info.st_size;
      }
    }
    if ((entries % 8) == 0) vTaskDelay(1); // core 0 idle watchdog must run
  }
  if (closedir(dir) && ok) { disable("directory_close", errno); ok = false; }
  if (ok) {
    // Retain the resolved high-water generation even if pruning removes every archive.
    if (result.any) archiveFloor = max(archiveFloor, result.newest + 1);
    portENTER_CRITICAL(&mux);
    snapshot.archives = result.count; snapshot.newest = result.any ? result.newest : 0;
    portEXIT_CRITICAL(&mux);
  }
  return ok;
}
bool spaceAvailable() {
  uint64_t total = 0, free = 0;
  // Unlike SD_MMC.usedBytes(), this distinguishes an I/O error from an empty card
  // and resolves the mounted volume by path instead of assuming FatFs drive 0.
  const esp_err_t result = esp_vfs_fat_info("/sdcard", &total, &free);
  if (result != ESP_OK || !total || free > total) {
    disable("space_query", result == ESP_OK ? EIO : int(result)); return false;
  }
#if DIAG_TEST_HOOKS
  if (fakeSpace) free = 0;
#endif
  portENTER_CRITICAL(&mux);
  snapshot.freeBytes = free; snapshot.cardBytes = SD_MMC.cardSize();
  portEXIT_CRITICAL(&mux);
  return true;
}
// Shared removal path: retention chooses the oldest; bench code may select only
// a synthetic archive successfully created during this boot.
bool pruneArchive(uint32_t number) {
  char path[80]; archivePath(number, path, sizeof(path));
  diagnosticsUsbBeforePrune(number); // Reader must close before removal.
#if DIAG_USB_TEST_FIXTURE
  if (fixtureFd >= 0 && fixtureDeleting && fixtureNumber == number)
    fixtureFinish("pruned"); // Close fixture-validation reader before normal pruning.
#endif
  diaginventory::writerChanged();
  if (unlink(path)) { disable("archive_delete", errno); return false; }
  portENTER_CRITICAL(&mux); ++snapshot.pruned; portEXIT_CRITICAL(&mux);
  vTaskDelay(1);
  return true;
}
bool prune(uint64_t incoming, uint32_t futureArchive = 0) {
  diag::breadcrumb(true, diag::Phase::SdPrune);
  Inventory files;
  if (!inventory(files)) return false;
  // Caller has already resolved the next generation before any pruning.
  for (;;) {
    if (!spaceAvailable()) return false;
    Snapshot s = readSnapshot();
    const bool enoughSpace = s.freeBytes >= reserveBytes + incoming;
    const bool enoughCount = files.count + futureArchive <= archiveLimit;
    const bool enoughContent = files.bytes + sizeBytes + incoming <= uint64_t(archiveLimit + 1) * fileLimit;
    if (enoughSpace && enoughCount && enoughContent) break;
    if (!files.any) { disable("reserve_exhausted", ENOSPC); return false; }
    if (!pruneArchive(files.oldest)) return false;
    if (!inventory(files)) return false;
  }
  portENTER_CRITICAL(&mux);
  snapshot.archives = files.count; snapshot.newest = files.any ? files.newest : 0;
  portEXIT_CRITICAL(&mux);
  diag::breadcrumb(true, diag::Phase::Idle);
  return true;
}
#if DIAG_USB_TEST_FIXTURE
void fixtureFinish(const char* result, int code) {
  if (fixtureFd >= 0) {
    const int old = fixtureFd; fixtureFd = -1;
    if (::close(old) && !code) { code = errno; result = "close_failed"; }
  }
  // Never remove a pre-existing temp file, a real archive or current.log.
  if (fixtureOwnsTemp) {
    if (unlink(FIXTURE_TEMP) && !code) { code = errno; result = "temp_cleanup_failed"; }
    fixtureOwnsTemp = false;
  }
  portENTER_CRITICAL(&mux);
  fixture.busy = fixture.requested = false;
  fixture.bytes = fixtureWritten; fixture.number = fixtureNumber;
  fixture.result = result; fixture.error = code;
  portEXIT_CRITICAL(&mux);
  diag::breadcrumb(true, diag::Phase::Idle);
  USBSerial.printf("[LOG FIXTURE] result=%s archive=%08lu bytes=%lu errno=%d\n",
    result, (unsigned long)fixtureNumber, (unsigned long)fixtureWritten, code);
}
// One 1 KiB batch per writer turn. Regular events/flushes run first; no new
// task, NVS access, heap buffer, or busy loop. The formatter is already PSRAM.
void fixtureTick() {
  bool start, deleting; uint32_t requestedNumber;
  portENTER_CRITICAL(&mux);
  start = fixture.requested; fixture.requested = false;
  deleting = fixture.deleting; requestedNumber = fixture.number;
  portEXIT_CRITICAL(&mux);
  if (start) {
    fixtureDeleting = deleting;
    fixtureWritten = 0; fixtureNumber = deleting ? requestedNumber : 0;
    fixtureStarted = nowMs();
    if (diagnosticsUsbBusy() || fileLimit != FILE_LIMIT || archiveLimit != ARCHIVE_LIMIT) {
      fixtureFinish("busy_or_test_limits"); return;
    }
    Inventory files;
    if (!inventory(files)) { fixtureFinish("inventory_failed"); return; }
    if (deleting) {
      char path[80]; archivePath(fixtureNumber, path, sizeof(path));
      fixtureFd = open(path, O_RDONLY);
      if (fixtureFd < 0) { fixtureFinish("fixture_open_failed", errno); return; }
      struct stat info{};
      if (fstat(fixtureFd, &info) || !S_ISREG(info.st_mode) || info.st_size != FILE_LIMIT) {
        fixtureFinish("not_test_fixture"); return;
      }
    } else {
      uint64_t total = 0, free = 0;
      if (esp_vfs_fat_info("/sdcard", &total, &free) != ESP_OK || !total || free > total) {
        fixtureFinish("space_query_failed"); return;
      }
      // Refuse rather than prune real evidence to make room for test data.
      // Extra headroom allows ordinary logging to continue during generation.
      if (files.count + 2 > archiveLimit || free < reserveBytes + FILE_LIMIT + 65536 ||
          files.bytes + sizeBytes + FILE_LIMIT + 65536 > uint64_t(archiveLimit + 1) * fileLimit) {
        fixtureFinish("insufficient_headroom"); return;
      }
      const uint32_t highest = max(generation, archiveFloor);
      if (highest >= MAX_GENERATION - 1) { fixtureFinish("generation_exhausted"); return; }
      fixtureNumber = highest + 1;
      fixtureFd = open(FIXTURE_TEMP, O_WRONLY | O_CREAT | O_EXCL, 0666);
      if (fixtureFd < 0) { fixtureFinish("temp_open_failed", errno); return; }
      fixtureOwnsTemp = true;
      // Reserve this number even if ordinary logging rotates during generation.
      archiveFloor = fixtureNumber + 1;
      portENTER_CRITICAL(&mux); fixture.number = fixtureNumber; portEXIT_CRITICAL(&mux);
    }
  }
  if (fixtureFd < 0) return;
  if (closing() || !good()) { fixtureFinish("cancelled"); return; }
  if (nowMs() - fixtureStarted >= 120000) { fixtureFinish("generation_timeout"); return; }
  if (fixtureDeleting) {
    if (fixtureWritten < FILE_LIMIT) {
      const uint64_t begin = esp_timer_get_time();
      const ssize_t got = read(fixtureFd, line, LINE_CAPACITY);
      const int code = errno;
      memorySample(esp_timer_get_time() - begin);
      if (got != ssize_t(LINE_CAPACITY)) {
        fixtureFinish("fixture_read_failed", got < 0 ? code : EIO); return;
      }
      // Validate EVERY byte, not only a marker or a CRC, before any removal.
      for (unsigned offset = 0; offset < LINE_CAPACITY; offset += 64) {
        char prefix[48];
        const int n = snprintf(prefix, sizeof(prefix), "USB_TEST_FIXTURE line=%08lu ",
          (unsigned long)((fixtureWritten + offset) / 64));
        for (int i = 0; i < 64; ++i) {
          const char expected = i < n ? prefix[i] : i == 63 ? '\n' : '.';
          if (line[offset + i] != expected) { fixtureFinish("not_test_fixture"); return; }
        }
      }
      fixtureWritten += LINE_CAPACITY;
      portENTER_CRITICAL(&mux); fixture.bytes = fixtureWritten; portEXIT_CRITICAL(&mux);
      return;
    }
    const int old = fixtureFd; fixtureFd = -1;
    if (::close(old)) { fixtureFinish("close_failed", errno); return; }
    char target[80]; archivePath(fixtureNumber, target, sizeof(target));
    if (unlink(target)) { fixtureFinish("fixture_delete_failed", errno); return; }
    Inventory files;
    if (!inventory(files) || !spaceAvailable()) { fixtureFinish("refresh_failed"); return; }
    char fields[80];
    snprintf(fields, sizeof(fields), "archive=%08lu synthetic=true",
      (unsigned long)fixtureNumber);
    diag::record("USB_TEST_DELETE", fields, true);
    fixtureFinish("deleted");
    return;
  }
  if (fixtureWritten < FILE_LIMIT) {
    // 32768 deterministic 64-byte lines. Every line visibly marks test data.
    for (unsigned offset = 0; offset < LINE_CAPACITY; offset += 64) {
      char* row = line + offset;
      const int n = snprintf(row, 64, "USB_TEST_FIXTURE line=%08lu ",
        (unsigned long)((fixtureWritten + offset) / 64));
      memset(row + n, '.', 63 - n); row[63] = '\n';
    }
    diag::breadcrumb(true, diag::Phase::SdWrite);
    const uint64_t begin = esp_timer_get_time();
    const ssize_t written = write(fixtureFd, line, LINE_CAPACITY);
    const int code = errno;
    memorySample(esp_timer_get_time() - begin);
    if (written > 0) fixtureWritten += uint32_t(written);
    portENTER_CRITICAL(&mux); fixture.bytes = fixtureWritten; portEXIT_CRITICAL(&mux);
    if (written != ssize_t(LINE_CAPACITY)) {
      fixtureFinish("write_failed", written < 0 ? code : EIO); return;
    }
    diag::breadcrumb(true, diag::Phase::Idle);
    return;
  }
  diag::breadcrumb(true, diag::Phase::SdFlush);
  const uint64_t begin = esp_timer_get_time();
  const int syncResult = fsync(fixtureFd), syncError = errno;
  memorySample(esp_timer_get_time() - begin);
  if (syncResult) { fixtureFinish("flush_failed", syncError); return; }
  const int old = fixtureFd; fixtureFd = -1;
  if (::close(old)) { fixtureFinish("close_failed", errno); return; }
  char target[80]; archivePath(fixtureNumber, target, sizeof(target));
  struct stat info{};
  if (stat(target, &info) == 0) { fixtureFinish("archive_exists", EEXIST); return; }
  if (errno != ENOENT) { fixtureFinish("archive_lookup_failed", errno); return; }
  // Only this writer mutates the directory. Rename publishes only a complete
  // closed fixture; interrupted partial temp files never enter the USB list.
  if (rename(FIXTURE_TEMP, target)) { fixtureFinish("rename_failed", errno); return; }
  fixtureOwnsTemp = false;
  Inventory files;
  if (!inventory(files) || !spaceAvailable()) { fixtureFinish("refresh_failed"); return; }
  char fields[112];
  snprintf(fields, sizeof(fields), "archive=%08lu bytes=%lu synthetic=true",
    (unsigned long)fixtureNumber, (unsigned long)fixtureWritten);
  diag::record("USB_TEST_FIXTURE", fields, true);
  fixtureFinish("ok");
}
#endif

bool flushFile() {
  if (fd < 0 || !dirty) return true;
  diag::breadcrumb(true, diag::Phase::SdFlush);
  memorySample();
  const uint64_t begin = esp_timer_get_time();
  const int result = fsync(fd);
  const int code = errno;
  const uint64_t elapsed = esp_timer_get_time() - begin;
  memorySample(elapsed);
  portENTER_CRITICAL(&mux);
  snapshot.flushMaxUs = max(snapshot.flushMaxUs, elapsed);
  if (elapsed > SLOW_US) ++snapshot.slowWrites;
  portEXIT_CRITICAL(&mux);
  if (result) { disable("flush_failed", code); return false; }
  dirty = false; lastFlush = nowMs();
  diag::breadcrumb(true, diag::Phase::Idle);
  return true;
}
bool closeFile() {
  if (fd < 0) return true;
  bool ok = flushFile();
  int old = fd; fd = -1;
  if (::close(old)) { if (ok) disable("close_failed", errno); return false; }
  return ok;
}
bool rawWrite(const char* bytes, size_t length) {
  if (fd < 0 || !good()) return false;
  if (sizeBytes + length > fileLimit) { disable("record_exceeds_cap", EFBIG); return false; }
#if DIAG_TEST_HOOKS
  if (fullWrites) { disable("test_full_write", ENOSPC); return false; }
#endif
  diag::breadcrumb(true, diag::Phase::SdWrite, sequence);
  memorySample();
  const uint64_t begin = esp_timer_get_time();
  const ssize_t written = write(fd, bytes, length);
  const int code = errno;
  const uint64_t elapsed = esp_timer_get_time() - begin;
  memorySample(elapsed);
  if (written > 0) { sizeBytes += written; dirty = true; }
  portENTER_CRITICAL(&mux);
  snapshot.size = sizeBytes; ++snapshot.writes;
  snapshot.writeMaxUs = max(snapshot.writeMaxUs, elapsed);
  if (elapsed > SLOW_US) ++snapshot.slowWrites;
  portEXIT_CRITICAL(&mux);
  if (written != static_cast<ssize_t>(length)) {
    disable("write_failed", written < 0 ? code : EIO); return false;
  }
  diag::breadcrumb(true, diag::Phase::Idle);
  return true;
}
size_t formatLine(const diag::Stamp& when, const char* level, const char* event, const char* fields) {
  if (sequence == UINT64_MAX) { disable("sequence_exhausted", EOVERFLOW); return 0; }
  char local[48]; diag::localTime(when, local, sizeof(local));
  const int length = snprintf(line, LINE_CAPACITY,
    "local=%s time=%s seq=%llu boot=%llu up_ms=%llu level=%s event=%s %s%s\n",
    local, diag::qualityName(when.quality), static_cast<unsigned long long>(sequence + 1),
    static_cast<unsigned long long>(diag::identity.boot),
    static_cast<unsigned long long>(when.upMs), level, event, fields,
    when.test ? " clock_source=test" : "");
  if (length < 0 || size_t(length) >= LINE_CAPACITY) { disable("record_too_long", EOVERFLOW); return 0; }
  return length;
}
bool direct(const diag::Stamp& when, const char* level, const char* event, const char* fields) {
  size_t length = formatLine(when, level, event, fields);
  if (!length) return false;
  ++sequence;
  const bool ok = rawWrite(line, length);
  if (ok) diagtime::written(line,length,!strcmp(event,"FILE_OPEN"),sizeBytes,generation);
  return ok;
}
bool writeRecord(const diag::Stamp& when, const char* level, const char* event, const char* fields);
bool writeBoot(const diag::Stamp& when, const char* context) {
  char fields[384];
  const auto& id = diag::identity;
  const Snapshot current = readSnapshot();
  snprintf(fields, sizeof(fields),
    "format=1 build=%s compiled=\"%s %s\" session=%s persistent=%u reset=%s reset_code=%d wake_code=%d cpu_mhz=%u context=%s snapshot_valid=%u operation=%s screen=%u",
    DIAG_BUILD_TAG, __DATE__, __TIME__, id.session, id.persistent, diag::resetName(id.reset),
    id.reset, id.wake, getCpuFrequencyMhz(), context, current.healthUp != 0,
    current.health.live ? "live" : current.health.image ? "image" : "idle", current.health.screen);
  return writeRecord(when, "INFO", "BOOT", fields);
}
#if DIAG_TEST_HOOKS
void testPause(Test point) {
  if (pauseAt != point) return;
  pauseAt = Test::None;
  // A partial header must actually reach the card before stopping.
  flushFile();
  USBSerial.println("[LOG TEST] writer paused at requested boundary; reset the board to continue");
  while (!closing()) vTaskDelay(pdMS_TO_TICKS(20)); // do not starve idle or shutdown
}
#endif
bool createCurrent(uint32_t next, const char* reason) {
  fd = openCurrentForWrite(O_WRONLY | O_CREAT | O_EXCL);
  if (fd < 0) { disable("current_create", errno); return false; }
  generation = next; sizeBytes = 0;
#if DIAG_TEST_HOOKS
  if (pauseAt == Test::Partial) {
    if (!rawWrite("local=unknown time=unk", strlen("local=unknown time=unk"))) return false;
    testPause(Test::Partial);
  } else testPause(Test::Header);
  if (closing()) return false;
#endif
  char fields[160];
  snprintf(fields, sizeof(fields), "format=1 generation=%lu reason=%s session=%s",
           static_cast<unsigned long>(generation), reason, diag::identity.session);
  if (!direct(diag::stamp(), "INFO", "FILE_OPEN", fields)) return false;
  portENTER_CRITICAL(&mux); snapshot.generation = generation; portEXIT_CRITICAL(&mux);
  return true;
}
bool unused(uint32_t n) {
  char path[80]; archivePath(n, path, sizeof(path));
  struct stat info{};
  if (stat(path, &info) == 0) { disable("archive_collision", EEXIST); return false; }
  if (errno != ENOENT) { disable("archive_lookup", errno); return false; }
  return true;
}
bool rotate(const char* reason) {
  diaginventory::writerPreempt();
  diaginventory::writerChanged();
  diag::breadcrumb(true, diag::Phase::SdRotate);
  Inventory files;
  if (!inventory(files)) return false;
  uint32_t archive = max(generation, archiveFloor);
  if (archive >= MAX_GENERATION) { disable("generation_exhausted", EOVERFLOW); return false; }
  uint32_t next = archive + 1;
  if (!unused(archive) || !prune(2048, 1) || !closeFile()) return false;
  char target[80]; archivePath(archive, target, sizeof(target));
  if (rename(CURRENT, target)) {
    disable("archive_rename", errno); return false;
  }
#if DIAG_TEST_HOOKS
  testPause(Test::Rename);
  if (closing()) return false;
#endif
  if (!createCurrent(next, reason) || !writeBoot(diag::stamp(), "rotation")) return false;
  portENTER_CRITICAL(&mux); ++snapshot.rotations; portEXIT_CRITICAL(&mux);
  if (!prune(0) || !flushFile()) return false;
  return true;
}
bool writeRecord(const diag::Stamp& when, const char* level, const char* event, const char* fields) {
  size_t length = formatLine(when, level, event, fields);
  if (!length) return false;
  if (sizeBytes + length > fileLimit) {
    if (!rotate("size")) return false;
    length = formatLine(when, level, event, fields); // header consumed sequence numbers
  }
  // Space check once per bounded batch, plus rotation; each record still enforces size.
  if (!length) return false;
  ++sequence;
  const bool ok = rawWrite(line, length);
  if (ok) diagtime::written(line,length,!strcmp(event,"FILE_OPEN"),sizeBytes,generation);
  return ok;
}
bool headerToken(const char*& cursor, const char* key, char* output, size_t capacity) {
  const size_t keyLength = strlen(key);
  if (strncmp(cursor, key, keyLength)) return false;
  cursor += keyLength;
  const char* end = cursor;
  while (*end && *end != ' ' && *end != '\n' && *end != '\r') ++end;
  const size_t length = end - cursor;
  if (!length || length >= capacity) return false;
  memcpy(output, cursor, length); output[length] = 0;
  cursor = end;
  if (*cursor == ' ') ++cursor;
  return true;
}
bool decimal(const char* text, uint64_t maximum, uint64_t& value) {
  value = 0;
  if (!*text) return false;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9') return false;
    const unsigned digit = *text - '0';
    if (value > maximum / 10 || (value == maximum / 10 && digit > maximum % 10)) return false;
    value = value * 10 + digit;
  }
  return true;
}
bool headerGeneration(const char* header, uint32_t& result) {
  // Validate every common field in order, without scanf's overflow/sign acceptance.
  const char* cursor = header;
  char token[64];
  uint64_t number;
  if (!headerToken(cursor, "local=", token, sizeof(token))) return false;
  if (strcmp(token, "unknown") && (strlen(token) != 29 || token[10] != 'T' ||
      token[19] != '.' || (token[23] != '-' && token[23] != '+'))) return false;
  if (!headerToken(cursor, "time=", token, sizeof(token)) ||
      (strcmp(token,"unknown") && strcmp(token,"approx") && strcmp(token,"synced"))) return false;
  if (!headerToken(cursor, "seq=", token, sizeof(token)) || !decimal(token,UINT64_MAX,number) || !number) return false;
  if (!headerToken(cursor, "boot=", token, sizeof(token)) || !decimal(token,UINT64_MAX,number)) return false;
  if (!headerToken(cursor, "up_ms=", token, sizeof(token)) || !decimal(token,UINT64_MAX,number)) return false;
  if (!headerToken(cursor, "level=", token, sizeof(token)) || strcmp(token,"INFO")) return false;
  if (!headerToken(cursor, "event=", token, sizeof(token)) || strcmp(token,"FILE_OPEN")) return false;
  if (!headerToken(cursor, "format=", token, sizeof(token)) || strcmp(token,"1")) return false;
  if (!headerToken(cursor, "generation=", token, sizeof(token)) || !decimal(token,MAX_GENERATION,number)) return false;
  result = static_cast<uint32_t>(number);
  return true;
}
bool openStorage() {
  diag::breadcrumb(true, diag::Phase::SdMount);
  memorySample();
  uint64_t begin = esp_timer_get_time();
  captureStartup(StartupPoint::BeforeMount);
  // Maker BSP uses these same one-bit pins. No formatting or rail changes.
  if (!SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA) ||
      !SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 3)) {
    memorySample(esp_timer_get_time() - begin);
    captureStartup(StartupPoint::AfterMount);
    disable("mount_failed_or_no_card", EIO); return false;
  }
  captureStartup(StartupPoint::AfterMount);
  mounted = true; memorySample(esp_timer_get_time() - begin);
  struct stat info{};
  if (stat(ROOT, &info)) {
    if (errno != ENOENT || mkdir(ROOT, 0775)) { disable("logs_directory", errno); return false; }
  } else if (!S_ISDIR(info.st_mode)) { disable("logs_not_directory", ENOTDIR); return false; }
  Inventory files;
  if (!inventory(files)) return false;
  uint32_t next = files.any ? files.newest + 1 : 1;
  if (next > MAX_GENERATION) { disable("generation_exhausted", EOVERFLOW); return false; }
  bool exists = stat(CURRENT, &info) == 0;
  if (!exists && errno != ENOENT) { disable("current_stat", errno); return false; }
  if (exists && !S_ISREG(info.st_mode)) { disable("current_not_file", EINVAL); return false; }
  bool valid = false, incomplete = false;
  uint32_t saved = 0;
  if (exists && info.st_size > 0) {
    int reader = open(CURRENT, O_RDONLY);
    if (reader < 0) { disable("header_open", errno); return false; }
    ssize_t length = read(reader, line, LINE_CAPACITY - 1);
    bool readOk = length >= 0;
    char tail = 0;
    if (readOk) {
      line[length] = 0;
      char* end = static_cast<char*>(memchr(line, '\n', length));
      if (end) {
        end[1] = 0; valid = headerGeneration(line, saved);
        if (valid) diagtime::restoreCurrent(line,size_t(end-line)+1,uint64_t(info.st_size),saved);
      }
      if (lseek(reader, -1, SEEK_END) < 0 || read(reader, &tail, 1) != 1) readOk = false;
      incomplete = tail != '\n';
    }
    int code = errno;
    if (::close(reader)) { disable("header_close", errno); return false; }
    if (!readOk) { disable("header_read", code ? code : EIO); return false; }
  }
  if (exists && info.st_size > 0 && !valid) {
    // Salvage uses ONLY archive names, never the untrusted header generation.
    // No pruning before the corrupt current has been preserved.
    if (next >= MAX_GENERATION) { disable("generation_exhausted", EOVERFLOW); return false; }
    if (!unused(next)) return false;
    char target[80]; archivePath(next, target, sizeof(target));
    if (rename(CURRENT, target)) { disable("header_salvage_rename", errno); return false; }
    ++next; exists = false;
    // The corrupt file is preserved intact; report oversize through status if needed.
    if (!createCurrent(next, "corrupt_header")) return false;
    if (!writeBoot(bootStamp, "boot_after_salvage")) return false;
  } else if (!exists) {
    sizeBytes = 0;
    if (!prune(2048) || !createCurrent(next, "new") || !writeBoot(bootStamp, "boot")) return false;
  } else {
    // Empty fresh files are recoverable, but are never reopened with truncation.
    if (uint64_t(info.st_size) > FILE_LIMIT) { disable("oversized_current", EFBIG); return false; }
    fd = openCurrentForWrite(O_WRONLY | O_APPEND);
    if (fd < 0) { disable("append_open", errno); return false; }
    sizeBytes = info.st_size; generation = valid ? saved : next;
    portENTER_CRITICAL(&mux); snapshot.generation = generation; snapshot.size = sizeBytes; portEXIT_CRITICAL(&mux);
    if (!sizeBytes) {
      char fields[128];
      snprintf(fields, sizeof(fields), "format=1 generation=%lu reason=empty_recovery",
               static_cast<unsigned long>(generation));
      if (!prune(2048) || !direct(diag::stamp(), "INFO", "FILE_OPEN", fields)) return false;
    } else if (incomplete) {
      if (sizeBytes + 512 > fileLimit) {
        if (!rotate("incomplete_tail")) return false;
      } else {
        if (!prune(512) || !rawWrite("\n", 1) ||
            !direct(diag::stamp(), "WARN", "TAIL_RECOVERY", "action=added_newline")) return false;
      }
    }
    if (!writeBoot(bootStamp, "append")) return false;
  }
  if (!prune(2048)) return false;
  const auto& id = diag::identity;
  for (int slot = 0; slot < 2; ++slot) {
    bool validSlot = slot ? id.writerValid : id.mainValid;
    const diag::Crumb& c = slot ? id.previousWriter : id.previousMain;
    char fields[256];
    snprintf(fields, sizeof(fields),
      "slot=%s valid=%u prior_boot=%llu phase=%s operation=%llu prior_up_ms=%llu crumb_seq=%lu",
      slot ? "writer" : "main", validSlot, static_cast<unsigned long long>(validSlot ? c.boot : 0),
      validSlot ? diag::phaseName(c.phase) : "unknown",
      static_cast<unsigned long long>(validSlot ? c.operation : 0),
      static_cast<unsigned long long>(validSlot ? c.upMs : 0),
      static_cast<unsigned long>(validSlot ? c.sequence : 0));
    if (!writeRecord(bootStamp, "INFO", "PREVIOUS_BREADCRUMB", fields)) return false;
  }
  if (!writeRecord(diag::stamp(), "INFO", "LOGGER_START",
                   "mount=ok mode=sdmmc_1bit mhz=20 format_on_failure=false") || !flushFile()) return false;
  if (readSnapshot().oversized)
    writeRecord(diag::stamp(), "WARN", "OVERSIZED_ARCHIVE", "action=preserved_under_retention");
  setState(State::Ready);
  diag::breadcrumb(true, diag::Phase::Idle);
  return true;
}

void writeHealth(const char* event = "HEALTH") {
  Snapshot s = readSnapshot();
  char fields[704];
  const uint64_t up = nowMs();
  const int length = snprintf(fields, sizeof(fields),
    "snapshot_valid=%u wifi=%u mqtt=%u rssi=%d snapshot_age_ms=%llu screen=%u operation=%s moving=%u usb_power=%u battery=%u battery_mv=%u "
    "internal_free=%u internal_min=%u internal_largest=%u psram_free=%u dma_min=%u dma_largest=%u "
    "sync_age_ms=%lld queue_high=%u drops=%u suppressed=%u truncated=%u writes=%u slow=%u write_max_us=%llu flush_max_us=%llu "
    "sd_max_us=%llu stack_min=%u free_bytes=%llu size=%llu generation=%lu archives=%lu pruned=%lu",
    s.healthUp != 0,s.health.wifi,s.health.mqtt,s.health.wifi ? s.health.rssi : 0,
    static_cast<unsigned long long>(s.healthUp ? up - s.healthUp : up),s.health.screen,
    s.health.live ? "live" : s.health.image ? "image" : "idle",s.health.moving,s.health.usbPower,s.health.battery,s.health.batteryMv,
    unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),unsigned(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
    unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),unsigned(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)),
    s.dmaMin,s.dmaLargest,static_cast<long long>(diag::syncAgeMs()),s.highWater,s.drops,s.routineDrops,s.truncated,s.writes,s.slowWrites,
    static_cast<unsigned long long>(s.writeMaxUs),static_cast<unsigned long long>(s.flushMaxUs),
    static_cast<unsigned long long>(s.sdMaxUs),s.stackMin,
    static_cast<unsigned long long>(s.freeBytes),static_cast<unsigned long long>(s.size),
    static_cast<unsigned long>(s.generation),static_cast<unsigned long>(s.archives),static_cast<unsigned long>(s.pruned));
  if (length < 0 || size_t(length) >= sizeof(fields)) {
    portENTER_CRITICAL(&mux); ++snapshot.truncated; ++snapshot.drops; portEXIT_CRITICAL(&mux);
    return;
  }
  writeRecord(diag::stamp(), "INFO", event, fields);
  // Separate bounded companion record, using the same main-task health snapshot.
  snprintf(fields, sizeof(fields),
    "snapshot_valid=%u snapshot_age_ms=%llu connection=%u rssi=%d rssi_valid=%u wifi_suppressed=%lu "
    "mqtt_inbound=%llu power_count=%llu energy_count=%llu inbound_age_ms=%lld image_suppressed=%lu",
    s.healthUp != 0, static_cast<unsigned long long>(s.healthUp ? up - s.healthUp : up),
    s.health.wifiConnection, s.health.lastRssi, s.health.rssiValid,
    static_cast<unsigned long>(s.health.wifiSuppressed),
    static_cast<unsigned long long>(s.health.mqttInbound), static_cast<unsigned long long>(s.health.mqttPower),
    static_cast<unsigned long long>(s.health.mqttEnergy),
    s.health.mqttInboundKnown ? static_cast<long long>(up - s.health.mqttInboundAt) : -1LL,
    static_cast<unsigned long>(s.health.mqttImageSuppressed));
  writeRecord(diag::stamp(), "INFO", "NET_HEALTH", fields);
  snprintf(fields, sizeof(fields), "loop_max_ms=%llu loop_gaps=%llu pending_suppressed=%lu scope=main_loop",
    (unsigned long long)s.health.loopMaxMs, (unsigned long long)s.health.loopGaps,
    (unsigned long)s.health.loopSuppressed);
  writeRecord(diag::stamp(), "INFO", "OP_HEALTH", fields);
}
bool pop(Event& event) {
  portENTER_CRITICAL(&mux);
  if (!count) { portEXIT_CRITICAL(&mux); return false; }
  event = queue[head];
  head = (head + 1) % QUEUE_COUNT; --count;
  portEXIT_CRITICAL(&mux);
  return true;
}
#if DIAG_TEST_HOOKS
NvsStress readNvsStress() {
  portENTER_CRITICAL(&mux); NvsStress copy = nvsStress; portEXIT_CRITICAL(&mux);
  return copy;
}
// Main-task only. Never invoke Preferences from writerTask or any writer callee.
void finishNvsStress(const char* reason) {
  portENTER_CRITICAL(&mux);
  const bool active = nvsStress.active;
  nvsStress.active = false;
  portEXIT_CRITICAL(&mux);
  if (!active) return;
  bool cleanupOk = false;
  if (stressPreferencesOpen) {
    cleanupOk = !stressPreferences.isKey("pulse") || stressPreferences.remove("pulse");
    stressPreferences.end();
    stressPreferencesOpen = false;
  }
  portENTER_CRITICAL(&mux);
  nvsStress.cleanupOk = cleanupOk;
  if (!cleanupOk) ++nvsStress.nvsErrors;
  nvsStress.summaryPending = true;
  portEXIT_CRITICAL(&mux);
  USBSerial.printf("[LOG TEST] NVS stress ended reason=%s key_removed=%u; use log status for counts\n",
                   reason, cleanupOk);
}
void startNvsStress() {
  const Snapshot s = readSnapshot();
  const NvsStress previous = readNvsStress();
  portENTER_CRITICAL(&mux);
  const bool hookPending = pendingTest != Test::None;
  portEXIT_CRITICAL(&mux);
  if (s.state != State::Ready || !setupFinished || previous.active ||
      previous.summaryPending || closing() || hookPending) {
    USBSerial.println("[LOG TEST] NVS stress unavailable or busy");
    return;
  }
  // Dedicated dummy key; never touch calibration, screen preferences or boot counter.
  stressPreferencesOpen = stressPreferences.begin("diagStress", false);
  if (!stressPreferencesOpen) {
    USBSerial.println("[LOG TEST] NVS stress namespace open failed");
    return;
  }
  if (stressPreferences.isKey("pulse") && !stressPreferences.remove("pulse")) {
    stressPreferences.end(); stressPreferencesOpen = false;
    USBSerial.println("[LOG TEST] NVS stress stale-key cleanup failed");
    return;
  }
  const uint64_t begin = nowMs();
  portENTER_CRITICAL(&mux);
  nvsStress = NvsStress{};
  nvsStress.startMs = begin; nvsStress.endMs = begin + NVS_STRESS_MS;
  nvsStress.active = true;
  portEXIT_CRITICAL(&mux);
  nextNvsWrite = begin;
  diag::record("TEST_NVS_START", "duration_ms=30000 interval_ms=100 max_writes=300", true);
  USBSerial.println("[LOG TEST] NVS stress started: 30 s, up to 300 dummy-key commits; writer flushes test records");
}
void nvsStressMainTick() {
  const NvsStress s = readNvsStress();
  if (!s.active) return;
  const uint64_t now = nowMs();
  if (now >= s.endMs || s.nvsWrites >= NVS_WRITE_LIMIT ||
      readSnapshot().state != State::Ready || closing()) {
    finishNvsStress(now >= s.endMs || s.nvsWrites >= NVS_WRITE_LIMIT ? "complete" : "logger_stopped");
    return;
  }
  if (now < nextNvsWrite) return;
  // No catch-up burst after a blocking MQTT call. Only this main-task tick commits NVS.
  const bool ok = stressPreferences.putUInt("pulse", s.nvsWrites + 1) == sizeof(uint32_t);
  const uint64_t completed = nowMs();
  portENTER_CRITICAL(&mux);
  if (ok) {
    if (!nvsStress.nvsWrites) nvsStress.firstNvsMs = completed;
    ++nvsStress.nvsWrites; nvsStress.lastNvsMs = completed;
  } else ++nvsStress.nvsErrors;
  portEXIT_CRITICAL(&mux);
  nextNvsWrite = completed + NVS_INTERVAL_MS;
  if (!ok) finishNvsStress("nvs_write_failed");
}
// Writer only. No flash operations: observe main-task counters and exercise SD writes.
void nvsStressWriterTick() {
  const NvsStress s = readNvsStress();
  if (!s.active && !s.summaryPending) return;
  if (s.active && nowMs() >= s.endMs) return; // Bound SD stress even if the main loop blocks.
  char fields[320];
  if (s.active) {
    snprintf(fields, sizeof(fields), "source=test nvs_writes=%u sd_records=%u",
             s.nvsWrites, s.sdRecords + 1);
    if (prune(LINE_CAPACITY) && writeRecord(diag::stamp(), "WARN", "TEST_NVS_SD", fields) && flushFile()) {
      const uint64_t completed = nowMs();
      portENTER_CRITICAL(&mux);
      if (!nvsStress.sdRecords) nvsStress.firstSdMs = completed;
      ++nvsStress.sdRecords; nvsStress.lastSdMs = completed;
      portEXIT_CRITICAL(&mux);
    }
  } else {
    snprintf(fields, sizeof(fields),
      "source=test nvs_writes=%u nvs_errors=%u sd_records=%u key_removed=%u first_nvs_ms=%llu last_nvs_ms=%llu first_sd_ms=%llu last_sd_ms=%llu",
      s.nvsWrites, s.nvsErrors, s.sdRecords, s.cleanupOk,
      static_cast<unsigned long long>(s.firstNvsMs), static_cast<unsigned long long>(s.lastNvsMs),
      static_cast<unsigned long long>(s.firstSdMs), static_cast<unsigned long long>(s.lastSdMs));
    if (prune(LINE_CAPACITY) && writeRecord(diag::stamp(), "WARN", "TEST_NVS_END", fields) && flushFile()) {
      portENTER_CRITICAL(&mux); nvsStress.summaryPending = false; portEXIT_CRITICAL(&mux);
    }
  }
}
void processTest() {
  portENTER_CRITICAL(&mux); Test test = pendingTest; pendingTest = Test::None; portEXIT_CRITICAL(&mux);
  if (test == Test::None) return;
  writeRecord(diag::stamp(), "WARN", "TEST_HOOK", "source=serial deliberate=true");
  if (test == Test::Small) { fileLimit = 8192; archiveLimit = 3; reserveBytes = 16384; }
  if (test == Test::Normal) { fileLimit = FILE_LIMIT; archiveLimit = ARCHIVE_LIMIT; reserveBytes = RESERVE; fakeSpace = fullWrites = false; }
  if (test == Test::Space) fakeSpace = true;
  if (test == Test::Full) fullWrites = true;
  if (test == Test::Rename || test == Test::Header || test == Test::Partial) pauseAt = test;
  if (test == Test::Rotate || pauseAt != Test::None || sizeBytes >= fileLimit) rotate("test");
  if (test == Test::Space) prune(1024);
  if (test == Test::Full) writeRecord(diag::stamp(), "WARN", "TEST_FULL_WRITE", "source=test");
}
#endif
#if DIAG_USB_TEST_FIXTURE
const char* gateName(UsbGate gate) {
  return gate == UsbGate::Queue ? "queue" : gate == UsbGate::Prune ? "prune" : "none";
}
void printUsbGate() {
  portENTER_CRITICAL(&mux); const UsbGateStatus g = usbGate; portEXIT_CRITICAL(&mux);
  USBSerial.printf("[LOG USB GATE] armed=%s active=%s target=%lu fired=%u added=%lu queued_at_test=%lu/%u result=%s outcome=%s\n",
    gateName(g.armed),gateName(g.active),(unsigned long)g.target,unsigned(g.fired),
    (unsigned long)g.added,(unsigned long)g.queued,unsigned(QUEUE_COUNT),g.result,g.outcome);
}
void usbGateBegin(const char* name) {
  uint32_t number = 0;
  const bool current = !strcmp(name,"current.log");
  const bool archive = archiveNumber(name,number);
  portENTER_CRITICAL(&mux);
  if (usbGate.armed != UsbGate::None) {
    const bool match = usbGate.armed == UsbGate::Queue ? current : archive && number == usbGate.target;
    usbGate.active = match ? usbGate.armed : UsbGate::None;
    usbGate.armed = UsbGate::None;
    usbGate.result = match ? "waiting_for_data" : "wrong_file";
  }
  portEXIT_CRITICAL(&mux);
}
void usbGateEnd(const char* outcome) {
  portENTER_CRITICAL(&mux);
  if (usbGate.active != UsbGate::None) {
    usbGate.outcome = outcome; usbGate.active = UsbGate::None;
    if (!usbGate.fired) usbGate.result = "ended_before_test";
  }
  portEXIT_CRITICAL(&mux);
}
void usbGateTick() {
  portENTER_CRITICAL(&mux); const UsbGateStatus g = usbGate; portEXIT_CRITICAL(&mux);
  if (g.active == UsbGate::None || g.fired) return;
  bool current; uint32_t number; uint64_t bytes;
  if (!diagnosticsUsbTestProgress(current,number,bytes) || bytes < 1440) return;
  if (g.active == UsbGate::Queue && current && diagnosticsUsbPaused()) {
    // Real PSRAM queue entries, not a fake status count. Prepare outside the lock,
    // then fill only to the guard threshold atomically; preserve all real entries.
    writerEvent = Event{}; writerEvent.when = diag::stamp();
    strcpy(writerEvent.event,"USB_QUEUE_TEST");
    strcpy(writerEvent.fields,"source=bench synthetic=true");
    constexpr uint32_t target = (QUEUE_COUNT + 1) / 2;
    static_assert(target <= QUEUE_COUNT - IMPORTANT_RESERVE, "Bench fill must leave important reserve");
    portENTER_CRITICAL(&mux);
    if (accepting && queue) {
      while (count < target) {
        queue[(head + count) % QUEUE_COUNT] = writerEvent; ++count; ++usbGate.added;
      }
      snapshot.highWater = max(snapshot.highWater,count);
      usbGate.queued = count; usbGate.result = usbGate.added ? "injected" : "already_busy";
    } else usbGate.result = "queue_unavailable";
    usbGate.fired = true;
    portEXIT_CRITICAL(&mux);
    // The unchanged diagnosticsUsbTick() below must detect >=50%, resume the
    // append handle, and let the normal writer drain the injected events.
  } else if (g.active == UsbGate::Prune && !current && number == g.target) {
    portENTER_CRITICAL(&mux);
    const bool owned = !fixture.busy && fixture.number == number &&
      fixture.bytes == FILE_LIMIT && !strcmp(fixture.result,"ok");
    usbGate.fired = true;
    portEXIT_CRITICAL(&mux);
    const char* result = "fixture_not_owned";
    if (owned) {
      // Do not lower retention limits: only the freshly generated fixture is
      // eligible. The same close-reader/unlink helper is used by real pruning.
      char path[80]; archivePath(number,path,sizeof(path));
      struct stat info{};
      if (stat(path,&info) || !S_ISREG(info.st_mode) || info.st_size != FILE_LIMIT) result = "fixture_changed";
      else if (pruneArchive(number)) {
        Inventory files;
        result = inventory(files) && spaceAvailable() ? "pruned" : "refresh_failed";
        portENTER_CRITICAL(&mux); fixture.result = "pruned_by_test"; portEXIT_CRITICAL(&mux);
        diag::record("USB_PRUNE_TEST","synthetic=true",true);
      } else result = "prune_failed";
    }
    portENTER_CRITICAL(&mux); usbGate.result = result; portEXIT_CRITICAL(&mux);
  }
  printUsbGate();
}
#endif
DiagnosticsUsbStatus usbStatus() {
  const Snapshot s = readSnapshot();
  portENTER_CRITICAL(&mux);
  const uint32_t queued = count;
#if DIAG_TEST_HOOKS
  const bool testBusy = pendingTest != Test::None || nvsStress.active || nvsStress.summaryPending;
#else
  const bool testBusy = false;
#endif
#if DIAG_USB_TEST_FIXTURE
  const bool fixtureActive = fixture.busy;
#else
  const bool fixtureActive = false;
#endif
  portEXIT_CRITICAL(&mux);
  return {diag::identity.boot, nowMs(), s.size, s.cardBytes, s.freeBytes,
          s.newest, s.archives + (s.generation ? 1u : 0u), s.drops, queued, QUEUE_COUNT,
          stateName(s.state), s.state == State::Ready && !testBusy && !fixtureActive, closing()};
}
bool usbBegin(const char* name) {
  char fields[128]; snprintf(fields,sizeof(fields),"name=%s bytes=0 duration_ms=0 result=started",name);
  const bool ok = prune(2 * LINE_CAPACITY) &&
                  writeRecord(diag::stamp(),"INFO","USB_GET_BEGIN",fields);
#if DIAG_USB_TEST_FIXTURE
  if (ok) usbGateBegin(name);
#endif
  return ok;
}
bool usbResume() {
  if (!good()) return false;
  if (fd >= 0) return true;
  fd = openCurrentForWrite(O_WRONLY | O_APPEND); // Never create/truncate as recovery.
  if (fd < 0) { disable("current_reopen",errno); return false; }
  return true;
}
void usbEnd(const char* name, uint64_t bytes, uint64_t elapsed, const char* result) {
#if DIAG_USB_TEST_FIXTURE
  usbGateEnd(result);
#endif
  char fields[160];
  snprintf(fields,sizeof(fields),"name=%s bytes=%llu duration_ms=%llu result=%s",
           name,(unsigned long long)bytes,(unsigned long long)elapsed,result);
  // Queue after appends resume; never recursively rotate from a prune callback.
  diag::record("USB_GET_END",fields,true);
}
// Writer review rule: never initiate flash, NVS or partition operations here or
// from its callees. Use FatFS/SDMMC sector wrappers, never raw host/command APIs
// with stack-local data buffers. Cache-off and DMA restrictions still apply.
void writerTask(void*) {
  diaginventory::writerOnline();
  diagtransfer::writerOnline();
  // The task stack and control block exist here, before formatter or SD work.
  // Do not add a startup barrier: main-task Wi-Fi setup may overlap these readings.
  captureStartup(StartupPoint::WriterEntry);
  uint8_t stackMarker = 0;
  uint8_t* const stackStart = pxTaskGetStackStart(nullptr); // Self, never a cleared handle.
#if DIAG_WRITER_STACK_PSRAM
  const void* const tcb = &writerTcb;
#else
  const void* const tcb = xTaskGetCurrentTaskHandle();
#endif
  const bool stackExternal = esp_ptr_external_ram(stackStart);
  const bool stackLocalExternal = esp_ptr_external_ram(&stackMarker);
  const bool tcbInternal = esp_ptr_internal(tcb) && esp_ptr_byte_accessible(tcb);
  portENTER_CRITICAL(&mux);
  snapshot.writerCore = xPortGetCoreID();
  snapshot.stackStart = reinterpret_cast<uintptr_t>(stackStart);
  snapshot.stackExternal = stackExternal;
  snapshot.stackLocalExternal = stackLocalExternal;
  snapshot.tcbInternal = tcbInternal;
  snapshot.placementValid = true;
  snapshot.writerLifecycle = WriterLifecycle::Active;
  portEXIT_CRITICAL(&mux);
  line = static_cast<char*>(heap_caps_malloc(LINE_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  captureStartup(StartupPoint::AfterFormatter);
  if (!line) disable("formatter_allocation", ENOMEM);
  else if (openStorage()) {
    USBSerial.printf("[LOG] ready file=/logs/current.log boot=%llu queue_bytes=%u stack_bytes=%u core=%d\n",
      static_cast<unsigned long long>(diag::identity.boot), unsigned(QUEUE_COUNT * sizeof(Event)), unsigned(WRITER_STACK), int(xPortGetCoreID()));
  }
  captureStartup(StartupPoint::StorageDone);
  nextHealth = nowMs() + HEALTH_MS;
  nextClock = nowMs();
  uint64_t nextMemorySample = 0;
  while (good() && (fd >= 0 || diagnosticsUsbPaused())) {
    // USB uses shorter turns, but retain the ordinary writer sampling cadence.
    if (nowMs() >= nextMemorySample) { memorySample(); nextMemorySample = nowMs() + 20; }
    if (closing()) {
#if DIAG_USB_TEST_FIXTURE
      if (fixtureBusy()) fixtureFinish("shutdown");
      usbGateEnd("shutdown");
      portENTER_CRITICAL(&mux); usbGate.armed = UsbGate::None; portEXIT_CRITICAL(&mux);
#endif
      diagtransfer::writerOffline();
      diaginventory::writerOffline();
      diagnosticsUsbStop(); // Close reader and resume append before the close drain.
      if (!good()) break;
#if DIAG_TEST_HOOKS
      nvsStressWriterTick(); // Main stopped the test; persist its final counts before close.
      if (!good()) break;
#endif
      setState(State::Closing);
      // Producers stop before the request; drain only the finite pre-close queue.
      if (!prune(QUEUE_COUNT * LINE_CAPACITY + 1024)) break;
      while (pop(writerEvent) && good()) {
        writeRecord(writerEvent.when, writerEvent.important ? "INFO" : "DEBUG", writerEvent.event, writerEvent.fields);
        vTaskDelay(1);
      }
      char fields[96];
      snprintf(fields, sizeof(fields), "reason=%s pending=0",
               sleepRequested ? "deep_sleep" : "shutdown");
      writeRecord(closeStamp, "INFO", "SESSION_END", fields);
      diag::breadcrumb(true, diag::Phase::SdClose);
      if (closeFile()) {
        diag::breadcrumb(true, diag::Phase::SdClose);
        setState(State::Closed);
      }
      break;
    }
#if DIAG_TEST_HOOKS
    processTest();
    if (!good()) break;
    nvsStressWriterTick();
    if (!good()) break;
#endif
    if (nowMs() >= nextClock) { diag::clockPoll(); nextClock = nowMs() + 1000; }
#if DIAG_USB_TEST_FIXTURE
    usbGateTick();
    if (!good()) break;
#endif
    if (diagnosticsUsbPaused()) {
      diaginventory::writerTick(good()); // stop acknowledgement also while USB pauses appends
      diagnosticsUsbTick(); // Bounds and queue checks continue while append is closed.
      vTaskDelay(1);
      continue;
    }
    bool queued;
    portENTER_CRITICAL(&mux); queued = count != 0; portEXIT_CRITICAL(&mux);
    bool health = nowMs() >= nextHealth;
    if (queued || health) {
      if (!prune(4 * LINE_CAPACITY)) break;
      for (int batch = 0; batch < 4 && pop(writerEvent) && good(); ++batch)
        writeRecord(writerEvent.when, "INFO", writerEvent.event, writerEvent.fields);
      Snapshot s = readSnapshot();
      if (s.drops != lastDropReport || s.slowWrites != lastSlowReport) {
        char fields[160];
        snprintf(fields, sizeof(fields), "drops=%u routine_suppressed=%u slow_total=%u", s.drops,s.routineDrops,s.slowWrites);
        lastDropReport = s.drops; lastSlowReport = s.slowWrites;
        writeRecord(diag::stamp(), "WARN", "LOGGER_COUNTS", fields);
      }
      if (health && good()) { writeHealth(); nextHealth = nowMs() + HEALTH_MS; }
    }
    if (dirty && nowMs() - lastFlush >= FLUSH_MS) flushFile();
    if (good()) diagnosticsUsbTick(); // Logging batches have priority over USB.
    diaginventory::writerTick(good()); // Cache yields to logging and USB.
#if DIAG_USB_TEST_FIXTURE
    if (good() && fixtureBusy()) fixtureTick();
#endif
    vTaskDelay(diagnosticsUsbBusy() ? 1 : pdMS_TO_TICKS(20)); // Idle watchdog always runs.
  }
#if DIAG_USB_TEST_FIXTURE
  if (fixtureBusy()) fixtureFinish("logger_stopped");
  usbGateEnd("logger_stopped");
  portENTER_CRITICAL(&mux); usbGate.armed = UsbGate::None; portEXIT_CRITICAL(&mux);
#endif
  diagtransfer::writerOffline();
  diaginventory::writerOffline();
  diagnosticsUsbStop();
  if (fd >= 0) { // terminal error: release handle, never retry writes this boot
    int old = fd; fd = -1; ::close(old);
  }
  if (mounted) { SD_MMC.end(); mounted = false; }
  if (line) { heap_caps_free(line); line = nullptr; }
  portENTER_CRITICAL(&mux);
  accepting = false;
  portEXIT_CRITICAL(&mux);
  // Measure after all filesystem/formatter cleanup, while our own stack is valid.
  const uint32_t finalMargin = uxTaskGetStackHighWaterMark(nullptr);
  portENTER_CRITICAL(&mux);
  snapshot.stackMin = min(snapshot.stackMin, finalMargin);
  snapshot.stackFinalMargin = finalMargin;
#if DIAG_WRITER_STACK_PSRAM
  snapshot.writerLifecycle = WriterLifecycle::Parked;
#else
  snapshot.writerLifecycle = WriterLifecycle::Deleted; // Cleanup complete; RTOS exit follows.
#endif
  closeDone = true;
  portEXIT_CRITICAL(&mux);
  // Queue storage stays allocated: clock callbacks can still observe its disabled state.
#if DIAG_WRITER_STACK_PSRAM
  // Once per boot: retain the static TCB, PSRAM stack and task runtime state.
  // Never free an executing stack. A stray resume immediately parks again.
  for (;;) vTaskSuspend(nullptr);
#else
  vTaskDelete(nullptr);
#endif
}
} // namespace

bool diagnosticsHeaderValid(const char* line, uint32_t& generation) {
  return headerGeneration(line,generation); // Recovery acceptance is unchanged.
}

namespace diag {
bool record(const char* event, const char* fields, bool important) {
  Event item{};
  item.when = stamp(); item.important = important;
  if (strlen(event) >= sizeof(item.event) || strlen(fields) >= sizeof(item.fields)) {
    portENTER_CRITICAL(&mux); ++snapshot.truncated; ++snapshot.drops; portEXIT_CRITICAL(&mux);
    return false; // refuse a malformed/truncated diagnostic record
  }
  memcpy(item.event,event,strlen(event)+1); memcpy(item.fields,fields,strlen(fields)+1);
  portENTER_CRITICAL(&mux);
  if (!accepting || !queue) { portEXIT_CRITICAL(&mux); return false; }
  if (count >= QUEUE_COUNT || (!important && count >= QUEUE_COUNT - IMPORTANT_RESERVE)) {
    ++snapshot.drops;
    if (!important) ++snapshot.routineDrops;
    portEXIT_CRITICAL(&mux); return false;
  }
  queue[(head + count) % QUEUE_COUNT] = item; ++count;
  snapshot.highWater = max(snapshot.highWater, count);
  portEXIT_CRITICAL(&mux);
  return true;
}
} // namespace diag

void diagnosticsInitEarly() {
  if (initialized) return;
  initialized = true;
  diagnosticsUsbInit({usbStatus, usbBegin, closeFile, usbResume, usbEnd});
#if DIAG_ENABLED
  diag::initializeIdentityClock(); // Boot-counter NVS on the main task, never writer
  bootStamp = diag::stamp();
  queue = static_cast<Event*>(heap_caps_calloc(QUEUE_COUNT, sizeof(Event), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!queue) { disable("queue_allocation", ENOMEM); return; }
  portENTER_CRITICAL(&mux); accepting = true; snapshot.state = State::Starting; portEXIT_CRITICAL(&mux);
  if (!diag::identity.persistent) {
    USBSerial.printf("[LOG] boot counter unavailable; session=%s is nonpersistent\n", diag::identity.session);
    diag::record("BOOT_COUNTER_ERROR","persistent=false",true);
  }
#endif
}
void diagnosticsStart() {
#if DIAG_ENABLED
  if (started || !queue || !good()) return;
  started = true;
  captureStartup(StartupPoint::BeforeClock);
  diag::startClock(); // asynchronous; no wait for Wi-Fi or valid time
  captureStartup(StartupPoint::AfterClock);
  captureStartup(StartupPoint::BeforeWriter);
  portENTER_CRITICAL(&mux);
  snapshot.writerLifecycle = WriterLifecycle::Starting;
  portEXIT_CRITICAL(&mux);
  // Called from setup(), where USBSerial.begin() allocated the HWCDC interrupt.
  // Core 3.1.3 task writes flush the FIFO without the ISR sharing their TX mutex.
  // Keep this writer on that same core so a task flush cannot run concurrently
  // with the ISR filling a packet on the other core. Still a separate task.
  const BaseType_t writerCore = xPortGetCoreID();
#if DIAG_WRITER_STACK_PSRAM
  StackType_t* const stack = static_cast<StackType_t*>(
    heap_caps_malloc(WRITER_STACK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  // Fail closed if a different SDK changes the assumed memory placement.
  const bool placementOk = stack && esp_ptr_external_ram(stack) &&
    esp_ptr_internal(&writerTcb) && esp_ptr_byte_accessible(&writerTcb);
  const bool created = placementOk &&
    xTaskCreateStaticPinnedToCore(writerTask, "sd_logger", WRITER_STACK, nullptr,
                                 1, stack, &writerTcb, writerCore) != nullptr;
  if (!created) heap_caps_free(stack); // No task owns it on this path. Never free the TCB.
#else
  // Ordinary task stacks stay internal; do not retain a possibly stale task handle.
  const bool created =
    xTaskCreatePinnedToCore(writerTask, "sd_logger", WRITER_STACK, nullptr, 1, nullptr, writerCore) == pdPASS;
#endif
  if (!created) {
    portENTER_CRITICAL(&mux);
    snapshot.writerLifecycle = WriterLifecycle::CreateFailed;
    closeDone = true;
    portEXIT_CRITICAL(&mux);
    disable("writer_allocation", ENOMEM); // No internal fallback for the PSRAM experiment.
  }
#endif
}
void diagnosticsSetupComplete() {
#if DIAG_ENABLED
  setupFinished = true;
  diag::breadcrumb(false, diag::Phase::Idle);
  diag::record("SETUP_COMPLETE","result=ready",true);
#endif
}
bool diagnosticsHealthDue() {
#if DIAG_ENABLED
#if DIAG_TEST_HOOKS
  nvsStressMainTick(); // This existing background-tick entry is called only by the main task.
#endif
  uint64_t now = nowMs();
  if (now < nextHealthCapture) return false;
  nextHealthCapture = now + 1000;
  return true;
#else
  return false;
#endif
}
void diagnosticsUpdateHealth(const DiagnosticsHealth& value) {
  portENTER_CRITICAL(&mux);
  snapshot.health = value; snapshot.healthUp = nowMs();
  portEXIT_CRITICAL(&mux);
}
void diagnosticsPrintStatus() {
  Snapshot s = readSnapshot();
  const bool stackMeasured = s.stackMin != UINT32_MAX;
  const bool measured = s.internalMin != UINT32_MAX;
  if (!stackMeasured) s.stackMin = 0;
  if (!measured) {
    s.internalMin = s.internalLargest = s.dmaMin = s.dmaLargest = 0;
  }
  uint32_t queued;
  portENTER_CRITICAL(&mux); queued = count; portEXIT_CRITICAL(&mux);
  USBSerial.printf("[LOG] state=%s boot=%llu session=%s up_ms=%llu clock=%s setup=%u hooks=%u file_bytes=%llu generation=%lu newest=%lu archives=%lu card_bytes=%llu free_bytes=%llu queue=%u/%u high=%u drops=%u suppressed=%u truncated=%u error=%s errno=%d\n",
    stateName(s.state),static_cast<unsigned long long>(diag::identity.boot),diag::identity.session,
    static_cast<unsigned long long>(nowMs()),diag::qualityName(diag::stamp().quality),setupFinished,DIAG_TEST_HOOKS,
    static_cast<unsigned long long>(s.size),static_cast<unsigned long>(s.generation),static_cast<unsigned long>(s.newest),
    static_cast<unsigned long>(s.archives),static_cast<unsigned long long>(s.cardBytes),static_cast<unsigned long long>(s.freeBytes),
    queued,unsigned(QUEUE_COUNT),s.highWater,s.drops,s.routineDrops,s.truncated,*s.error ? s.error : "none",s.errorCode);
  USBSerial.printf("[LOG] measured=%u stack_min=%u internal_min=%u internal_largest=%u dma_min=%u dma_largest=%u writes=%u slow=%u write_max_us=%llu flush_max_us=%llu sd_max_us=%llu rotations=%u pruned=%u oversized=%u\n",
    measured,s.stackMin,s.internalMin,s.internalLargest,s.dmaMin,s.dmaLargest,s.writes,s.slowWrites,
    static_cast<unsigned long long>(s.writeMaxUs),static_cast<unsigned long long>(s.flushMaxUs),
    static_cast<unsigned long long>(s.sdMaxUs),s.rotations,s.pruned,s.oversized);
  // Values were captured by the writer itself. Never dereference a task handle here.
  const int stackUsed = stackMeasured ? int(WRITER_STACK - min(uint32_t(WRITER_STACK), s.stackMin)) : -1;
  const int finalMargin = s.stackFinalMargin == UINT32_MAX ? -1 : int(s.stackFinalMargin);
  USBSerial.printf("[LOG STACK] stack_mode=%s stack_bytes=%u placement_valid=%u stack_start=0x%lx stack_external=%d stack_local_external=%d tcb_internal=%d tcb_bytes=%u writer_lifecycle=%s stack_used_max=%d stack_final_margin=%d writer_core=%d\n",
    STACK_MODE, unsigned(WRITER_STACK), s.placementValid, static_cast<unsigned long>(s.stackStart),
    s.placementValid ? int(s.stackExternal) : -1, s.placementValid ? int(s.stackLocalExternal) : -1,
    s.placementValid ? int(s.tcbInternal) : -1, unsigned(sizeof(StaticTask_t)),
    lifecycleName(s.writerLifecycle), stackUsed, finalMargin, s.writerCore);
#if DIAG_TEST_HOOKS
  const NvsStress stress = readNvsStress();
  USBSerial.printf("[LOG TEST] nvs_active=%u summary_pending=%u nvs_writes=%u nvs_errors=%u sd_records=%u key_removed=%u first_nvs_ms=%llu last_nvs_ms=%llu first_sd_ms=%llu last_sd_ms=%llu\n",
    stress.active, stress.summaryPending, stress.nvsWrites, stress.nvsErrors, stress.sdRecords, stress.cleanupOk,
    static_cast<unsigned long long>(stress.firstNvsMs), static_cast<unsigned long long>(stress.lastNvsMs),
    static_cast<unsigned long long>(stress.firstSdMs), static_cast<unsigned long long>(stress.lastSdMs));
#endif
#if DIAG_USB_TEST_FIXTURE
  portENTER_CRITICAL(&mux); const FixtureStatus f = fixture; portEXIT_CRITICAL(&mux);
  USBSerial.printf("[LOG FIXTURE] enabled=1 active=%u archive=%08lu bytes=%lu result=%s errno=%d\n",
    unsigned(f.busy), (unsigned long)f.number, (unsigned long)f.bytes, f.result, f.error);
  printUsbGate();
#endif
  printStartupMemory();
  diagnosticsUsbCommand("log status");
}
bool diagnosticsUsbTransferActive() { return diagnosticsUsbBusy(); }
void diagnosticsUsbMainTick() {
  if (!writerRunning(readSnapshot().writerLifecycle)) diagnosticsUsbOfflineTick();
}
#if DIAG_TEST_HOOKS
namespace {
void watchdogTest(void*) {
  if (esp_task_wdt_add(nullptr) != ESP_OK) {
    USBSerial.println("[LOG TEST] watchdog registration failed"); vTaskDelete(nullptr); return;
  }
  diag::breadcrumb(false, diag::Phase::TestWatchdog, 1);
  // Real registered task that does not feed; sleeping leaves both idle tasks runnable.
  for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
}
#endif
bool diagnosticsStorageReady() { return usbStatus().ready; }
bool diagnosticsStorageClosing() { return closing(); }
bool diagnosticsCommand(const char* command) {
  if (logRetrievalCommand(command)) return true;
#if DIAG_USB_TEST_FIXTURE
  if (!strcmp(command,"log test usb off")) {
    if (diagnosticsUsbBusy()) {
      USBSerial.println("[LOG USB GATE] busy: use log abort, then disarm while idle"); return true;
    }
    portENTER_CRITICAL(&mux);
    usbGate.armed = usbGate.active = UsbGate::None; usbGate.result = "disarmed";
    portEXIT_CRITICAL(&mux);
    printUsbGate(); return true;
  }
  const bool queueTest = !strcmp(command,"log test queue");
  const bool pruneTest = !strncmp(command,"log test prune ",15);
  if (queueTest || pruneTest) {
    uint64_t number = 0;
    if (pruneTest && (strlen(command+15)>8 || !decimal(command+15,MAX_GENERATION,number) || !number)) {
      USBSerial.println("[LOG USB GATE] Use log test prune <fresh fixture number>"); return true;
    }
    const bool busy = diagnosticsUsbBusy();
    portENTER_CRITICAL(&mux);
    const bool owned = !pruneTest || (fixture.number == number && fixture.bytes == FILE_LIMIT && !strcmp(fixture.result,"ok"));
    const bool allowed = !busy && snapshot.state == State::Ready && !closeRequested && !fixture.busy && owned;
    if (allowed) {
      usbGate = UsbGateStatus{};
      usbGate.armed = queueTest ? UsbGate::Queue : UsbGate::Prune;
      usbGate.target = uint32_t(number); usbGate.result = "armed";
    }
    portEXIT_CRITICAL(&mux);
    if (!allowed) USBSerial.println("[LOG USB GATE] refused: busy, unavailable, or not a fixture created this boot");
    else printUsbGate();
    return true;
  }
  const bool createFixture = !strcmp(command, "log test file");
  const bool deleteFixture = !strncmp(command, "log test del ", 13);
  if (createFixture || deleteFixture) {
    uint64_t requestedNumber = 0;
    if (deleteFixture && (strlen(command + 13) > 8 ||
        !decimal(command + 13, MAX_GENERATION, requestedNumber) || !requestedNumber)) {
      USBSerial.println("[LOG FIXTURE] Use log test del <archive number>"); return true;
    }
    const bool usbBusy = diagnosticsUsbBusy();
    portENTER_CRITICAL(&mux);
    const bool available = !usbBusy && snapshot.state == State::Ready && !closeRequested && !fixture.busy;
    if (available) {
      fixture.busy = fixture.requested = true;
      fixture.bytes = 0; fixture.number = uint32_t(requestedNumber);
      fixture.deleting = deleteFixture; fixture.result = "queued"; fixture.error = 0;
    }
    portEXIT_CRITICAL(&mux);
    USBSerial.println(!available ? "[LOG FIXTURE] unavailable or busy" : deleteFixture ?
      "[LOG FIXTURE] queued: verify every test byte before deleting; wait for result=deleted" :
      "[LOG FIXTURE] queued: synthetic 2 MiB archive; wait for result=ok");
    return true;
  }
#endif
  if (diagnosticsUsbCommand(command)) return true;
#if DIAG_TEST_HOOKS
  if (!strcmp(command, "log test nvs")) { startNvsStress(); return true; }
  if (!strcmp(command, "log test nvs stop")) { finishNvsStress("serial_stop"); return true; }
  // Keep the stress run separate from destructive faults and limit changes.
  const NvsStress stress = readNvsStress();
  if (!strncmp(command, "log test ", 9) && (stress.active || stress.summaryPending)) {
    USBSerial.println("[LOG TEST] Finish NVS stress before another hook"); return true;
  }
  if (diag::clockTest(command)) { USBSerial.println("[LOG TEST] clock hook applied"); return true; }
  if (!strcmp(command,"log test panic")) {
    diag::record("TEST_PANIC","source=serial",true);
    diag::breadcrumb(false,diag::Phase::TestPanic,1);
    abort(); // no panic handler writes to SD
  }
  if (!strcmp(command,"log test watchdog")) {
    diag::record("TEST_WATCHDOG","source=serial",true);
    if (xTaskCreatePinnedToCore(watchdogTest,"diag_wdt",2048,nullptr,1,nullptr,1) != pdPASS)
      USBSerial.println("[LOG TEST] watchdog task allocation failed");
    return true;
  }
  struct Choice { const char* name; Test test; };
  static const Choice choices[] = {
    {"log test small",Test::Small},{"log test normal",Test::Normal},{"log test rotate",Test::Rotate},
    {"log test rename",Test::Rename},{"log test header",Test::Header},{"log test partial",Test::Partial},
    {"log test space",Test::Space},{"log test full",Test::Full}
  };
  for (const Choice& choice : choices) {
    if (strcmp(command,choice.name)) continue;
    portENTER_CRITICAL(&mux);
    const bool available = snapshot.state == State::Ready && pendingTest == Test::None && !closeRequested;
    if (available) pendingTest = choice.test;
    portEXIT_CRITICAL(&mux);
    USBSerial.println(available ? "[LOG TEST] hook queued" : "[LOG TEST] unavailable or busy");
    return true;
  }
#endif
  if (!strncmp(command,"log ",4)) {
    USBSerial.println("[LOG] Use log status. Use log list, log get current, log get <generation>, or log abort; fault hooks require DIAG_TEST_HOOKS=1.");
    return true;
  }
  return false;
}
bool diagnosticsClose(bool deepSleep, uint32_t waitMs) {
  logRetrievalExit(deepSleep ? "deep_sleep" : "shutdown");
#if DIAG_ENABLED
  if (!initialized) return true;
#if DIAG_TEST_HOOKS
  finishNvsStress("close"); // Main-task-only cleanup of the deliberate dummy-key test.
#endif
  diag::Stamp when = diag::stamp();
  diag::breadcrumb(false,deepSleep ? diag::Phase::Sleep : diag::Phase::Shutdown);
  portENTER_CRITICAL(&mux);
  if (!closeRequested) {
    closeStamp = when; sleepRequested = deepSleep; closeRequested = true; accepting = false;
  }
  bool active = writerRunning(snapshot.writerLifecycle);
  portEXIT_CRITICAL(&mux);
  if (!active) return true;
  const uint64_t end = nowMs() + min(waitMs, uint32_t(500));
  while (nowMs() < end) {
    portENTER_CRITICAL(&mux); bool done = closeDone; portEXIT_CRITICAL(&mux);
    if (done) return readSnapshot().state == State::Closed;
    delay(1);
  }
  // No serial output here: its own wait could exceed the caller's close budget.
  return false;
#else
  return true;
#endif
}
