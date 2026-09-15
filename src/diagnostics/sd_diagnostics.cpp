#include "sd_diagnostics.h"
#include "diagnostics_internal.h"
#include "../../pin_config.h"
#include <Arduino.h>
#include "HWCDC.h"
#include <SD_MMC.h>
#include <esp_vfs_fat.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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
constexpr size_t WRITER_STACK = 6144; // ESP-IDF task API uses bytes, internal RAM.
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
  uint32_t internalMin = UINT32_MAX, internalLargest = UINT32_MAX;
  uint32_t dmaMin = UINT32_MAX, dmaLargest = UINT32_MAX;
  uint64_t healthUp = 0;
  DiagnosticsHealth health;
};
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Snapshot snapshot;
Event* queue = nullptr;
uint32_t head = 0, count = 0;
bool initialized = false, started = false;
bool accepting = false, closeRequested = false, closeDone = false;
bool sleepRequested = false;
TaskHandle_t writerHandle = nullptr;
diag::Stamp bootStamp{}, closeStamp{};
uint64_t nextHealthCapture = 0;
bool setupFinished = false;

#if DIAG_TEST_HOOKS
enum class Test : uint8_t { None, Small, Normal, Rotate, Rename, Header, Partial, Space, Full };
Test pendingTest = Test::None;
Test pauseAt = Test::None;
bool fullWrites = false, fakeSpace = false;
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
    char path[80]; archivePath(files.oldest, path, sizeof(path));
    if (unlink(path)) { disable("archive_delete", errno); return false; }
    portENTER_CRITICAL(&mux); ++snapshot.pruned; portEXIT_CRITICAL(&mux);
    vTaskDelay(1);
    if (!inventory(files)) return false;
  }
  portENTER_CRITICAL(&mux);
  snapshot.archives = files.count; snapshot.newest = files.any ? files.newest : 0;
  portEXIT_CRITICAL(&mux);
  diag::breadcrumb(true, diag::Phase::Idle);
  return true;
}
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
  return rawWrite(line, length);
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
  fd = open(CURRENT, O_WRONLY | O_CREAT | O_EXCL, 0666);
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
  return rawWrite(line, length);
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
  // Maker BSP uses these same one-bit pins. No formatting or rail changes.
  if (!SD_MMC.setPins(SDMMC_CLK, SDMMC_CMD, SDMMC_DATA) ||
      !SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 3)) {
    memorySample(esp_timer_get_time() - begin);
    disable("mount_failed_or_no_card", EIO); return false;
  }
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
      if (end) { end[1] = 0; valid = headerGeneration(line, saved); }
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
    fd = open(CURRENT, O_WRONLY | O_APPEND);
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
void writerTask(void*) {
  line = static_cast<char*>(heap_caps_malloc(LINE_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!line) disable("formatter_allocation", ENOMEM);
  else if (openStorage()) {
    USBSerial.printf("[LOG] ready file=/logs/current.log boot=%llu queue_bytes=%u stack_bytes=%u core=0\n",
      static_cast<unsigned long long>(diag::identity.boot), unsigned(QUEUE_COUNT * sizeof(Event)), unsigned(WRITER_STACK));
  }
  nextHealth = nowMs() + HEALTH_MS;
  nextClock = nowMs();
  while (good() && fd >= 0) {
    memorySample();
    if (closing()) {
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
#endif
    if (nowMs() >= nextClock) { diag::clockPoll(); nextClock = nowMs() + 1000; }
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
    vTaskDelay(pdMS_TO_TICKS(20)); // always yield, including fault hooks
  }
  if (fd >= 0) { // terminal error: release handle, never retry writes this boot
    int old = fd; fd = -1; ::close(old);
  }
  if (mounted) { SD_MMC.end(); mounted = false; }
  if (line) { heap_caps_free(line); line = nullptr; }
  portENTER_CRITICAL(&mux);
  accepting = false; closeDone = true; writerHandle = nullptr;
  portEXIT_CRITICAL(&mux);
  // Queue storage stays allocated: clock callbacks can still observe its disabled state.
  vTaskDelete(nullptr);
}
} // namespace

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
#if DIAG_ENABLED
  diag::initializeIdentityClock(); // NVS only here, never writer
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
  diag::startClock(); // asynchronous; no wait for Wi-Fi or valid time
  // ESP-IDF allocates ordinary task stacks internally even with PSRAM enabled.
  if (xTaskCreatePinnedToCore(writerTask, "sd_logger", WRITER_STACK, nullptr, 1, &writerHandle, 0) != pdPASS) {
    disable("writer_allocation", ENOMEM);
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
  const bool measured = s.stackMin != UINT32_MAX;
  if (!measured) {
    s.stackMin = s.internalMin = s.internalLargest = s.dmaMin = s.dmaLargest = 0;
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
bool diagnosticsCommand(const char* command) {
  if (!strcmp(command,"log status")) { diagnosticsPrintStatus(); return true; }
#if DIAG_TEST_HOOKS
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
    USBSerial.println("[LOG] Use log status. File retrieval is not implemented; fault hooks require DIAG_TEST_HOOKS=1.");
    return true;
  }
  return false;
}
bool diagnosticsClose(bool deepSleep, uint32_t waitMs) {
#if DIAG_ENABLED
  if (!initialized) return true;
  diag::Stamp when = diag::stamp();
  diag::breadcrumb(false,deepSleep ? diag::Phase::Sleep : diag::Phase::Shutdown);
  portENTER_CRITICAL(&mux);
  if (!closeRequested) {
    closeStamp = when; sleepRequested = deepSleep; closeRequested = true; accepting = false;
  }
  bool active = writerHandle != nullptr;
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
