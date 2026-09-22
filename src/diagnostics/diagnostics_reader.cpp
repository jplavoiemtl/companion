#include "diagnostics_reader.h"
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/task.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

namespace diagreader {
namespace {
constexpr char ROOT[] = "/sdcard/logs/";
portMUX_TYPE sessionMux = portMUX_INITIALIZER_UNLOCKED;
Hooks hooks{};
ReaderState state;
Accepted pending;
uint64_t nextGeneration = 0, retainedGeneration = 0;
bool reserved = false, invalidated = false, abortRequested = false;
Published snapshot{0, false, "none"};
uint64_t milliseconds() { return esp_timer_get_time() / 1000; }
// Aliases are private to the writer implementation, never writable by an adapter.
Buffers*& buffers = state.buffers;
FileEntry*& entries = state.entries;
size_t& entryCount = state.entryCount;
size_t& pendingBytes = state.pendingBytes;
int& reader = state.reader;
bool& isCurrent = state.isCurrent;
bool& paused = state.paused;
bool& begun = state.begun;
uint32_t& fileNumber = state.fileNumber;
uint32_t& crc = state.crc;
uint64_t& fileSize = state.fileSize;
uint64_t& sentBytes = state.sentBytes;
uint64_t& startedAt = state.startedAt;
uint64_t& lastProgress = state.lastProgress;
char (&filename)[24] = state.filename;
} // namespace
void init(const Hooks& value) { hooks = value; }
Status status() { return hooks.status ? hooks.status() : Status{}; }
const ReaderState& view() { return state; }
bool reserve(Request request, uint32_t number, uint64_t at) {
  portENTER_CRITICAL(&sessionMux);
  // Never wrap/reuse an identity; theoretical exhaustion is a busy refusal.
  if (reserved || nextGeneration == UINT64_MAX) {
    portEXIT_CRITICAL(&sessionMux); return false;
  }
  retainedGeneration = ++nextGeneration;
  reserved = true; invalidated = false;
  pending = {request, number, at, retainedGeneration, false};
  portEXIT_CRITICAL(&sessionMux); return true;
}
bool busy() {
  portENTER_CRITICAL(&sessionMux); const bool value = reserved;
  portEXIT_CRITICAL(&sessionMux); return value;
}
void requestAbort() {
  portENTER_CRITICAL(&sessionMux); abortRequested = true; portEXIT_CRITICAL(&sessionMux);
}
bool abortPending() {
  portENTER_CRITICAL(&sessionMux); const bool value = abortRequested;
  portEXIT_CRITICAL(&sessionMux); return value;
}
bool takeAbort() {
  portENTER_CRITICAL(&sessionMux); const bool value = abortRequested;
  abortRequested = false; portEXIT_CRITICAL(&sessionMux); return value;
}
Accepted takeRequest() {
  portENTER_CRITICAL(&sessionMux);
  Accepted value = pending; value.abort = abortRequested;
  pending = {}; abortRequested = false;
  portEXIT_CRITICAL(&sessionMux); return value;
}
void invalidate(uint64_t generation) {
  portENTER_CRITICAL(&sessionMux);
  if (reserved && retainedGeneration == generation) invalidated = true;
  portEXIT_CRITICAL(&sessionMux);
}
bool release(uint64_t generation) {
  // Writer, or exclusive terminal-writer fallback: adapter relinquished its reference.
  portENTER_CRITICAL(&sessionMux);
  const bool matches = reserved && retainedGeneration == generation;
  portEXIT_CRITICAL(&sessionMux);
  if (!matches || reader >= 0 || paused) return false;
  // Invalidation rejects progress, NOT this matching release acknowledgement.
  if (buffers) heap_caps_free(buffers);
  if (entries) heap_caps_free(entries);
  buffers = nullptr; entries = nullptr; pendingBytes = 0;
  portENTER_CRITICAL(&sessionMux);
  reserved = false; invalidated = false; retainedGeneration = 0;
  portEXIT_CRITICAL(&sessionMux); return true;
}
void publish() {
  portENTER_CRITICAL(&sessionMux);
  snapshot.bytes = sentBytes; snapshot.paused = paused;
  portEXIT_CRITICAL(&sessionMux);
}
Published published() {
  portENTER_CRITICAL(&sessionMux); const Published value = snapshot;
  portEXIT_CRITICAL(&sessionMux); return value;
}
bool parseNumber(const char* text, uint32_t& n) {
  if (!*text) return false;
  n = 0;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9' || n > (99999999u - (*text - '0')) / 10) return false;
    n = n * 10 + (*text - '0');
  }
  return true;
}
bool archiveName(const char* name, uint32_t& number) {
  if (strlen(name) != 20 || strncmp(name, "archive-", 8) || strcmp(name + 16, ".log")) return false;
  char digits[9]; memcpy(digits, name + 8, 8); digits[8] = 0;
  return parseNumber(digits, number);
}
void nameFor(const FileEntry& file, char* output, size_t capacity) {
  if (file.current) snprintf(output, capacity, "current.log");
  else snprintf(output, capacity, "archive-%08lu.log", static_cast<unsigned long>(file.number));
}
uint32_t updateCrc(uint32_t value, const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    value ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit) value = (value >> 1) ^ (0xedb88320u & (0u - (value & 1u)));
  }
  return value;
}
bool closeReaderAndResume(bool recordEnd, const char* result, bool keepEvent) {
  bool ok = true;
  if (reader >= 0) { ok = ::close(reader) == 0; reader = -1; }
  if (paused) { paused = false; ok = hooks.resume() && ok; }
  publish();
  if (begun && recordEnd) hooks.end(filename, sentBytes, milliseconds() - startedAt, result);
  if (!keepEvent) {
    begun = false;
    portENTER_CRITICAL(&sessionMux); snapshot.result = result; portEXIT_CRITICAL(&sessionMux);
  }
  return ok;
}
const char* stopReason(const char* (*transportStop)()) {
  const auto s = hooks.status();
  if (s.closing) return "shutdown";
  if (!s.ready) return "logger_failed";
  if (const char* reason = transportStop()) return reason;
  if (abortPending()) return "aborted";
  if (s.queued * 2 >= s.capacity) return "logger_busy";
  const uint64_t now = milliseconds();
  if (isCurrent && now - startedAt >= CURRENT_MS) return "timeout";
  if (now - lastProgress >= STALL_MS) return "stalled";
  return nullptr;
}
const char* captureList(const char* (*transportStop)()) {
  entries = static_cast<FileEntry*>(heap_caps_calloc(MAX_ENTRIES + 1, sizeof(FileEntry), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!entries) return "memory";
  DIR* dir = opendir(ROOT);
  if (!dir) return "read_failed";
  size_t seen = 0; entryCount = 0; const char* failure = nullptr;
  for (;;) {
    if ((failure = stopReason(transportStop))) break;
    errno = 0; dirent* item = readdir(dir);
    if (!item) { if (errno) failure = "read_failed"; break; }
    if (++seen > MAX_ENTRIES) { failure = "directory_limit"; break; }
    const bool current = !strcmp(item->d_name, "current.log");
    uint32_t n = 0;
    if (current || archiveName(item->d_name, n)) {
      char path[64]; snprintf(path, sizeof(path), "%s%s", ROOT, item->d_name);
      struct stat st{};
      if (stat(path, &st)) { failure = "read_failed"; break; }
      if (S_ISREG(st.st_mode) && st.st_size >= 0) entries[entryCount++] = {uint64_t(st.st_size), n, current};
    }
    if (!(seen % 8)) vTaskDelay(1);
  }
  if (closedir(dir) && !failure) failure = "read_failed";
  return failure;
}
const char* start(const Accepted& accepted, const char* (*transportStop)()) {
  portENTER_CRITICAL(&sessionMux);
  const bool valid = reserved && retainedGeneration == accepted.generation && !invalidated;
  portEXIT_CRITICAL(&sessionMux);
  if (!valid) return "aborted";
  isCurrent = accepted.request == Request::Current; fileNumber = accepted.number;
  startedAt = lastProgress = accepted.at; sentBytes = fileSize = 0; crc = 0xffffffff;
  begun = paused = false; pendingBytes = 0; publish();
  buffers = static_cast<Buffers*>(heap_caps_calloc(1, sizeof(Buffers), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!buffers) return "memory";
  if (const char* reason = stopReason(transportStop)) return reason;
  if (accepted.request == Request::List) return captureList(transportStop);
  nameFor({0,accepted.number,isCurrent}, filename, sizeof(filename));
  char path[64]; snprintf(path, sizeof(path), "%s%s", ROOT, filename);
  struct stat st{};
  if (stat(path, &st)) return errno == ENOENT ? "not_found" : "read_failed";
  if (!S_ISREG(st.st_mode) || st.st_size < 0) return "not_found";
  if (accepted.request != Request::HttpArchive) {
    if (!hooks.begin(filename)) return "logger_failed";
    begun = true;
  }
  if (isCurrent) {
    paused = true; publish();
    if (!hooks.pause()) return "logger_failed";
  }
  reader = open(path, O_RDONLY);
  if (reader < 0) return "read_failed";
  if (fstat(reader, &st) || st.st_size < 0) return "read_failed";
  fileSize = st.st_size; return nullptr;
}
const char* readChunk() {
  portENTER_CRITICAL(&sessionMux);
  const bool valid = reserved && !invalidated;
  portEXIT_CRITICAL(&sessionMux);
  if (!valid) return "aborted";
  if (!pendingBytes) {
    const size_t wanted = fileSize-sentBytes < CHUNK ? size_t(fileSize-sentBytes) : CHUNK;
    const ssize_t got = read(reader,buffers->raw,wanted);
    if (got <= 0) return "read_failed";
    pendingBytes = size_t(got);
  }
  return nullptr;
}
bool progress(uint64_t generation, bool body) {
  portENTER_CRITICAL(&sessionMux);
  const bool valid = reserved && retainedGeneration == generation && !invalidated;
  portEXIT_CRITICAL(&sessionMux);
  if (!valid) return false;
  // Writer-only: invalidate and release also execute on writer; main posts abort.
  lastProgress = milliseconds(); // Includes BEGIN, END and LIST, exactly as USB did.
  if (body) {
    crc = updateCrc(crc,buffers->raw,pendingBytes); sentBytes += pendingBytes;
    pendingBytes = 0; publish();
  }
  return true;
}
void terminalClock() { lastProgress = milliseconds(); }
bool beforePrune(uint32_t number) {
  return reader >= 0 && !isCurrent && number == fileNumber;
}
} // namespace diagreader

namespace diagreader {
bool progressBytes(uint64_t generation, size_t bytes, uint64_t at) {
  portENTER_CRITICAL(&sessionMux);
  const bool valid = reserved && retainedGeneration == generation && !invalidated;
  portEXIT_CRITICAL(&sessionMux);
  if (!valid || !bytes || bytes > pendingBytes) return false;
  crc = updateCrc(crc,buffers->raw,bytes);
  sentBytes += bytes; pendingBytes -= bytes;
  if (pendingBytes) memmove(buffers->raw,buffers->raw+bytes,pendingBytes);
  lastProgress = at; publish(); return true;
}
}
