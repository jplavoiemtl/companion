#include "diagnostics_usb.h"
#include "HWCDC.h"
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
#include <stdlib.h>

extern HWCDC USBSerial;
namespace {
constexpr size_t WIRE = 240, CHUNK = 144, MAX_ENTRIES = 256;
constexpr uint64_t STALL_MS = 5000, CURRENT_MS = 120000;
constexpr char ROOT[] = "/sdcard/logs/";
enum class Request : uint8_t { None, List, Current, Archive };
enum class Phase : uint8_t { Idle, Begin, Data, End, List, Terminal };
struct FileEntry { uint64_t size; uint32_t number; bool current; };
struct Buffers { char wire[WIRE + 1]; uint8_t raw[CHUNK]; };
portMUX_TYPE usbMux = portMUX_INITIALIZER_UNLOCKED;
DiagnosticsUsbHooks hooks{};
Request pending = Request::None;
uint32_t requestedNumber = 0;
uint64_t acceptedAt = 0;
bool reserved = false, abortRequested = false, statusRequested = false;
const char* controlError = nullptr; // Coalesced bounded reply slot, never heap strings.
uint64_t controlAt = 0;
uint64_t publishedBytes = 0;
bool publishedPaused = false;
const char* lastResult = "none";
// Remaining state belongs exclusively to the writer (or terminal/off fallback).
Phase phase = Phase::Idle;
Buffers* buffers = nullptr;
FileEntry* entries = nullptr;
size_t entryCount = 0, entryIndex = 0, pendingBytes = 0;
int reader = -1;
bool isCurrent = false, paused = false, begun = false;
uint32_t fileNumber = 0, dataLines = 0, crc = 0xffffffff;
uint64_t fileSize = 0, sentBytes = 0, startedAt = 0, lastProgress = 0;
char filename[24] = {};
const char* terminalReason = nullptr;
const char* terminalPhase = "idle";
// Retain the last transfer-write observation; status/error writes do not overwrite it.
size_t lastLineBytes = 0;
int lastTxFree = -1, lastWriteBytes = -1;
const char* phaseName(Phase value) {
  switch (value) {
    case Phase::Begin: return "begin";
    case Phase::Data: return "data";
    case Phase::End: return "end";
    case Phase::List: return "list";
    case Phase::Terminal: return "terminal";
    default: return "idle";
  }
}
uint8_t statusPart = 0;
uint64_t statusAt = 0;

uint64_t milliseconds() { return esp_timer_get_time() / 1000; }
void publish() {
  portENTER_CRITICAL(&usbMux);
  publishedBytes = sentBytes; publishedPaused = paused;
  portEXIT_CRITICAL(&usbMux);
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
void queueError(const char* reason) {
  portENTER_CRITICAL(&usbMux);
  if (!controlError || strcmp(controlError, "aborted") || !strcmp(reason, "aborted")) {
    controlError = reason; controlAt = milliseconds();
  }
  portEXIT_CRITICAL(&usbMux);
}
// Never spin on space. Caller retries on another writer turn and rechecks limits.
// Core 3.1.3 serial mutex/timeout are unchanged; space is not a reservation.
const char* stopReason();
int sendLine(const char* bytes, bool transfer = false) {
  const size_t length = strlen(bytes);
  if (transfer) { lastLineBytes = length; lastTxFree = lastWriteBytes = -1; }
  if (!length || length > WIRE) return -1;
  if (!USBSerial.isConnected()) return -1;
  const int space = USBSerial.availableForWrite();
  if (transfer) lastTxFree = space;
  if (space < int(length)) return 0;
  if (!USBSerial.isConnected()) return -1;
  if (transfer && stopReason()) return -2;
  const size_t written = USBSerial.write(reinterpret_cast<const uint8_t*>(bytes), length);
  if (transfer) lastWriteBytes = int(written);
  return written == length ? 1 : -1;
}
void release() {
  if (buffers) heap_caps_free(buffers);
  if (entries) heap_caps_free(entries);
  buffers = nullptr; entries = nullptr; phase = Phase::Idle; pendingBytes = 0;
  portENTER_CRITICAL(&usbMux); reserved = false; portEXIT_CRITICAL(&usbMux);
}
bool closeReaderAndResume(bool recordEnd, const char* result, bool keepEvent = false) {
  bool ok = true;
  if (reader >= 0) { ok = ::close(reader) == 0; reader = -1; }
  if (paused) { paused = false; ok = hooks.resume() && ok; }
  publish();
  if (begun && recordEnd) hooks.end(filename, sentBytes, milliseconds() - startedAt, result);
  if (!keepEvent) {
    begun = false;
    portENTER_CRITICAL(&usbMux); lastResult = result; portEXIT_CRITICAL(&usbMux);
  }
  return ok;
}
void finishError(const char* reason, bool reply = true) {
  if (!strcmp(reason,"aborted")) {
    portENTER_CRITICAL(&usbMux); abortRequested = false; portEXIT_CRITICAL(&usbMux);
  }
  terminalPhase = phaseName(phase);
  const bool ok = closeReaderAndResume(strcmp(reason,"shutdown") != 0, reason);
  terminalReason = ok ? reason : "logger_failed";
  pendingBytes = 0;
  // Cleanup happens before any error output, including failed read-open.
  if (!reply || !USBSerial.isConnected()) { release(); return; }
  phase = Phase::Terminal; lastProgress = milliseconds();
}
const char* stopReason() {
  const auto s = hooks.status();
  if (s.closing) return "shutdown";
  if (!s.ready) return "logger_failed";
  if (!USBSerial.isConnected()) return "disconnected";
  portENTER_CRITICAL(&usbMux); bool abort = abortRequested; portEXIT_CRITICAL(&usbMux);
  if (abort) return "aborted";
  if (s.queued * 2 >= s.capacity) return "logger_busy";
  const uint64_t now = milliseconds();
  if (isCurrent && now - startedAt >= CURRENT_MS) return "timeout";
  if (now - lastProgress >= STALL_MS) return "stalled";
  return nullptr;
}
uint32_t updateCrc(uint32_t value, const uint8_t* data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    value ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit) value = (value >> 1) ^ (0xedb88320u & (0u - (value & 1u)));
  }
  return value;
}
void encode64(const uint8_t* data, size_t length, char* out) {
  constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for (size_t i = 0; i < length; i += 3) {
    const uint32_t v = (uint32_t(data[i]) << 16) | (i+1 < length ? uint32_t(data[i+1]) << 8 : 0) | (i+2 < length ? data[i+2] : 0);
    *out++ = alphabet[(v >> 18) & 63]; *out++ = alphabet[(v >> 12) & 63];
    *out++ = i+1 < length ? alphabet[(v >> 6) & 63] : '=';
    *out++ = i+2 < length ? alphabet[v & 63] : '=';
  }
  *out = 0;
}
bool captureList() {
  entries = static_cast<FileEntry*>(heap_caps_calloc(MAX_ENTRIES + 1, sizeof(FileEntry), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!entries) { finishError("memory"); return false; }
  DIR* dir = opendir(ROOT);
  if (!dir) { finishError("read_failed"); return false; }
  size_t seen = 0; entryCount = entryIndex = 0; const char* failure = nullptr;
  for (;;) {
    if ((failure = stopReason())) break;
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
  if (failure) { finishError(failure, strcmp(failure,"shutdown") && strcmp(failure,"disconnected")); return false; }
  return true;
}
void start(Request request, uint32_t number, uint64_t accepted) {
  isCurrent = request == Request::Current; fileNumber = number;
  startedAt = lastProgress = accepted; sentBytes = fileSize = 0; dataLines = 0; crc = 0xffffffff;
  begun = paused = false; terminalReason = nullptr; pendingBytes = 0;
  terminalPhase = "idle"; lastLineBytes = 0; lastTxFree = lastWriteBytes = -1; publish();
  buffers = static_cast<Buffers*>(heap_caps_calloc(1, sizeof(Buffers), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!buffers) { finishError("memory"); return; }
  if (const char* reason = stopReason()) { finishError(reason, strcmp(reason,"shutdown") && strcmp(reason,"disconnected")); return; }
  if (request == Request::List) { if (captureList()) phase = Phase::List; return; }
  nameFor({0,number,isCurrent}, filename, sizeof(filename));
  char path[64]; snprintf(path, sizeof(path), "%s%s", ROOT, filename);
  struct stat st{};
  if (stat(path, &st)) { finishError(errno == ENOENT ? "not_found" : "read_failed"); return; }
  if (!S_ISREG(st.st_mode) || st.st_size < 0) { finishError("not_found"); return; }
  if (!hooks.begin(filename)) { finishError("logger_failed"); return; }
  begun = true;
  if (isCurrent) {
    // Mark before calling pause: a flush/close failure still takes cleanup.
    paused = true; publish();
    if (!hooks.pause()) { finishError("logger_failed"); return; }
  }
  reader = open(path, O_RDONLY);
  if (reader < 0) { finishError("read_failed"); return; }
  if (fstat(reader, &st) || st.st_size < 0) { finishError("read_failed"); return; }
  fileSize = st.st_size; phase = Phase::Begin;
}
void controlTick() {
  bool wantStatus; const char* error; uint64_t errorAt;
  portENTER_CRITICAL(&usbMux);
  wantStatus = statusRequested; statusRequested = false;
  error = controlError; errorAt = controlAt;
  portEXIT_CRITICAL(&usbMux);
  if (wantStatus && !statusPart) { statusPart = 1; statusAt = milliseconds(); }
  char wire[WIRE + 1];
  if (error) {
    snprintf(wire,sizeof(wire),"\n@@ERR reason=%s\n",error);
    if (sendLine(wire) != 0 || milliseconds() - errorAt >= STALL_MS) {
      portENTER_CRITICAL(&usbMux);
      if (controlError == error && controlAt == errorAt) controlError = nullptr;
      portEXIT_CRITICAL(&usbMux);
    }
    return;
  }
  if (!statusPart) return;
  const auto s = hooks.status();
  if (statusPart == 1) {
    snprintf(wire,sizeof(wire),"\n@@STATUS boot=%llu up_ms=%llu logger=%s current_size=%llu newest=%lu drops=%lu card_mib=%llu free_mib=%llu files=%lu\n",
      (unsigned long long)s.boot,(unsigned long long)s.uptime,s.logger,(unsigned long long)s.size,
      (unsigned long)s.newest,(unsigned long)s.drops,(unsigned long long)(s.cardBytes >> 20),
      (unsigned long long)(s.freeBytes >> 20),(unsigned long)s.files);
  } else {
    portENTER_CRITICAL(&usbMux);
    const bool active = reserved, pause = publishedPaused;
    const uint64_t bytes = publishedBytes; const char* result = lastResult;
    portEXIT_CRITICAL(&usbMux);
    snprintf(wire,sizeof(wire),"\n@@USB active=%u paused=%u bytes=%llu result=%s queue=%lu/%lu current_limit_ms=%llu stall_ms=%llu\n",
      active,pause,(unsigned long long)bytes,result,(unsigned long)s.queued,(unsigned long)s.capacity,
      (unsigned long long)CURRENT_MS,(unsigned long long)STALL_MS);
  }
  const int sent = sendLine(wire);
  if (sent > 0) statusPart = statusPart == 1 ? 2 : 0;
  else if (sent < 0 || milliseconds() - statusAt >= STALL_MS) statusPart = 0;
}
} // namespace

void diagnosticsUsbInit(const DiagnosticsUsbHooks& value) { hooks = value; }
bool diagnosticsUsbBusy() {
  portENTER_CRITICAL(&usbMux); const bool value = reserved; portEXIT_CRITICAL(&usbMux); return value;
}
bool diagnosticsUsbPaused() { return paused; }
bool diagnosticsUsbCommand(const char* command) {
  if (!strcmp(command,"log status")) {
    portENTER_CRITICAL(&usbMux); statusRequested = true; portEXIT_CRITICAL(&usbMux); return true;
  }
  if (!strcmp(command,"log abort")) {
    portENTER_CRITICAL(&usbMux); abortRequested = true; portEXIT_CRITICAL(&usbMux); return true;
  }
  if (strncmp(command,"log ",4)) return false;
  if (diagnosticsUsbBusy()) { queueError("busy"); return true; }
  Request request; uint32_t number = 0;
  if (!strcmp(command,"log list")) request = Request::List;
  else if (!strcmp(command,"log get current")) request = Request::Current;
  else if (!strncmp(command,"log get ",8)) {
    if (!parseNumber(command+8,number)) { queueError("invalid_argument"); return true; }
    request = Request::Archive;
  } else return false; // Stage 1 hooks retain their parser when idle.
  if (!hooks.status || !hooks.status().ready) { queueError("unavailable"); return true; }
  portENTER_CRITICAL(&usbMux);
  pending = request; requestedNumber = number; acceptedAt = milliseconds(); reserved = true;
  portEXIT_CRITICAL(&usbMux);
  return true;
}
void diagnosticsUsbTick() {
  if (!hooks.status) return;
  if (hooks.status().closing) { diagnosticsUsbStop(); return; }
  // Explicit abort is acknowledged only after cleanup, including queued requests.
  portENTER_CRITICAL(&usbMux);
  bool abort = abortRequested; abortRequested = false;
  Request request = pending; uint32_t number = requestedNumber; uint64_t accepted = acceptedAt;
  pending = Request::None;
  portEXIT_CRITICAL(&usbMux);
  if (abort) {
    if (phase != Phase::Idle || request != Request::None) finishError("aborted");
    else queueError("aborted");
  } else if (request != Request::None) start(request,number,accepted);
  if (phase != Phase::Idle && phase != Phase::Terminal) {
    if (const char* reason = stopReason()) finishError(reason, strcmp(reason,"shutdown") && strcmp(reason,"disconnected"));
  }
  controlTick();
  for (unsigned work = 0; work < 4 && phase != Phase::Idle; ++work) {
    if (phase == Phase::Terminal) {
      // No append pause remains here. Missing error delivery expires independently.
      char wire[WIRE+1];
      if (!strcmp(terminalReason,"stalled")) {
        snprintf(wire,sizeof(wire),
          "\n@@ERR reason=stalled phase=%s bytes=%llu line_bytes=%u tx_free=%d write_bytes=%d\n",
          terminalPhase,(unsigned long long)sentBytes,unsigned(lastLineBytes),lastTxFree,lastWriteBytes);
      } else snprintf(wire,sizeof(wire),"\n@@ERR reason=%s\n",terminalReason);
      if (sendLine(wire) != 0 || milliseconds()-lastProgress >= STALL_MS) release();
      break;
    }
    if (const char* reason = stopReason()) {
      finishError(reason, strcmp(reason,"shutdown") && strcmp(reason,"disconnected")); break;
    }
    char* wire = buffers->wire;
    if (phase == Phase::Begin) snprintf(wire,WIRE+1,"\n@@BEGIN version=1 name=%s size=%llu\n",filename,(unsigned long long)fileSize);
    else if (phase == Phase::Data) {
      if (sentBytes == fileSize) {
        if (!closeReaderAndResume(false,"ok",true)) { finishError("logger_failed"); break; }
        phase = Phase::End;
      } else if (!pendingBytes) {
        const size_t wanted = fileSize-sentBytes < CHUNK ? size_t(fileSize-sentBytes) : CHUNK;
        const ssize_t got = read(reader,buffers->raw,wanted);
        if (got <= 0) { finishError("read_failed"); break; }
        pendingBytes = size_t(got);
      }
      if (phase == Phase::Data) {
        const int prefix = snprintf(wire,WIRE+1,"\n@@D %lu ",(unsigned long)(dataLines+1));
        encode64(buffers->raw,pendingBytes,wire+prefix); strcat(wire,"\n");
      }
    }
    if (phase == Phase::End) snprintf(wire,WIRE+1,"\n@@END name=%s bytes=%llu lines=%lu crc32=%08lX\n",
      filename,(unsigned long long)sentBytes,(unsigned long)dataLines,(unsigned long)(crc ^ 0xffffffff));
    if (phase == Phase::List) {
      if (entryIndex == entryCount) snprintf(wire,WIRE+1,"\n@@LIST_END count=%u\n",unsigned(entryCount));
      else {
        char name[24]; nameFor(entries[entryIndex],name,sizeof(name));
        snprintf(wire,WIRE+1,"\n@@FILE name=%s size=%llu\n",name,(unsigned long long)entries[entryIndex].size);
      }
    }
    // Space checking itself takes a driver mutex. Recheck all abort conditions
    // on every turn; the single write has the existing 100 ms driver timeout.
    const int sent = sendLine(wire,true);
    if (sent == -2) {
      const char* reason = stopReason();
      if (!reason) reason = "stalled";
      finishError(reason, strcmp(reason,"shutdown") && strcmp(reason,"disconnected")); break;
    }
    if (sent < 0) { finishError("stalled"); break; }
    if (!sent) break;
    lastProgress = milliseconds();
    if (phase == Phase::Begin) phase = Phase::Data;
    else if (phase == Phase::Data) {
      crc = updateCrc(crc,buffers->raw,pendingBytes); sentBytes += pendingBytes;
      pendingBytes = 0; ++dataLines; publish();
    } else if (phase == Phase::End) {
      closeReaderAndResume(true,"ok"); release();
    }
    else if (phase == Phase::List && entryIndex++ == entryCount) release();
  }
}
void diagnosticsUsbStop() {
  portENTER_CRITICAL(&usbMux);
  pending = Request::None; abortRequested = statusRequested = false; controlError = nullptr;
  portEXIT_CRITICAL(&usbMux);
  statusPart = 0;
  if (phase != Phase::Idle || paused || reader >= 0) closeReaderAndResume(false,"shutdown");
  release();
}
void diagnosticsUsbBeforePrune(uint32_t number) {
  if (reader >= 0 && !isCurrent && number == fileNumber) finishError("pruned");
}
void diagnosticsUsbOfflineTick() {
  if (!hooks.status) return;
  portENTER_CRITICAL(&usbMux);
  bool abort = abortRequested; abortRequested = false;
  portEXIT_CRITICAL(&usbMux);
  if (abort) queueError("aborted");
  controlTick(); // No filesystem access while the writer is absent/parked.
}
