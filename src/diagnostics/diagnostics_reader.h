#pragma once
#include <stdint.h>
#include <stddef.h>

// Main submits; only the SD writer starts/reads/cleans up. No transport calls here.
namespace diagreader {
constexpr size_t CHUNK = 144, SCRATCH = 241, MAX_ENTRIES = 256;
constexpr uint64_t STALL_MS = 5000, CURRENT_MS = 120000;
enum class Request : uint8_t { None, List, Current, Archive, HttpArchive };
struct Status {
  uint64_t boot, uptime, size, cardBytes, freeBytes;
  uint32_t newest, files, drops, queued, capacity;
  const char* logger;
  bool ready, closing;
};
struct Hooks {
  Status (*status)();
  bool (*begin)(const char* name);
  bool (*pause)();
  bool (*resume)();
  void (*end)(const char* name, uint64_t bytes, uint64_t ms, const char* result);
};
struct FileEntry { uint64_t size; uint32_t number; bool current; };
// Same combined PSRAM allocation as USB before extraction; scratch is adapter-owned.
struct Buffers { char wire[SCRATCH]; uint8_t raw[CHUNK]; };
struct ReaderState {
  Buffers* buffers = nullptr;
  FileEntry* entries = nullptr;
  size_t entryCount = 0, pendingBytes = 0;
  int reader = -1;
  bool isCurrent = false, paused = false, begun = false;
  uint32_t fileNumber = 0, crc = 0xffffffff;
  uint64_t fileSize = 0, sentBytes = 0, startedAt = 0, lastProgress = 0;
  char filename[24] = {};
};
struct Accepted {
  Request request = Request::None;
  uint32_t number = 0;
  uint64_t at = 0, generation = 0;
  bool abort = false;
};
struct Published { uint64_t bytes; bool paused; const char* result; };
void init(const Hooks& hooks);
// Thread-safe main/writer command exchange; zero is never a valid reservation.
bool reserve(Request request, uint32_t number, uint64_t at);
bool busy();
void requestAbort(); // Also preserves the USB idle-abort acknowledgement.
bool abortPending();
bool takeAbort(); // Consume only abort; offline fallback must leave requests alone.
Accepted takeRequest(); // Writer consumes main's command.
// Lifecycle/ack methods are writer-only while it runs. HTTP posts acknowledgements.
// After terminal writer publication, main may call release for an already-closed
// HTTP reader; this fallback does no SD work and cannot overlap writer access.
void invalidate(uint64_t generation); // Reject future progress, retain reservation/storage.
bool release(uint64_t generation); // Writer/terminal fallback, ONLY after adapter releases its buffer.
Status status(); // Locked logger snapshot, no SD work.
Published published(); // Thread-safe status snapshot.
// Writer-only view, never publish these pointers to another task without a handoff.
const ReaderState& view();
void publish();
bool parseNumber(const char* text, uint32_t& n);
bool archiveName(const char* name, uint32_t& number);
void nameFor(const FileEntry& file, char* output, size_t capacity);
const char* stopReason(const char* (*transportStop)());
const char* start(const Accepted& request, const char* (*transportStop)());
const char* readChunk();
bool progress(uint64_t generation, bool body); // USB calls after a complete wire line.
bool progressBytes(uint64_t generation, size_t bytes, uint64_t at); // Writer consumes HTTP ack.
void terminalClock(); // Error-delivery deadline; does not revive an invalid generation.
bool closeReaderAndResume(bool recordEnd, const char* result, bool keepEvent = false);
bool beforePrune(uint32_t number); // Writer checks this before unlink.
uint32_t updateCrc(uint32_t value, const uint8_t* data, size_t length);
} // namespace diagreader
