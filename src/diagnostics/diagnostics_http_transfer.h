#pragma once
#include "diagnostics_reader.h"
// Bounded mailbox. Only writer calls reader lifecycle/SD methods.
namespace diagtransfer {
struct View {
  uint64_t id=0, generation=0, size=0, offset=0, started=0, closedAt=0, cancelledAt=0, readerAt=0;
  uint64_t writerBytes=0;
  uint32_t writerCrc=0;
  size_t length=0;
  bool reserved=false, metadata=false, closed=false, released=false;
  const char* result="none";
  uint8_t data[diagreader::CHUNK] = {};
};
struct Result {
  uint64_t id=0, expected=0, bytes=0, started=0, firstBody=0, lastBody=0;
  uint64_t maxGap=0, cancelledAt=0, closedAt=0, releasedAt=0;
  uint64_t writerBytes=0;
  uint32_t number=0, crc=0, writerCrc=0;
  const char* crcCheck="unavailable";
  const char* result="none";
};
// HTTP task: IDs never wrap. Every acknowledgement checks identity.
uint64_t request(uint32_t number, uint64_t now);
View view(); // Copies bytes under lock; no shared pointer escapes.
bool progress(uint64_t id, uint64_t bytes, uint64_t at);
void cancel(uint64_t id, const char* reason);
void release(uint64_t id, const Result& result);
Result last();
void compareWriter(Result& result, const View& closed); // No locks; uses immutable close snapshot.
bool busy();
// Writer dispatches its single shared request queue to this adapter.
void writerOnline();
void writerOffline(); // Refuse new reservations before terminal writer cleanup.
const char* failure();
void accept(const diagreader::Accepted& request);
void tick();
void stop(const diagreader::Accepted& pending);
void beforePrune(uint32_t number);
// Main ONLY after writer lifecycle is terminal; no SD operations or waits.
void offlineTick();
void cancelAll(const char* reason); // Main/lifecycle; no wait.
}
