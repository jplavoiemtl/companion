#pragma once
#include "diagnostics_reader.h"
namespace diaginventory {
struct Entry { diagreader::FileEntry file; diagtime::Ends times; uint64_t retryAt=0; };
static_assert(sizeof(Entry)<=64,"inventory entry budget");
struct View {
  const Entry* entries = nullptr;
  size_t count = 0;
  uint64_t at = 0;
  bool valid = false, stale = true;
  const char* error = "pending";
  int slot = -1;
};
// Lifecycle worker: allocation before requesting writer work; stop never waits.
bool start(uint64_t generation);
void stop();
bool dispose(uint64_t generation); // false until writer quiescence and zero HTTP pins
// HTTP task: immutable snapshot pin, never SD work.
View pin(uint64_t now);
void unpin(const View& view);
// SD writer only. Tick before USB to preempt scanning, and after normal batches.
void writerOnline();
void writerOffline();
void writerTick(bool storageReady);
void writerChanged();
void writerPreempt(); // closes a scan before the USB adapter opens its reader
} // namespace diaginventory
