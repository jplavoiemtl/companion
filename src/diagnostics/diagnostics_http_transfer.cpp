#include "diagnostics_http_transfer.h"
#include "diagnostics_inventory.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <string.h>
namespace diagtransfer {
namespace {
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
View box;
Result lastResult;
uint64_t nextId=0, acknowledged=0, progressAt=0;
const char* cancellation=nullptr;
// Writer only, then main's terminal-writer fallback after SD cleanup publication.
uint64_t readerGeneration=0;
bool started=false, online=false;
uint64_t nowMs() { return esp_timer_get_time()/1000; }
const char* transportStop() {
  portENTER_CRITICAL(&mux); const char* value=cancellation; portEXIT_CRITICAL(&mux);
  return value;
}
void close(const char* reason) {
  if (!readerGeneration || view().closed) return;
  diagreader::invalidate(readerGeneration);
  const bool ok=diagreader::closeReaderAndResume(false,reason);
  const auto& writerState=diagreader::view();
  portENTER_CRITICAL(&mux);
  box.pausedAt=writerState.pausedAt; box.readerClosedAt=writerState.readerClosedAt;
  box.resumedAt=writerState.resumedAt;
  box.writerBytes=writerState.sentBytes; box.writerCrc=writerState.crc ^ 0xffffffff;
  box.closed=true; box.closedAt=nowMs(); box.result=ok ? reason : "logger_failed";
  if (box.released && lastResult.id==box.id) {
    lastResult.closedAt=box.closedAt;
    if (strcmp(box.result,"ok")) lastResult.result=box.result;
    compareWriter(lastResult,box);
  }
  if (strcmp(box.result,"ok") && !cancellation) { cancellation=box.result; box.cancelledAt=nowMs(); }
  portEXIT_CRITICAL(&mux);
}
void finishRelease() {
  const View s=view();
  if (!s.reserved || !s.closed || !s.released || !readerGeneration) return;
  if (!diagreader::release(readerGeneration)) return; // Retain; main escalates stuck STOPPING.
  readerGeneration=0; started=false;
  portENTER_CRITICAL(&mux); box.reserved=false; portEXIT_CRITICAL(&mux);
}
}
uint64_t request(uint32_t number, uint64_t now, bool current) {
  portENTER_CRITICAL(&mux);
  if (!online || box.reserved || nextId==UINT64_MAX) { portEXIT_CRITICAL(&mux); return 0; }
  // Lock order transfer -> reader; writer never holds reader lock while taking transfer lock.
  if (!diagreader::reserve(current ? diagreader::Request::HttpCurrent : diagreader::Request::HttpArchive,number,now)) {
    portEXIT_CRITICAL(&mux); return 0;
  }
  box=View{}; box.reserved=true; box.id=++nextId; box.started=now;
  acknowledged=progressAt=0; cancellation=nullptr;
  const uint64_t id=box.id;
  portEXIT_CRITICAL(&mux); return id;
}
View view() { portENTER_CRITICAL(&mux); View s=box; portEXIT_CRITICAL(&mux); return s; }
bool busy() { portENTER_CRITICAL(&mux); const bool value=box.reserved; portEXIT_CRITICAL(&mux); return value; }
bool progress(uint64_t id, uint64_t bytes, uint64_t at) {
  portENTER_CRITICAL(&mux);
  const bool ok=box.reserved && box.id==id && !box.closed && !box.released &&
    !cancellation && bytes>acknowledged && bytes<=box.offset+box.length;
  if (ok) { acknowledged=bytes; progressAt=at; }
  portEXIT_CRITICAL(&mux); return ok;
}
void cancel(uint64_t id, const char* reason) {
  portENTER_CRITICAL(&mux);
  if (box.reserved && box.id==id && !cancellation) { cancellation=reason; box.cancelledAt=nowMs(); }
  portEXIT_CRITICAL(&mux);
}
void cancelAll(const char* reason) { const View s=view(); if(s.reserved) cancel(s.id,reason); }
void release(uint64_t id, const Result& result) {
  portENTER_CRITICAL(&mux);
  // Cancellation rejects progress, never the matching transport release.
  if (box.reserved && box.id==id && !box.released) {
    box.released=true; lastResult=result;
    if (box.closed) { lastResult.closedAt=box.closedAt; if (strcmp(box.result,"ok")) lastResult.result=box.result; }
    compareWriter(lastResult,box);
    if (!cancellation && strcmp(result.result,"ok")) { cancellation=result.result; box.cancelledAt=nowMs(); }
  }
  portEXIT_CRITICAL(&mux);
}
void compareWriter(Result& result, const View& closed) {
  if (!closed.closed || closed.id!=result.id) { result.crcCheck="unavailable"; return; }
  result.pausedAt=closed.pausedAt; result.readerClosedAt=closed.readerClosedAt; result.resumedAt=closed.resumedAt;
  result.appends=closed.pausedAt ? (closed.resumedAt ? "resumed" : "resume_failed") : "unpaused";
  result.writerBytes=closed.writerBytes; result.writerCrc=closed.writerCrc;
  // An in-flight send may succeed after writer cancellation invalidates its ack.
  // Unequal prefix lengths are expected in that race, not evidence of corruption.
  if (result.bytes!=result.writerBytes) result.crcCheck="prefix_diff";
  else if (result.crc==result.writerCrc) result.crcCheck="match";
  else {
    result.crcCheck="mismatch";
    if (!strcmp(result.result,"ok")) result.result="crc_mismatch";
  }
}
Result last() { portENTER_CRITICAL(&mux); Result s=lastResult; portEXIT_CRITICAL(&mux); return s; }
void writerOnline() { portENTER_CRITICAL(&mux); online=true; portEXIT_CRITICAL(&mux); }
void writerOffline() {
  portENTER_CRITICAL(&mux); online=false;
  if (box.reserved && !cancellation) { cancellation="shutdown"; box.cancelledAt=nowMs(); }
  portEXIT_CRITICAL(&mux);
}
const char* failure() {
  portENTER_CRITICAL(&mux);
  const bool stuck=box.reserved && box.released && lastResult.id==box.id && nowMs()-lastResult.releasedAt>=10000;
  portEXIT_CRITICAL(&mux); return stuck ? "reader_release" : nullptr;
}
void accept(const diagreader::Accepted& request) {
  readerGeneration=request.generation;
  portENTER_CRITICAL(&mux); box.generation=readerGeneration; box.readerAt=nowMs(); portEXIT_CRITICAL(&mux);
  if(request.abort) cancelAll("aborted");
  diaginventory::writerPreempt();
  if (const char* reason=diagreader::start(request,transportStop)) { close(reason); return; }
  started=true;
  const auto& r=diagreader::view();
  portENTER_CRITICAL(&mux); box.size=r.fileSize; box.metadata=true; portEXIT_CRITICAL(&mux);
}
void tick() {
  const View s=view();
  if (!s.reserved || !readerGeneration) return;
  if (s.closed) { finishRelease(); return; }
  // Consume positive transport acknowledgements before checking no-progress timeout.
  uint64_t bytes,at;
  portENTER_CRITICAL(&mux); bytes=acknowledged; at=progressAt; portEXIT_CRITICAL(&mux);
  const auto& r=diagreader::view();
  if (bytes>r.sentBytes && !diagreader::progressBytes(readerGeneration,size_t(bytes-r.sentBytes),at)) {
    close("progress_refused"); return;
  }
  if (const char* reason=diagreader::stopReason(transportStop)) { close(reason); return; }
  if (!started) { close("not_started"); return; }
  if (r.sentBytes==r.fileSize) { close("ok"); finishRelease(); return; }
  if (s.length && bytes<s.offset+s.length) return;
  if (const char* reason=diagreader::readChunk()) { close(reason); return; }
  portENTER_CRITICAL(&mux);
  box.offset=r.sentBytes; box.length=r.pendingBytes;
  memcpy(box.data,r.buffers->raw,r.pendingBytes);
  portEXIT_CRITICAL(&mux);
}
void stop(const diagreader::Accepted& pending) {
  if(pending.request==diagreader::Request::HttpArchive || pending.request==diagreader::Request::HttpCurrent) {
    readerGeneration=pending.generation;
    cancelAll("shutdown");
    // Initialize state; stop guard prevents SD acquisition.
    diagreader::start(pending,transportStop);
  }
  if (busy()) { cancelAll("shutdown"); close("shutdown"); finishRelease(); }
}
void beforePrune(uint32_t number) {
  if (busy() && diagreader::beforePrune(number)) { cancelAll("pruned"); close("pruned"); }
}
void offlineTick() { finishRelease(); }
}
