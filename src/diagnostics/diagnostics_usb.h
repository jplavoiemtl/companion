#pragma once
#include <stdint.h>
#include <stddef.h>
#include "diagnostics_config.h"

// Main posts commands; tick/stop/beforePrune are called only by the SD writer.
struct DiagnosticsUsbStatus {
  uint64_t boot, uptime, size, cardBytes, freeBytes;
  uint32_t newest, files, drops, queued, capacity;
  const char* logger;
  bool ready, closing;
};
struct DiagnosticsUsbHooks {
  DiagnosticsUsbStatus (*status)();
  bool (*begin)(const char* name); // Persist USB_GET_BEGIN before snapshot.
  bool (*pause)();
  bool (*resume)();
  void (*end)(const char* name, uint64_t bytes, uint64_t ms, const char* result);
};
void diagnosticsUsbInit(const DiagnosticsUsbHooks& hooks);
bool diagnosticsUsbCommand(const char* command);
bool diagnosticsUsbBusy();
bool diagnosticsUsbPaused(); // Writer only.
void diagnosticsUsbTick();
void diagnosticsUsbStop(); // Shutdown or terminal logger failure: no USB output.
void diagnosticsUsbBeforePrune(uint32_t generation);
// Only when the writer lifecycle is terminal/off; never accesses SD in that state.
void diagnosticsUsbOfflineTick();

#if DIAG_USB_TEST_FIXTURE
// Writer-only view, for one-shot bench injection after real data has been sent.
bool diagnosticsUsbTestProgress(bool& current, uint32_t& number, uint64_t& bytes);
#endif
