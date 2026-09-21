#pragma once
#include <stdint.h>
#include <stddef.h>
#include "diagnostics_config.h"
#include "diagnostics_reader.h"

// Main posts commands; tick/stop/beforePrune are called only by the SD writer.
using DiagnosticsUsbStatus = diagreader::Status;
using DiagnosticsUsbHooks = diagreader::Hooks;
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
