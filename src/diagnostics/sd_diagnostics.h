#pragma once
#include <stdint.h>
#include "diagnostics_config.h"

// All calls from task context. No caller touches SD; only the writer owns it.
struct DiagnosticsHealth {
  bool wifi = false;
  bool mqtt = false;
  int16_t rssi = 0;
  bool usbPower = false;
  bool battery = false;
  uint16_t batteryMv = 0;
  uint8_t screen = 0; // 0=other, 1=dashboard, 2=media, 3=G-meter, 4=inclinometer
  bool live = false;
  bool image = false;
  bool moving = false;
};
void diagnosticsInitEarly(); // setup, internal stack: NVS, RTC, queue, clock
void diagnosticsStart();     // setup only, same core as USBSerial.begin(); does not wait for SD
void diagnosticsSetupComplete();
bool diagnosticsHealthDue(); // main task only: health rate limit and optional NVS stress tick
void diagnosticsUpdateHealth(const DiagnosticsHealth& health);
void diagnosticsPrintStatus(); // snapshots only; never waits for SD
bool diagnosticsCommand(const char* command); // Bounded USB commands and fault hooks
bool diagnosticsUsbTransferActive();
void diagnosticsUsbMainTick(); // Control-only fallback when writer is disabled/off
bool diagnosticsClose(bool deepSleep, uint32_t waitMs = 500); // bounded, idempotent
