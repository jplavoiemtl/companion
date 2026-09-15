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
void diagnosticsStart();     // after board power initialization; does not wait for SD
void diagnosticsSetupComplete();
bool diagnosticsHealthDue(); // rate limits main-task snapshot reads to once per second
void diagnosticsUpdateHealth(const DiagnosticsHealth& health);
void diagnosticsPrintStatus(); // snapshots only; never waits for SD
bool diagnosticsCommand(const char* command); // log status and compile-time test hooks
bool diagnosticsClose(bool deepSleep, uint32_t waitMs = 500); // bounded, idempotent
