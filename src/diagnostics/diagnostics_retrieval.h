#pragma once
#include <stdint.h>

// Main-task only, including commands, touch, media admission and power callbacks.
// HTTP lifecycle runs on a separate worker; mode remains main-owned.
bool logRetrievalActive(); // STARTING, ACTIVE and STOPPING all exclude media.
bool logRetrievalCommand(const char* command);
void logRetrievalTick();
void logRetrievalTouch(); // Valid panel touch only; no progress/link-based reset.
void logRetrievalExit(const char* reason); // Literal reason, non-waiting, no serial output.
