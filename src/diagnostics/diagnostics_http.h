#pragma once
#include <stdint.h>
namespace diaghttp {
// Main task; all return without waiting for HTTP, SD, or lifecycle completion.
bool interfaceAllowed();
bool start();
void stop();
bool activate();
bool stopped();
const char* failure();
const char* stage();
uint64_t activity();
bool idleExpired(uint64_t now, uint64_t panelActivity, uint64_t limit);
void report();
void notice(bool stuck); // Main/LVGL only, survives screen changes.
} // namespace diaghttp
