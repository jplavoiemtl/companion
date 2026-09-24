#pragma once
#include <lvgl.h>
// Main/LVGL only. Install the event override at UI setup; arm after diagnostics setup.
void logRetrievalUiInit();
void logRetrievalUiTick();
void logRetrievalUiPowerDown();
void logRetrievalUiEntryEvent(lv_event_t* event);
