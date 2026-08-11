#pragma once

#include <Arduino.h>
#include <lvgl.h>

struct ImageFetcherConfig {
  uint16_t screenWidth;
  uint16_t screenHeight;
  lv_obj_t* screen1;
  lv_obj_t* screen2;
  lv_obj_t* screen3;
  lv_obj_t* inclinometerScreen;
  lv_obj_t* imgScreen2Background;
};

void imageFetcherInit(const ImageFetcherConfig& config);
void imageFetcherLoop();
bool requestLatestImage();

// True from the moment a fetch is queued until it completes, fails, or times out.
// Callers in loop() use this to defer anything that blocks for seconds at a time —
// every millisecond spent elsewhere comes straight out of the image loading budget.
bool imageFetcherIsBusy();

#ifdef __cplusplus
extern "C" {
#endif

void buttonLatest_event_handler(lv_event_t* e);
void buttonNew_event_handler(lv_event_t* e);
void buttonBack_event_handler(lv_event_t* e);
void screen2_event_handler(lv_event_t* e);

#ifdef __cplusplus
}
#endif

