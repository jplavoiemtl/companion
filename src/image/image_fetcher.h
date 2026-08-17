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
// Fetch the "latest" endpoint. Pass fromNotification = true when the request originates from
// an MQTT push rather than a button press: such a request is dropped if an image already
// reached the screen within the last few seconds, which suppresses the server's own
// notification echoing back to the unit that just pressed "new".
bool requestLatestImage(bool fromNotification = false);

// True from the moment a fetch is queued until it completes, fails, or times out.
// Callers in loop() use this to defer anything that blocks for seconds at a time —
// every millisecond spent elsewhere comes straight out of the image loading budget.
bool imageFetcherIsBusy();

// Shared TLS client.
//
// This board cannot afford two. mbedTLS needs contiguous ~16 KB buffers, and
// with roughly 48 KB free but a largest free block of only ~32 KB, a second
// WiFiClientSecure fails its handshake with "SSL - Memory allocation failed".
// This one is a global constructed at startup, when the heap is still
// unfragmented, which is why it succeeds where a later one does not.
//
// Safe to share because only one module fetches at a time: the video module and
// this one each stand down while the other is active.
class WiFiClientSecure;
WiFiClientSecure* imageFetcherSecureClient();

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

