#pragma once

#include <Arduino.h>
#include <lvgl.h>

// ============================================================================
// Live Video Feed Module
// ============================================================================
// Ported from the home_panel project. See docs/live_video_feed_port.md for the
// measurement history and the reasoning behind each choice.
//
// Displays a live view of the entrance camera by polling Frigate through the
// remote HTTPS image proxy. This board runs on a phone hotspot and cannot reach
// Frigate on the LAN, hence the proxy route.
//
// Behaviour:
// - The Screen1 camera button starts a 60 second feed on Screen2
// - A motion push over MQTT shows the still for 10 seconds, then the same feed
// - When the 60 seconds elapse the panel returns to the previous screen
// - Still-image requests are ignored while the feed is running
//
// Measured on hardware: ~1.9 fps. http dominates at ~360 ms per frame over
// cellular; decode is 46 ms and blit 109 ms. The 155 ms of decode+blit puts the
// hard ceiling at 6.4 fps, so the network is the limit, not the board.
//
// Known trade-off: the fetch is blocking, so IMU sampling drops from ~50 Hz to
// ~2 Hz for the duration of a feed. Accepted deliberately - the inclinometer and
// G-meter are not being read while someone is looking at the front door. A
// non-blocking prefetching client would fix it and roughly halve the frame
// period, at the cost of hand-rolled HTTP over a shared TLS socket.
// ============================================================================

struct VideoStreamConfig {
  uint16_t screenWidth;           // LVGL horizontal res at ROT_NONE (368)
  uint16_t screenHeight;          // LVGL vertical res at ROT_NONE (448)
  lv_obj_t* screen1;              // Home screen, returned to when the feed ends
  lv_obj_t* screenVideo;          // Image screen (shared with the still images)
  lv_obj_t* imgVideoBackground;   // Full-screen image widget on that screen
};

// Initialize. Call after ui_init().
void videoStreamInit(const VideoStreamConfig& config);

// Periodic processing - drives the feed. Call in loop().
void videoStreamLoop();

// Start the feed. Returns false if it could not be started.
bool videoStreamStart();

// Stop and release buffers. Safe to call when already stopped.
void videoStreamStop();

// True while a feed is running. The image fetcher uses this to stand down;
// without it an MQTT image push mid-feed frees buffers still being rendered.
bool videoStreamActive();
