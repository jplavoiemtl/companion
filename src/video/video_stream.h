#pragma once

#include <Arduino.h>
#include <lvgl.h>

// ============================================================================
// Live Video Feed Module - PHASE 1 MEASUREMENT SPIKE
// ============================================================================
// Ported from the home_panel project. See doc/live_video_feed_port.md.
//
// This build is a measurement spike, not the finished feature. It exists to
// answer two questions that decide whether the feature is viable here:
//
//   1. How long does ESP32_JPEG take to decode a 640x360 frame at 240 MHz?
//   2. Which orientation approach is cheaper?
//        A) rotate during decode, display at LV_DISP_ROT_NONE
//        B) decode unrotated, let LVGL rotate at LV_DISP_ROT_90
//
// The burst runs 30 frames: the first 15 with approach A, the second 15 with
// approach B, then reports both averages. Splitting one burst rather than
// flashing twice keeps network conditions identical between the two, which
// matters over cellular where latency varies a lot.
//
// The orientation flips visibly halfway through the burst. That is expected.
//
// Deliberately NOT included yet (Phase 2 work):
//   - the prefetching HTTP client; this uses a plain blocking GET so that
//     network behaviour does not contaminate the decode and blit numbers
//   - the motion -> still -> video chain
// ============================================================================

struct VideoStreamConfig {
  uint16_t screenWidth;           // LVGL horizontal res at ROT_NONE (368)
  uint16_t screenHeight;          // LVGL vertical res at ROT_NONE (448)
  lv_obj_t* screen1;              // Home screen, returned to when the burst ends
  lv_obj_t* screenVideo;          // Image screen (shared with the still images)
  lv_obj_t* imgVideoBackground;   // Full-screen image widget on that screen
};

// Initialize. Call after ui_init().
void videoStreamInit(const VideoStreamConfig& config);

// Periodic processing - drives the burst. Call in loop().
void videoStreamLoop();

// Start the measurement burst. Returns false if it could not be started.
bool videoStreamStart();

// Stop and release buffers. Safe to call when already stopped.
void videoStreamStop();

// True while a burst is running. The image fetcher uses this to stand down;
// without it an MQTT image push mid-burst frees buffers still being rendered.
bool videoStreamActive();
