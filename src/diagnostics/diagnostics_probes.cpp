#include "diagnostics_probes.h"
#include "HWCDC.h"
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>

extern HWCDC USBSerial;

namespace {
constexpr uint64_t SAMPLE_INTERVAL_US = 10000;
constexpr size_t WINDOW_COUNT = static_cast<size_t>(ProbeWindow::Count);
const char* const names[WINDOW_COUNT] = {"mqtt_connect", "image_https", "live_tls", "live", "normal"};

struct WindowState {
  bool active = false;
  uint32_t generation = 0;
  int64_t startUs = 0;
  int64_t lastSampleUs = 0;
  uint64_t maxGapUs = 0;
  uint64_t maxScanUs = 0;
  size_t lowestLargest = 0;
  uint32_t samples = 0;  // Periodic only; begin/end readings are separate.
  bool skipFirstImu = true;  // First interval may span preceding excluded work.
  float imuMinHz = 0;
  double imuSumHz = 0;
  uint32_t imuSamples = 0;
};
WindowState windows[WINDOW_COUNT];
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
esp_timer_handle_t sampler = nullptr;
bool initialized = false;
bool samplerReady = false;

void includeReading(WindowState& state, size_t largest, int64_t when, uint64_t scanUs) {
  if (largest < state.lowestLargest) state.lowestLargest = largest;
  const uint64_t gap = static_cast<uint64_t>(when - state.lastSampleUs);
  if (gap > state.maxGapUs) state.maxGapUs = gap;
  if (scanUs > state.maxScanUs) state.maxScanUs = scanUs;
  state.lastSampleUs = when;
}

void sampleInternalHeap(void*) {
  uint32_t generations[WINDOW_COUNT] = {};
  bool selected[WINDOW_COUNT] = {};
  bool any = false;
  portENTER_CRITICAL(&stateMux);
  for (size_t i = 0; i < WINDOW_COUNT; ++i) {
    selected[i] = windows[i].active;
    generations[i] = windows[i].generation;
    any |= selected[i];
  }
  portEXIT_CRITICAL(&stateMux);
  if (!any) return;

  // Task-dispatched esp_timer callback, never an ISR. One scan serves overlapping
  // windows. No heap scan, allocation or serial output under stateMux.
  const int64_t scanStart = esp_timer_get_time();
  const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&stateMux);
  for (size_t i = 0; i < WINDOW_COUNT; ++i) {
    WindowState& state = windows[i];
    // A slow scan must not land in a later window with the same name.
    if (selected[i] && state.active && state.generation == generations[i]) {
      includeReading(state, largest, now, static_cast<uint64_t>(now - scanStart));
      ++state.samples;
    }
  }
  portEXIT_CRITICAL(&stateMux);
}
}  // namespace

void diagnosticsProbeInit() {
  if (initialized) return;
  initialized = true;
  esp_timer_create_args_t args = {};
  args.callback = sampleInternalHeap;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "diag_probe";
  args.skip_unhandled_events = true;  // No catch-up scan burst after a delay.
  esp_err_t error = esp_timer_create(&args, &sampler);
  if (error == ESP_OK) error = esp_timer_start_periodic(sampler, SAMPLE_INTERVAL_US);
  if (error != ESP_OK) {
    if (sampler) esp_timer_delete(sampler);
    sampler = nullptr;
    USBSerial.printf("[PROBE] sampler=unavailable error=%d; boundary readings only\n", error);
    return;  // Measurement failure must not change companion operation.
  }
  samplerReady = true;
  USBSerial.println("[PROBE] sampler=ready interval_ms=10 caps=INTERNAL task=esp_timer");
}

void diagnosticsProbeBegin(ProbeWindow window) {
  const size_t index = static_cast<size_t>(window);
  if (index >= WINDOW_COUNT) return;
  if (window != ProbeWindow::Normal) diagnosticsProbeNormalUpdate(false);
  const int64_t scanStart = esp_timer_get_time();
  const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&stateMux);
  WindowState& state = windows[index];
  const uint32_t generation = state.generation + 1;
  state = WindowState{};
  state.generation = generation;
  state.startUs = now;
  state.lastSampleUs = now;
  state.lowestLargest = largest;
  state.maxScanUs = static_cast<uint64_t>(now - scanStart);
  state.active = true;
  portEXIT_CRITICAL(&stateMux);
}

void diagnosticsProbeEnd(ProbeWindow window) {
  const size_t index = static_cast<size_t>(window);
  if (index >= WINDOW_COUNT) return;
  // Exact since-boot API, not a sampled minimum or a window-local value.
  const size_t minInternalBoot = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  const int64_t scanStart = esp_timer_get_time();
  const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  const int64_t now = esp_timer_get_time();
  WindowState snapshot;
  portENTER_CRITICAL(&stateMux);
  WindowState& state = windows[index];
  if (!state.active) {
    portEXIT_CRITICAL(&stateMux);
    return;
  }
  // A timer scan may finish during this boundary scan. Avoid backwards time.
  const int64_t endUs = now > state.lastSampleUs ? now : state.lastSampleUs;
  includeReading(state, largest, endUs, static_cast<uint64_t>(now - scanStart));
  snapshot = state;
  state.active = false;
  portEXIT_CRITICAL(&stateMux);

  char imuSummary[96] = {};
  if (window == ProbeWindow::Normal) {
    snprintf(imuSummary, sizeof(imuSummary), " imu_n=%u imu_min_hz=%.2f imu_avg_hz=%.2f",
             static_cast<unsigned>(snapshot.imuSamples), snapshot.imuMinHz,
             snapshot.imuSamples ? snapshot.imuSumHz / snapshot.imuSamples : 0.0);
  }
  // One summary per window; never print from the timer task.
  USBSerial.printf("[PROBE] window=%s run=%u ms=%llu heap_min_boot=%u largest_min=%u interval_ms=10 samples=%u gap_max_us=%llu scan_max_us=%llu timer=%s%s\n",
                   names[index], static_cast<unsigned>(snapshot.generation),
                   static_cast<unsigned long long>((snapshot.lastSampleUs - snapshot.startUs) / 1000),
                   static_cast<unsigned>(minInternalBoot), static_cast<unsigned>(snapshot.lowestLargest),
                   static_cast<unsigned>(snapshot.samples), static_cast<unsigned long long>(snapshot.maxGapUs),
                   static_cast<unsigned long long>(snapshot.maxScanUs), samplerReady ? "on" : "off", imuSummary);
}

void diagnosticsProbeNormalUpdate(bool eligible) {
  const size_t normal = static_cast<size_t>(ProbeWindow::Normal);
  bool active;
  int64_t start;
  portENTER_CRITICAL(&stateMux);
  active = windows[normal].active;
  start = windows[normal].startUs;
  for (size_t i = 0; i < normal; ++i) {
    if (windows[i].active) eligible = false;
  }
  portEXIT_CRITICAL(&stateMux);
  if (active && (!eligible || esp_timer_get_time() - start >= 60000000)) {
    diagnosticsProbeEnd(ProbeWindow::Normal);
    active = false;
  }
  if (eligible && !active) diagnosticsProbeBegin(ProbeWindow::Normal);
}

void diagnosticsProbeImuSample(float frequencyHz) {
  if (!isfinite(frequencyHz) || frequencyHz <= 0) return;
  portENTER_CRITICAL(&stateMux);
  WindowState& state = windows[static_cast<size_t>(ProbeWindow::Normal)];
  if (state.active) {
    if (state.skipFirstImu) {
      state.skipFirstImu = false;
    } else {
      if (state.imuSamples == 0 || frequencyHz < state.imuMinHz) state.imuMinHz = frequencyHz;
      state.imuSumHz += frequencyHz;
      ++state.imuSamples;
    }
  }
  portEXIT_CRITICAL(&stateMux);
}
