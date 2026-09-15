#ifndef DIAGNOSTICS_PROBES_H
#define DIAGNOSTICS_PROBES_H
#include <stdint.h>

// Stage 0 only. Keep the sampler and windows identical in Stage 1.
enum class ProbeWindow : uint8_t { MqttConnect, ImageHttps, LiveTls, Live, Normal, Count };
void diagnosticsProbeInit();
// Application-task calls, never ISR calls. Different windows may overlap;
// the same window must not be nested.
void diagnosticsProbeBegin(ProbeWindow window);
void diagnosticsProbeEnd(ProbeWindow window);
// Call from loop before IMU work; pass false during media or future USB transfers.
void diagnosticsProbeNormalUpdate(bool eligible);
// Called only when the existing IMU sampling_frequency is freshly calculated.
void diagnosticsProbeImuSample(float frequencyHz);
#endif
