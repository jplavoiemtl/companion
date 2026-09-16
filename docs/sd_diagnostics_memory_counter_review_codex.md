# Codex counter-review: SD diagnostics memory experiment

Date: 2026-09-15. Review of [Claude's response](sd_diagnostics_memory_review_claude.md)
to the [original prompt](sd_diagnostics_memory_review_prompt.md).
Reference: sd-diagnostics at 54baf96.

**Recommendation:** implement a controlled PSRAM-stack A/B experiment after the
refinements below are reviewed. This does not accept Stage 1 or approve PSRAM
placement as the normal configuration.

Checked: installed configuration, firmware, bench results, and upstream IDF
revision 489d7a2b named by the SDK. Reading upstream source does not independently
verify every object in the distributed binaries. This was review only.

## 1. Summary and bench interpretation - refine

The writer stack is a reasonable first target. The claim that mounted SD's
resident internal cost is roughly zero is not established.

- 36584 - 30756 = 5828 is correct. These are since-boot heap minima from separate
  runs, not simultaneous measurements of resident allocations.
- The heap API sums individual heap-region minima that can occur at different
  times. Subtracting these values cannot isolate SD's cost. This also refines
  earlier descriptions of our probe as an exact global low-water mark. Keep
  the existing API and probes unchanged for the A/B comparison.
- The 6528-byte writer-creation reduction is strong evidence of a substantial
  allocation, consistent with its stack and bookkeeping.
- 30708 - 6528 = 14836 + 9344 is correct arithmetic. The 9344-byte fragment is
  inferred by subtraction, not observed. Different runs, allocator overhead,
  and separate stack/control-block allocations prevent treating it as a
  demonstrated split.
- Fragmentation or allocation placement is plausible. "This is not a leak" is
  too conclusive without repeated-cycle evidence.
- Agree that the observed Latest dip required no additional logger writes.
- One startup-order experiment failed; it does not refute all placement options.
- Clock setup initializes shared networking, but that does not establish that
  all its measured cost would occur identically without SNTP.

Evidence: [bench results](sd_diagnostics_bench_results.md) and
[heap API semantics](https://github.com/espressif/esp-idf/blob/489d7a2b/components/heap/include/esp_heap_caps.h#L220-L234).

## 2. PSRAM-stack audit - agree with direction, refine safety claims

### Creation

Explicit PSRAM storage with xTaskCreateStaticPinnedToCore and an internal,
byte-accessible StaticTask_t is valid. Use MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
for the stack.

Claude's uncertainty about xTaskCreatePinnedToCoreWithCaps can now be resolved:
its implementation explicitly allocates the control block through the internal
FreeRTOS allocator. Avoiding that API remains a reasonable simplicity choice,
rather than a safety requirement.

Sources:
[WithCaps creation](https://github.com/espressif/esp-idf/blob/489d7a2b/components/freertos/esp_additions/idf_additions.c#L33-L78),
[allocator and pointer checks](https://github.com/espressif/esp-idf/blob/489d7a2b/components/freertos/heap_idf.c#L43-L107).

### Cleanup

Suspending after cleanup is acceptable for this once-per-boot writer, but the
reasoning needs correction. CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP controls
a legacy cleanup callback. Disabling it does not prohibit deleting static tasks.

Ordinary deletion leaves caller-owned stack/control storage allocated.
Suspension additionally retains task-local runtime resources, so retained cost
is not necessarily exactly "8 KB plus 350 bytes." Never free the executing stack.

Treat parked storage as an intentional once-per-boot reservation. Preserve
cleanup of the file, mounted card and formatter, producer shutdown, the logger
disabled/closed state and close acknowledgement. The parked task must not return
or resume filesystem work if unexpectedly resumed.

Sources:
[task deletion](https://github.com/espressif/esp-idf/blob/489d7a2b/components/freertos/FreeRTOS-Kernel/tasks.c#L4892-L4940),
[legacy cleanup hook](https://github.com/espressif/esp-idf/blob/489d7a2b/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c#L644-L670).

### Flash and NVS

Agree that the current writer has no direct flash/NVS operations. Ordinary flash
operations coordinate both cores through IPC and scheduler suspension before
disabling caches. Another task's normal NVS write is not automatically
incompatible with this stack.

The writer must not initiate cache-disabling operations itself. Keep that
restriction explicit and test concurrent screen-memory and calibration saves.

Source:
[cache coordination](https://github.com/espressif/esp-idf/blob/489d7a2b/components/spi_flash/cache_utils.c#L114-L218).

### SDMMC - substantially verified at the pinned upstream revision

- The sector-read/write wrappers explicitly route ESP32-S3 PSRAM buffers through
  temporary DMA-capable buffers, copying one sector at a time.
- The inspected SD initialization transfers allocate DMA-capable buffers.
- The bounce-buffer mechanism is confirmed in source. Whether it caused a
  particular measured memory dip remains unproven.

The raw-host claim needs refinement. The host rejects buffers failing its
validation, but that is not a blanket guarantee that every unsuitable PSRAM
pointer is rejected. The shared validator recognizes aligned PSRAM, while the
sector wrappers add the peripheral-specific exclusion.

Keep using the existing filesystem/sector path. Reject the claim that the worst
possible outcome is necessarily just a clean error.

Sources:
[sector wrappers](https://github.com/espressif/esp-idf/blob/489d7a2b/components/sdmmc/sdmmc_cmd.c#L397-L567),
[SD initialization](https://github.com/espressif/esp-idf/blob/489d7a2b/components/sdmmc/sdmmc_sd.c#L86-L125),
[host validation](https://github.com/espressif/esp-idf/blob/489d7a2b/components/esp_driver_sdmmc/src/sdmmc_transaction.c#L112-L168),
[buffer validator](https://github.com/espressif/esp-idf/blob/489d7a2b/components/esp_hw_support/dma/esp_dma_utils.c#L191-L246).

### Other calls and capacity

Formatting, time, filesystem and USB calls show no identified blocker on the
normal cached path. That supports a bench experiment, not unconditional safety
certification or advance approval of future USB retrieval code.

An 8 KiB stack adds margin; it does not prove untested rotation, pruning,
recovery and close paths fit.

## 3. Retaining and shrinking the internal stack - mostly agree

Shrinking directly to 4 KiB is unjustified: the reported watermark implies
approximately 4156 bytes already used. Watermarks do not certify unexercised paths.

Moving large locals could reduce requirements, but neither a safe 5 KiB size
nor its contiguous-memory benefit is established.

## 4. Metric and gate - agree, with interpretation refinements

No allocation failure was observed. The 20480-byte threshold remains unchanged.

The probe covers the entire HTTPS GET window, including allocation transients
and periods after TLS buffers exist. It is a spare-memory policy, not a direct
test that TLS lacked room to allocate its buffers.

mbedTLS requests MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT. Matching internal and DMA
readings supports the present measurements but does not prove those capability
masks are universally interchangeable. Preserve existing probes for comparability.

A failed-allocation counter could complement the gate later. It is not required
for this first experiment.

Source:
[mbedTLS allocator](https://github.com/espressif/esp-idf/blob/489d7a2b/components/mbedtls/port/esp_mem.c#L14-L32).

## 5. Options ranking - refine

Support PSRAM-stack A/B first, with heap mapping next if results remain unclear.

- Recovery to 28-30 KB is a hypothesis, not a guaranteed outcome.
- One unsuccessful startup-order change does not refute placement generally.
- SD-buffer savings cannot be dismissed using the 5828-byte subtraction.
- Heap mapping can identify the known stack's location. It does not automatically
  identify every allocation's owner.

## 6. Revised experiment proposed for review

### Build variants

| Switch | Stack and task policy |
|--------|-----------------------|
| DIAG_WRITER_STACK_PSRAM=0, default | Preserve the internal 6144-byte path and its cleanup. |
| DIAG_WRITER_STACK_PSRAM=1 | 8192-byte PSRAM stack, internal byte-accessible StaticTask_t, explicit static creation, cleanup then park on terminal exit. |

Do not silently fall back to internal memory in the PSRAM variant. Allocation
or creation failure must be reported, disabling logging while the companion
remains usable. Release failed-creation allocations not owned by a running task.

This compares two practical configurations, including differing stack capacities
and terminal lifecycle; it is not a pure placement-only experiment.

### Retained log status evidence

- Requested stack mode and configured bytes.
- Actual stack location, with a valid/unavailable indicator.
- Whether the control block is internal.
- Writer lifecycle: active, parked, failed-to-start or deleted as applicable,
  separately from logger ready/disabled/closed state.
- Stack watermark after final cleanup work as well as during operation.
- Existing startup snapshots and logger/probe measurements.

Capture pointer properties while the writer is valid, preferably in the writer
itself. Do not query its handle after cleanup clears it: a null handle can refer
to the calling task instead. Avoid racing task deletion from status.

Do not require a 350-byte creation delta. A statically declared control block
consumes memory before that measurement window. Report size and placement
rather than inferring them from the delta.

### Bench order

JP performs one test at a time and builds and flashes from VS Code.

1. Pair the internal and PSRAM builds in the same sitting. Keep the card,
   hotspot, image and conditions the same. Hooks are off in both builds.
   Let setup finish, attach the web console, request log status, press Latest
   once and request log status again.
2. If memory passes, continue to paired Live performance testing with the
   existing probe windows and same-session baseline rule.
3. Test actual overlap between NVS saves and SD activity. Confirm overlap from
   evidence instead of assuming manually timed saves coincide with health writes.
4. Run rotation/pruning separately with DIAG_TEST_HOOKS enabled and small limits.
   Back up the card before destructive fault testing and preserve unrelated files.
   Restore normal limits and disable hooks afterward.
5. Include no-card cleanup, log-content inspection, and both deep-sleep and
   shutdown close paths. Observe lifecycle and final stack margin.

Claude's original instructions said hooks off but then requested small-limit
hooks. These require separate builds or test phases.

### Pass and stop criteria

For the first memory comparison:

- Logger ready, verified PSRAM stack and internal control block.
- image_https largest-block minimum at least 20480 bytes with comparable,
  adequate sampling coverage.
- Internal comparison approximately reproduces the previous result. If it does
  not, investigate the changed baseline instead of claiming an A/B gain.
- No new allocation/TLS errors, logger errors, corruption, resets or queue drops.

Later tests must meet the existing Live performance gate, preserve log data,
complete cleanup, and show sufficient stack margin on exercised paths.
A reported margin alone is not acceptance; review the observed minimum against
the full path coverage with JP.

Stop on errors, resets, corruption, new allocation failures or inadequate stack
margin. A result below 20480 bytes fails the gate. An unchanged result means the
stack move is not a sufficient fix, not necessarily that the stack had no influence.
Return to the internal variant and investigate.

## Decision

Agree with implementing this bounded experiment after review of these adjustments.
Keep the gate unchanged. Require JP's acceptance before making PSRAM placement
normal operation or advancing past Stage 1.
