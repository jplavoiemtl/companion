# Claude review: SD diagnostics internal-memory margin

Date: 2026-09-15. Reviewer: Claude Code. Analysis only; no firmware changes, builds,
flashing or commits.

Answers [sd_diagnostics_memory_review_prompt.md](sd_diagnostics_memory_review_prompt.md)
for branch `sd-diagnostics` at checkpoint `54baf96`. Evidence comes from
[sd_diagnostics_bench_results.md](sd_diagnostics_bench_results.md), the code under
`src/diagnostics/`, and the installed core 3.1.3 and SDK
(`esp32-arduino-libs/idf-release_v5.3-489d7a2b-v1`).

**Source limits:** the SDMMC driver, FATFS and the FreeRTOS kernel are compiled libraries
in the installed SDK (`libesp_driver_sdmmc.a`, `libsdmmc.a`, `libfatfs.a`). Only their
headers and `sdkconfig` are local. Statements about their internals are marked as
inference.

## Summary

The strongest evidence points at the **writer's 6144-byte internal task stack**. By total
size it accounts for nearly all of the logger's internal memory cost, and its placement
plausibly explains the lost contiguous block. Recommended next step: a single A/B build
switch that moves the writer stack to PSRAM, keeps the task control block internal, and
compares both builds in the same sitting.

## 1. What the results establish

### Confirmed from bench data

- **The logger's total internal cost is about the size of the writer stack.**
  - Card versus no card, lowest internal memory during HTTPS: 30756 versus 36584 bytes,
    a difference of **5828 bytes**. The no-card run frees the writer stack and all SD
    resources.
  - Writer creation measured **6528 bytes** (6144-byte stack plus task bookkeeping).
  - The mounted SD stack's resident internal cost is therefore roughly zero, within
    noise. The remaining logger memory (queue, formatter) is PSRAM in both runs.
- **Contiguous space dropped far more than total space.** The largest block fell by 13824
  bytes versus no card and 15872 bytes versus probe-only, while total free fell by only
  5.8-7.9 KB. This is fragmentation, not a leak.
- **The dip needs no SD write.** Write count and file size were unchanged during the
  failing Latest runs.
- **Startup order changed nothing.** Both orders produced exactly 14836 bytes. That points
  to deterministic placement rather than timing.
- **The clock interval (5580 bytes) is not a removable saving.** `configTzTime` calls
  `esp_netif_init` (core `esp32-hal-time.c`), which Wi-Fi initialization performs anyway.
  The allocation only happens earlier.

### Hypothesis consistent with the numbers

The 6.5 KB stack sits inside the free region that TLS later draws from, splitting it.
Probe-only largest block 30708 − 6528 = 24180 bytes remaining, in two pieces: the
observed 14836 and about 9344. The arithmetic fits exactly, but it is not proof of
placement.

### Unproven

- Actual block addresses and ownership.
- Whether several small allocations contribute instead of, or in addition to, the stack.
- The transient SD DMA buffer. The unchanged write count already argues against it.

### Is the stack the best target?

Yes. No other resident internal allocation of meaningful size is identified. The
240-byte snapshot array and SD_MMC's small internal structures are minor.

## 2. PSRAM stack safety audit

### Creation API

- Use **`xTaskCreateStaticPinnedToCore`** with the stack from
  `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)` and the `StaticTask_t` control block from
  **internal** memory.
- Avoid `xTaskCreatePinnedToCoreWithCaps`. It exists in the 5.3 header
  (`freertos/idf_additions.h:284`), but its implementation is compiled, so whether it
  also places the control block in PSRAM cannot be verified locally. Its header also
  recommends deleting such tasks from another task rather than self-deletion
  (`idf_additions.h:342`).
- `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` permits external stacks; it does not by
  itself prove this task is safe.

### Deletion and cleanup

- `CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP` is not set, so the kernel never frees a
  static task's buffers.
- The writer currently ends its terminal path with `vTaskDelete(nullptr)`
  (`sd_diagnostics.cpp:709`). With a static PSRAM stack, change this to: clean up SD,
  then suspend forever. The retained cost is about 8 KB of PSRAM for the stack and about
  350 internal bytes for the control block.
- Never free the stack from inside the task.

### Cache-disabled flash and NVS operations

This is the binding restriction: a task with an external stack must not itself disable
the flash cache.

- **Verified in code:** the writer makes no NVS, Preferences, `esp_partition` or
  `spi_flash` calls. `sd_diagnostics.cpp:740` keeps NVS in `diagnosticsInitEarly`, and the
  controlled-panic test hook runs from the main task (`sd_diagnostics.cpp:825`).
- **Inference (compiled `cache_utils`):** when another task writes NVS, IDF suspends
  scheduling on both cores while the cache is off, so the writer simply does not run in
  that window.
- **Enforce the rule** with a code comment and review check, and test concurrent
  screen-memory and calibration saves while the writer is busy.

### SDMMC DMA and stack-local buffers

**Unverified locally.** Understanding of the v5.3 sources:

- `sdmmc_read_sectors` and `sdmmc_write_sectors` copy non-DMA buffers through a temporary
  DMA-capable buffer.
- Card-initialization data transfers use heap DMA buffers.
- The host transaction code rejects a non-DMA data pointer with an error instead of
  transferring it.

If this holds, the worst case with a PSRAM stack is a visible mount or I/O error, not
silent corruption. **Verify against the v5.3 `esp_driver_sdmmc` and `sdmmc` sources
before relying on it.**

### Other calls on the writer stack

- **FATFS long-filename workspace** (`CONFIG_FATFS_LFN_STACK=y`) is ordinary stack memory,
  not a DMA buffer; fine in PSRAM.
- **ROM and newlib formatting and time calls** (`snprintf`, `localtime_r`) are fine while
  the cache is enabled.
- **`USBSerial.printf`** copies into the HWCDC ring buffer; no flash access.
- **Rotation, pruning, recovery and close** use the same file-system calls as normal
  writes.
- **Future USB retrieval** can run on this stack, provided it never triggers flash
  writes. The plan already keeps the NVS boot counter in setup.

### Additional benefit

With a PSRAM stack, the size can grow to **8192 bytes** at no internal cost. That removes
the thin margin on unmeasured paths: rotation, pruning, close and recovery.

## 3. Keeping the internal stack and reducing use

- The measured peak is 4156 bytes (6144 − 1988). Rotation, recovery and close peaks are
  unmeasured.
- Large locals could move to PSRAM buffers, for example `writeHealth` `fields[704]`
  (`sd_diagnostics.cpp:595`) and `fields[384]` (`sd_diagnostics.cpp:367`). That might allow
  about 5 KB, saving about 1 KB.
- A ~5 KB block still splits a free region. Unless it happens to land below the ~9.3 KB
  edge, which is placement luck, the largest block barely improves.
- Agree with Codex: dropping to 4096 bytes is not justified.

## 4. Metric and gate

- **The mask is adequate.** mbedTLS allocates internal byte-accessible memory
  (`CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y`). The probe uses `MALLOC_CAP_INTERNAL`, and the
  reported DMA largest block equals the internal largest block in every run (31732,
  14836), so the regions agree. Adding `MALLOC_CAP_8BIT` would make the match exact.
- **What it measures:** the lowest sampled largest free internal block during `GET()`,
  including handshake transients. In this CAR build the persistent MQTT TLS session is
  also held (port 9735, `secrets.h:13`). Sampling at 10 ms can miss shorter dips, so the
  true minimum is at most 14836 bytes.
- **Gate failure is not an allocation failure.** No TLS or allocation failure was observed
  and every image succeeded.
- **The 20480 gate is a headroom policy measured after the TLS buffers are allocated.**
  Its stated rationale (mbedTLS needs about 16 KB contiguous) describes the space needed
  before allocation, so as measured it is stricter than its rationale. The headroom still
  matters for long uptime fragmentation, larger certificate chains and Live after hours.
- **Recommendation: do not lower the gate now.** If the PSRAM-stack build returns about
  25-28 KB, the gate passes as defined and the question disappears.
- **For a later decision by JP:** a direct hard metric is a failed-allocation counter,
  registered with `heap_caps_register_failed_alloc_callback` (`heap/esp_heap_caps.h:66`)
  and reported in `log status` beside the headroom figure.

## 5. Options ranked

| Rank | Option | Evidence | Expected internal benefit | Risk |
|------|--------|----------|---------------------------|------|
| 1 | Writer stack in PSRAM, control block internal, suspend instead of delete | Strong: total cost ≈ stack; split arithmetic fits | About 6.2 KB free, and likely most of the contiguous loss, back toward about 28-30 KB | Low to moderate: no flash calls in the writer; SDMMC DMA handling needs a source check |
| 2 | Heap-map diagnostic first (`heap_caps_walk`, `esp_heap_caps.h:491`, plus `pxTaskGetStackStart`) | Would prove ownership | None by itself | Very low; costs a bench cycle |
| 3 | Shrink the internal stack by moving large locals to PSRAM | Weak for contiguity | About 1 KB free, little contiguous gain | Low; unmeasured peaks |
| 4 | Change allocation timing or placement | Refuted once (identical 14836) | Unpredictable | Fragile |
| 5 | SD_MMC `maxOpenFiles`, FATFS buffers, clock | Resident SD internal cost ≈ 0; clock work happens anyway | Negligible | Low |

## 6. Recommended next action

**One A/B experiment:** a build switch `DIAG_WRITER_STACK_PSRAM`.

| Value | Writer stack |
|-------|--------------|
| 0 | Current internal 6144-byte stack |
| 1 | 8192-byte PSRAM stack, internal `StaticTask_t`, suspend instead of delete on terminal exit |

Everything else stays identical, including the Stage 0 probes. JP flashes both builds in
one sitting.

### Evidence retrievable with `log status`

- New fields: `stack_caps=internal|spiram`, `stack_external=0|1` from
  `esp_ptr_external_ram(pxTaskGetStackStart(writerHandle))`, and the configured stack size.
- The existing retained startup snapshots. With the PSRAM build, the `writer_entry`
  interval should fall from about 6528 to about 350 bytes.

### Bench test

Card installed, fault hooks off, same image for both builds:

1. Boot each build, attach the page, run `log status`, request Latest once, run
   `log status` again.
2. **PSRAM build only:** trigger screen-memory and calibration NVS saves while the writer
   is writing health records. Then use the small-limit test hook to force a rotation and a
   prune, followed by a deep sleep close and a shutdown close.

### Pass (PSRAM build)

- Logger ready and `stack_external=1`.
- `image_https` largest-block minimum **at least 20480 bytes** with similar sample
  coverage, while the internal build again shows about 14836 bytes.
- Every step 2 action completes with no panic, no watchdog reset, no DMA or
  invalid-argument SD errors, no drops, and a reported stack margin.

### Stop and reassess (no gate change)

- **PSRAM build still near 14836 bytes:** the stack hypothesis is rejected. Run the
  heap-map diagnostic (option 2) next.
- **Any SD error or panic on the PSRAM stack:** return to the internal stack and verify the
  SDMMC sources before retrying.
