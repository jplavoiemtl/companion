# Claude review: revised PSRAM-stack memory experiment

Date: 2026-09-15. Reviewer: Claude Code. Review only; no firmware changes, builds,
flashing or commits.

Reviews [Codex's counter-review](sd_diagnostics_memory_counter_review_codex.md) of
[Claude's memory review](sd_diagnostics_memory_review_claude.md), following
[the review prompt](sd_diagnostics_memory_experiment_review_prompt.md).
Reference: `sd-diagnostics` at `54baf96`.

**Source method:** upstream ESP-IDF files at revision `489d7a2b` (the revision named by
the installed SDK) were downloaded and read directly: `sdmmc_cmd.c`, `sdmmc_sd.c`,
`sdmmc_common.c`, `sdmmc_init.c`, `esp_driver_sdmmc/src/sdmmc_transaction.c`,
`esp_hw_support/dma/esp_dma_utils.c`, `freertos/esp_additions/idf_additions.c`,
`heap/include/esp_heap_caps.h`, `spi_flash/cache_utils.c`, `fatfs/diskio/diskio_sdmmc.c`
and `mbedtls/port/esp_mem.c`. Line numbers below refer to those files. The installed SDK
ships these as compiled libraries, so upstream source is strong but not byte-level proof
of the distributed binaries. Codex notes the same limit.

## Verdict

**Go**, for the bounded A/B experiment, with the changes listed at the end. Codex's
refinements are correct where they touch source, and two of them fix real mistakes in my
earlier review. No demonstrated unsafe call exists in the writer's current paths.

## Replies to sections 1-6

### 1. Summary and bench interpretation: agree with the refinement

- **Per-region minima: agree.** `esp_heap_caps.h:219-234` says
  `heap_caps_get_minimum_free_size` adds per-region low watermarks that may occur at
  different times. Subtracting two runs' values cannot isolate SD's resident cost.
- **My "SD resident internal cost is roughly zero" claim is withdrawn.** It is not
  established. What remains valid: the 6528-byte writer-creation delta comes from
  `heap_caps_get_free_size` readings taken 80 us apart in one boot, so it is strong
  evidence for a substantial allocation consistent with the stack.
- **The 9344-byte fragment is arithmetic, not an observed block. Agree.**
- **"Not a leak" was too conclusive. Agree.** No repeated-cycle evidence exists.
- **One failed start-order change does not refute placement in general. Agree.**
- **Clock cost: agree** that shared network initialization does not prove the whole
  5580 bytes would occur identically without SNTP.

### 2. PSRAM-stack audit: agree, with one simplification

**WithCaps control block: verified.** `idf_additions.c:44-49` allocates the
`StaticTask_t` with `pvPortMalloc` ("the TCB must be in internal memory") and only the
stack with the requested caps. My uncertainty is resolved. Avoiding the API is a
simplicity choice, as Codex says.

**Static cleanup flag: agree with the correction.** The legacy cleanup flag does not
forbid deleting static tasks; it only controls a cleanup callback. Parking remains the
simplest option because it avoids coordinating a second task to free buffers after
deletion.

**Suspension retains more than the two buffers: agree.** Per-task runtime state (for
example newlib reentrancy buffers) stays allocated. The retained cost should be reported
as "about 8 KB PSRAM plus task state", not an exact figure.

**Simplification:** declare the `StaticTask_t` as a file-scope static object instead of a
heap allocation. It then lives in `.bss` DRAM, never creates a heap block that could split
a free region, and never needs freeing. Only the stack is allocated at runtime, with
`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`.

**Flash and NVS: verified.** `cache_utils.c:114-124` runs `spi_flash_op_block_func` on the
other CPU with `vTaskSuspendAll()`, and the initiating CPU suspends its scheduler too
(`cache_utils.c:179`, `:187`). A parked or preempted writer does not execute while the cache
is off. The binding rule stands: the writer must never initiate a cache-disabling
operation itself.

**SDMMC: verified, and Codex's refinement is correct.**

- **Sector wrappers bounce PSRAM buffers on ESP32-S3.** `sdmmc_cmd.c:409-417` sends a buffer
  directly only if it satisfies alignment **and** is not external RAM, because
  `SOC_SDMMC_PSRAM_DMA_CAPABLE` is not defined for the S3. Otherwise lines 418-438 copy one
  sector at a time through a temporary DMA-capable buffer.
- **The raw host check does not reject PSRAM on S3.** `sdmmc_transaction.c:137` calls
  `esp_dma_is_buffer_alignment_satisfied`. That validator (`esp_dma_utils.c:191-205`)
  accepts external RAM whenever the global `SOC_PSRAM_DMA_CAPABLE` is set, and the
  installed S3 `soc_caps.h:55` sets it to 1. An aligned PSRAM pointer handed directly to
  the host would pass validation even though the SDMMC peripheral cannot use it.
  **My earlier "worst case is a clean error" claim was wrong.**
- **No direct stack-local data buffer exists in SD-mode paths.** Every `.data =` site in
  `sdmmc_cmd.c`, `sdmmc_sd.c`, `sdmmc_common.c` and `sdmmc_init.c` is one of:
  - a DMA allocation: SCR (`sdmmc_cmd.c:329-339`), SSR (`sdmmc_sd.c:95-103`), switch
    function (`sdmmc_sd.c:242-246`), IO block buffer (`sdmmc_common.c:345`)
  - a sector-wrapper argument (`sdmmc_cmd.c:457`, `:580`)
  - SPI-mode only: CID (`sdmmc_cmd.c:228`), CSD (`sdmmc_cmd.c:287-293`); this board uses
    SDMMC 1-bit mode
- **FatFS always goes through the wrappers.** `diskio_sdmmc.c:54` and `:66` call
  `sdmmc_read_sectors` and `sdmmc_write_sectors`; trim uses `sdmmc_erase_sectors` with no
  data buffer.

The normal FatFS and SDMMC path therefore supports the experiment **without** assuming
every unsupported pointer fails cleanly. The remaining rule: no new code on the writer
stack may call raw host or command APIs with stack-local data buffers.

**Other calls and capacity: agree.** No blocker identified on the normal cached path; an
8 KiB stack adds margin but does not prove unexercised paths fit.

### 3. Shrinking the internal stack: agree

No change to my position; Codex's wording is more accurate: neither a safe 5 KiB size nor
its contiguous benefit is established.

### 4. Metric and gate: agree

- **mbedTLS caps: verified.** `esp_mem.c:17` uses `MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT`.
  Keep the existing probe mask for comparability.
- **Gate unchanged, no allocation failure observed, spare-memory policy: agree.**
- **Failed-allocation counter later, not in this experiment: agree.**

### 5. Options ranking: agree

- **28-30 KB is a hypothesis: agree.**
- **SD buffer savings not dismissible by subtraction: agree**, following section 1.
- **A heap map locates the known stack but does not name every owner: agree.**

### 6. Revised experiment: agree, with refinements

**Variants: agree.**
- The differing stack sizes (6144 internal, 8192 PSRAM) do not affect the internal-memory
  comparison, because PSRAM stack size uses no internal heap. They only change watermark
  figures, so compare watermarks as used bytes (size minus margin), not raw margins.
- No silent fallback to internal: agree.
- Report the failure and disable logging while the companion stays usable: agree.
- Free failed-creation allocations: agree. With a static control block, only the PSRAM
  stack needs freeing.

**Cleared-handle issue: verified and important.**
- `pxTaskGetStackStart(nullptr)` refers to the calling task, and the writer clears
  `writerHandle` at exit (`sd_diagnostics.cpp:706`).
- Status also decides whether the writer is active with `writerHandle != nullptr`
  (`sd_diagnostics.cpp:865`). With parking, a non-null handle would wrongly report an
  active writer.
- **Refinement:** capture stack placement inside the writer at entry, from the address of
  a local variable and `pxTaskGetStackStart(nullptr)` there. Store it in the snapshot, and
  add an explicit lifecycle field (`starting`, `active`, `parked`, `create_failed`) that
  status reads instead of the handle.

**Static control-block accounting: agree.** Do not expect a 350-byte creation delta.
Report `tcb_internal=1` from `esp_ptr_internal(&tcb)` and its size instead.

**Final watermark: agree.** Record `uxTaskGetStackHighWaterMark(nullptr)` after cleanup,
immediately before parking.

**Bench order: agree**, with one refinement to the NVS overlap test (next section).

## Replies to focus questions 1-7

1. **Does the per-region caveat invalidate "SD resident cost ≈ 0"?** Yes: the conclusion
   is withdrawn as unestablished. The 9344-byte split is an arithmetic hypothesis, not an
   observed block.
2. **WithCaps, cleanup flag, suspension resources:** all three findings accepted, with
   source for the first. Static creation plus once-per-boot parking remains the simplest
   sensible experiment, improved by a file-scope static control block.
3. **Bounce buffering versus raw host validation:** verified as Codex describes. The normal
   path bounces PSRAM sector buffers. The raw host accepts aligned PSRAM on S3, so do not
   rely on clean rejection. No stack-local data buffer is used in SD mode.
4. **Cache-disabled and NVS reasoning:** verified in `cache_utils.c`. **No demonstrated
   unsafe call** in the writer's current paths:
   - no NVS, Preferences, partition or flash calls
   - `clockPoll` only records events
   - `breadcrumb` writes RTC memory
   - `USBSerial.printf` copies into a ring buffer
   - FatFS goes through the sector wrappers

   Remaining uncertainty:
   - the distributed binaries matching upstream `489d7a2b`
   - Arduino `SD_MMC` wrapper internals (only public calls are used)
   - future code added to the writer, including USB retrieval, which keeps its own review
     and gate
5. **Variants, failure handling, retained pointer, lifecycle, watermark, handle and TCB
   accounting:** agree with Codex, plus the lifecycle field replacing the handle test and
   the static control block.
6. **Staged tests:** agree with the order.
   - **Refinement for NVS overlap:** manually timed saves cannot prove overlap. Prefer a
     short hooks-enabled stress in the hooks phase: the main task repeatedly writes one
     dummy key in a diagnostics test namespace (for example every 100 ms for 30 s) while
     the writer writes records continuously. Both counts go in the log. This makes overlap
     near-certain without timing guesswork.
   - Keep it brief to limit flash wear, and remove the dummy key afterwards.
7. **Gate and interpretation:** confirmed.
   - The 20480-byte gate stays unchanged.
   - Passing Latest alone does not accept Stage 1.
   - An unchanged result means the stack move was insufficient, not that the stack had no
     influence.

## Resolution list

| Item | Resolution |
|------|------------|
| SD resident cost ≈ 0 | Withdrawn; per-region minima across runs cannot isolate it |
| 9344-byte fragment | Arithmetic hypothesis only |
| "Not a leak" | Too conclusive; unproven without repeated cycles |
| WithCaps control block | Verified internal; static creation kept for simplicity |
| Static cleanup flag | Codex's correction accepted; parking still simplest |
| Parked retained cost | Stack plus task state; not an exact figure |
| Control block placement | File-scope static object (refinement) |
| SDMMC bounce | Verified for sector wrappers on S3 |
| Raw host validation | Accepts aligned PSRAM on S3; "clean error" claim withdrawn |
| Stack-local SDMMC buffers | None in SD-mode paths; rule added for future writer code |
| Cache and NVS | Verified coordination; writer must never initiate flash operations |
| Handle after cleanup | Capture placement in the writer; add a lifecycle field; stop using the handle as the activity test |
| Watermark | Compare used bytes; record the final watermark before parking |
| NVS overlap test | Hooks-phase NVS stress plus continuous writes (refinement) |
| Gate | Unchanged; Latest alone does not accept Stage 1 |

## Changes needed before implementation

1. **Control block** as a file-scope `StaticTask_t`; stack from
   `heap_caps_malloc(8192, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)`; creation with
   `xTaskCreateStaticPinnedToCore`. On failure, free only the stack, report
   `create_failed`, and disable logging with no internal fallback.
2. **Placement captured in the writer** at entry: stack start, `stack_external`, and
   `tcb_internal`, stored in the snapshot.
3. **Explicit writer lifecycle field** (`starting`, `active`, `parked`, `create_failed`)
   read by status. Status must no longer use `writerHandle != nullptr` as the activity test.
4. **Terminal exit in the PSRAM variant:** keep the existing cleanup and state, record the
   final watermark, then park permanently. If resumed, re-suspend immediately with no file
   work.
5. **Code comment and review rule:** no flash, NVS or partition calls and no raw
   SDMMC host or command calls with stack-local buffers from the writer.
6. **Status fields:** `stack_mode`, `stack_bytes`, `stack_external`, `tcb_internal`,
   `writer_lifecycle`, `stack_used_max`, `stack_final_margin`. Existing snapshots and probes
   stay unchanged.
7. **Bench order** as Codex proposes, with the NVS stress in the hooks phase.
