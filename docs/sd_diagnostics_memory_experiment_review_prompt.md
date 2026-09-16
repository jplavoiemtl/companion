# Prompt for Claude: review the revised memory experiment

Please review [Codex's counter-review and proposed experiment](sd_diagnostics_memory_counter_review_codex.md)
of [your memory review](sd_diagnostics_memory_review_claude.md).

Context: sd-diagnostics at 54baf96. Read CLAUDE.md first.
This is review only: do not change firmware, build, flash, commit, or push.
JP performs builds and flashing from VS Code.

Verify the counter-review's source claims independently. It links upstream IDF
revision 489d7a2b, named by the installed SDK. Also consult:
- [Bench results](sd_diagnostics_bench_results.md).
- [Accepted plan](sd_diagnostics_plan.md).
- [Stage 1 handoff](../src/diagnostics/STAGE1.md).
- [Writer implementation](../src/diagnostics/sd_diagnostics.cpp).
- [Clock code](../src/diagnostics/diagnostics_clock.cpp).
- The unchanged Stage 0 probes, local sketch.yaml and sdkconfig.

Reply to sections 1-6 with agree, disagree with source evidence, or refine.
Focus on:

1. Does the cross-run, per-region heap-minimum caveat invalidate the conclusion
   that mounted SD's resident internal cost is approximately zero? Is the
   9344-byte split an arithmetic hypothesis rather than an observed block?
2. Do you accept the findings about WithCaps keeping its control block internal,
   the legacy static cleanup flag, and suspension retaining additional task-local
   resources? Is explicit static creation plus once-per-boot parking still the
   simplest sensible experiment?
3. Verify sector-level DMA bounce buffering versus raw-host validation. Does the
   normal FatFS/SDMMC path support the experiment without assuming every
   unsupported pointer must fail cleanly?
4. Check cache-disabled/NVS reasoning and identify any concrete remaining blocker
   in the writer's call paths. Separate uncertainty from a demonstrated unsafe
   call. Future USB retrieval remains subject to its own review and gate.
5. Review DIAG_WRITER_STACK_PSRAM variants, failure handling, retained pointer
   and lifecycle status, and final watermark. Check the cleared-handle issue
   and accounting for a statically allocated TCB.
6. Review the staged tests: paired Latest with hooks off; Live only after memory
   passes; verified NVS/SD overlap; separate hooks-enabled rotation/pruning;
   no-card cleanup; log inspection; and both close paths.
7. Confirm that the 20480-byte gate stays unchanged, passing Latest alone does
   not accept Stage 1, and an unchanged result means the stack move was
   insufficient rather than proving the stack had no influence.

Do not implement yet. Give a short resolution list and a concrete go/no-go
recommendation. List changes needed before implementation; avoid unrelated
features or a broad redesign.

Write your response in a new file:
docs/sd_diagnostics_memory_experiment_review_claude.md

That response document is the only file to create or edit. Leave prior reviews,
plans and firmware unchanged.
