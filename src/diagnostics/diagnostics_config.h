#pragma once
// JP builds from VS Code. Leave fault hooks off for ordinary use and measurements.
#ifndef DIAG_ENABLED
#define DIAG_ENABLED 1
#endif
#ifndef DIAG_TEST_HOOKS
#define DIAG_TEST_HOOKS 1
#endif
// A/B bench experiment: 0 keeps the internal 6144-byte stack.
// 1 uses an 8192-byte PSRAM stack with a static internal control block.
#ifndef DIAG_WRITER_STACK_PSRAM
#define DIAG_WRITER_STACK_PSRAM 1
#endif
#if DIAG_WRITER_STACK_PSRAM != 0 && DIAG_WRITER_STACK_PSRAM != 1
#error "DIAG_WRITER_STACK_PSRAM must be 0 or 1"
#endif
#define DIAG_BUILD_TAG "stage1-basics"
