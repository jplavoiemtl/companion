#pragma once
// JP builds from VS Code. Leave fault hooks off for ordinary use and measurements.
#ifndef DIAG_ENABLED
#define DIAG_ENABLED 1
#endif
#ifndef DIAG_TEST_HOOKS
#define DIAG_TEST_HOOKS 0
#endif
// Temporary Stage 1B bench command: log test file creates a synthetic 2 MiB archive.
// Set back to 0 for normal builds. This does not enable the fault/reset hooks.
#ifndef DIAG_USB_TEST_FIXTURE
#define DIAG_USB_TEST_FIXTURE 1
#endif
#if DIAG_USB_TEST_FIXTURE && DIAG_TEST_HOOKS
#error "Generate the USB fixture with DIAG_TEST_HOOKS=0"
#endif
// A/B bench experiment: 0 keeps the internal 6144-byte stack.
// 1 uses an 8192-byte PSRAM stack with a static internal control block.
#ifndef DIAG_WRITER_STACK_PSRAM
#define DIAG_WRITER_STACK_PSRAM 1
#endif
#if DIAG_WRITER_STACK_PSRAM != 0 && DIAG_WRITER_STACK_PSRAM != 1
#error "DIAG_WRITER_STACK_PSRAM must be 0 or 1"
#endif
#define DIAG_BUILD_TAG "stage1b-usb"
