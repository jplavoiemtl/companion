#pragma once
// JP builds from VS Code. Leave fault hooks off for ordinary use and measurements.
#ifndef DIAG_ENABLED
#define DIAG_ENABLED 1
#endif
#ifndef DIAG_TEST_HOOKS
#define DIAG_TEST_HOOKS 0
#endif
// Temporary Stage 1B commands: fixture create/delete, pacing, queue/prune gates.
// log test slow on arms 100 ms per data line; off restores normal sending.
// Off in normal builds; set to 1 only for USB bench tests, with fault hooks off.
#ifndef DIAG_USB_TEST_FIXTURE
#define DIAG_USB_TEST_FIXTURE 0
#endif
#if DIAG_USB_TEST_FIXTURE && DIAG_TEST_HOOKS
#error "Generate the USB fixture with DIAG_TEST_HOOKS=0"
#endif
// Accepted default: 1 uses an 8192-byte PSRAM stack and static internal TCB.
// 0 retains the internal 6144-byte stack for controlled comparisons.
#ifndef DIAG_WRITER_STACK_PSRAM
#define DIAG_WRITER_STACK_PSRAM 1
#endif
#if DIAG_WRITER_STACK_PSRAM != 0 && DIAG_WRITER_STACK_PSRAM != 1
#error "DIAG_WRITER_STACK_PSRAM must be 0 or 1"
#endif
#define DIAG_BUILD_TAG "stage1b-usb"
