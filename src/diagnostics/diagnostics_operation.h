#pragma once
#include "sd_diagnostics.h"
// Main-task observers only; all labels are code literals. No filesystem access.
namespace diagop {
uint32_t nextId();
void block(const char* name, uint64_t id, uint64_t start, uint64_t end);
void observe(uint8_t screen, bool usb, bool moving);
void health(DiagnosticsHealth& h);
class Loop {
 public: Loop(); ~Loop();
};
class Block {
 public: explicit Block(const char* name, uint64_t id = 0); ~Block();
 private: const char* name_; uint64_t id_, start_;
};
}
