#pragma once
#include <stddef.h>
#include <stdint.h>
// Recovery validator wrapper: implementation delegates to the unchanged boot validator.
bool diagnosticsHeaderValid(const char* line, uint32_t& generation);
namespace diagtime {
enum class Quality : uint8_t { Pending, Unknown, Approx, Synced, Test, Malformed, Inconsistent, Io };
struct Time {
  uint16_t year=0, ms=0;
  int16_t offset=0;
  uint8_t month=0, day=0, hour=0, minute=0, second=0;
  Quality quality=Quality::Pending;
};
static_assert(sizeof(Time)<=16,"download timestamp stays compact");
struct Ends { Time opened, last; bool fragment=false; };
struct Record { Time time; uint64_t boot=0, sequence=0; bool valid=false; };
// Pure bounded parsing; header validation does not alter recovery policy.
Record parse(const char* line, size_t length, bool header);
bool validDate(const Time& value);
void format(const Time& value, char* out, size_t capacity);
void prefix(const Time& value, char* out, size_t capacity);
// Lifecycle worker: scratch exists before inventory is enabled, freed after quiescence.
bool allocate();
void dispose();
// Writer only. No current-file listing reads. All archive descriptors close in this call.
Ends archive(const char* path, uint64_t size);
const char* readStart(int reader, Time& out); // Exact position-zero restore or error.
void restoreCurrent(const char* header, size_t length, uint64_t size, uint32_t generation);
void written(const char* line, size_t length, bool first, uint64_t size, uint32_t generation);
Ends current(uint64_t& size, uint32_t& generation);
}
