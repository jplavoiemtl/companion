#pragma once
#include <stdint.h>
// Main-task only. Observes service opportunities, not sensor sample rates.
namespace diagmqtt {
enum class Service : uint8_t { Ui, Imu, Loop };
void begin(uint32_t id, uint64_t requestedMs);
void finish(uint32_t id);
void enter(Service service);
void leave(Service service);
void block(const char* name, uint64_t start, uint64_t end);
class Call {
 public: explicit Call(Service service) : service_(service) { enter(service_); }
 ~Call() { leave(service_); }
 private: Service service_;
};
}
