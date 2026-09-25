#pragma once
#include "net_module.h"
#include <stdint.h>
#include "../diagnostics/diagnostics_internal.h"
namespace mqttowner {
constexpr uint32_t DNS_MS=15000, ATTEMPT_MS=35000, STUCK_MS=40000, SOCKET_MS=5000;
enum class Phase : uint8_t { Idle, Dns, Lease, Tcp, Tls, Mqtt, Subscribe, Online, Cleanup, Fault, Stopped };
struct View {
  uint32_t epoch=1, attemptEpoch=0, id=0;
  uint64_t started=0, phaseAt=0;
  Phase phase=Phase::Idle;
  bool link=false, stop=false, busy=false, connected=false, lease=false, requestedLease=false;
  bool allocationFailed=false;
  bool stackExternal=false, tcbInternal=false;
  int state=-1;
  uint32_t stackMin=0, internalMin=UINT32_MAX, largestMin=UINT32_MAX, dmaMin=UINT32_MAX, dmaLargestMin=UINT32_MAX;
  uint32_t rxDrops=0, txDrops=0, completionDrops=0, leaseTimeouts=0;
  uint32_t cancelled=0, stuck=0, txOversize=0, txAccepted=0, txRejected=0, rxPacketDrops=0;
};
struct Result {
  uint32_t id=0, epoch=0; uint64_t started=0, ended=0;
  uint32_t dnsMs=0, tcpMs=0, tlsMs=0, mqttMs=0;
  uint8_t valid=0, subscriptions=0;
  bool ok=false, counted=false, test=false;
  int state=-1, error=0;
  bool errorFresh=false;
  diag::Stamp when{};
  const char* dnsResult="not_run";
  const char* reason="none";
  Phase failedPhase=Phase::Idle;
};
struct Rx { uint32_t epoch; uint16_t length; uint8_t topic; uint64_t at; char payload[513]; };
struct Tx { uint32_t epoch; uint16_t length; uint8_t topic; uint64_t at; char payload[768]; char category[16], trigger[16]; };
struct Sent { uint32_t epoch; diag::Stamp when; char category[16], trigger[16]; bool accepted; };
bool init(const NetConfig& cfg);
void linkEvent(bool up); // driver: fixed metadata only, no allocations/network/UI
void invalidate(bool stopping=false);
View view();
bool request(uint32_t id,int connection,bool test);
void arbitrate(bool available); // main only; includes late-grant protection
void sample(); // main samples memory only, no worker network state queries
bool takeResult(Result& result);
bool takeLoss(int& state);
bool takeRx(Rx& message);
bool publish(uint8_t topic,const char* payload,const char* category,const char* trigger);
bool takeSent(Sent& completion);
void acknowledgeReady(uint32_t epoch);
const char* phaseName(Phase phase);
}
