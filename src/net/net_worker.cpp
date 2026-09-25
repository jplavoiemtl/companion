#include "net_worker.h"
#include "mqtt_client/OwnedPubSubClient.h"
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_memory_utils.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>
#include <lwip/dns.h>
#include <lwip/tcpip.h>
#include <string.h>
#if defined(__has_include) && __has_include("secrets_private.h")
#include "secrets_private.h"
#else
#include "secrets.h"
#endif

namespace mqttowner {
namespace {
constexpr size_t STACK_BYTES=12288, WIRE_BYTES=512;
constexpr uint32_t INTERNAL_GATE=20480;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
StaticTask_t workerTcb;
StackType_t* workerStack=nullptr;
TaskHandle_t workerHandle=nullptr;
NetConfig config{}; // Immutable after init; all configuration strings have static lifetime.
View status;
struct Command { uint32_t id,epoch; uint64_t started; int connection; bool test; };
Command pending{};
bool commandReady=false,resultReady=false,readyAck=false,allocationFailed=false,lossReady=false;
int lossState=-1;
Result finalResult{};
struct Queues { Rx rx[4]; Tx tx[4]; Sent sent[4]; };
Queues* queues=nullptr;
uint8_t rxHead=0,rxCount=0,txHead=0,txCount=0,sentHead=0,sentCount=0;
static_assert(sizeof(Queues)<=8192,"MQTT queue PSRAM budget");

// Retain callback arguments until acknowledgement, NEVER until an elapsed timeout.
struct DnsSlot {
  bool used=false,done=false,abandoned=false;
  uint32_t id=0,epoch=0;
  err_t error=ERR_OK;
  ip_addr_t address{};
  char hostname[254]{};
};
DnsSlot dnsSlots[2];
static_assert(sizeof(dnsSlots)+sizeof(View)+sizeof(Result)+sizeof(Command)+sizeof(NetConfig)+128<=2048,
              "MQTT internal control metadata budget (static TCB excluded)");
uint64_t nowMs() { return esp_timer_get_time()/1000; }
bool current(uint32_t epoch) {
  portENTER_CRITICAL(&mux);
  bool ok=status.epoch==epoch && status.link && !status.stop && status.phase!=Phase::Fault;
  portEXIT_CRITICAL(&mux); return ok;
}
void clearMessagesLocked() { status.rxDrops+=rxCount; rxCount=txCount=0; }
void invalidateLocked(bool stopping) {
  if(status.phase==Phase::Online && !status.busy) { status.busy=true; status.started=nowMs(); }
  ++status.epoch; status.stop|=stopping; status.connected=false; clearMessagesLocked();
  // Resource/lease ownership survives invalidation until the owner cleans up.
}
void completeDns(DnsSlot* slot,const ip_addr_t* address,err_t error) {
  portENTER_CRITICAL(&mux);
  slot->error=error; if(address) slot->address=*address;
  slot->done=true; if(slot->abandoned) slot->used=false;
  portEXIT_CRITICAL(&mux);
}
void dnsFound(const char*,const ip_addr_t* address,void* argument) {
  completeDns(static_cast<DnsSlot*>(argument),address,address ? ERR_OK : ERR_VAL);
}
void submitDns(void* argument) {
  auto* slot=static_cast<DnsSlot*>(argument);
  ip_addr_t address{};
  const err_t error=dns_gethostbyname_addrtype(slot->hostname,&address,dnsFound,slot,LWIP_DNS_ADDRTYPE_IPV4);
  if(error!=ERR_INPROGRESS) completeDns(slot,error==ERR_OK ? &address : nullptr,error);
}
const char* resolve(const Command& cmd,const char* hostname,IPAddress& address,Result& result) {
  if(strchr(hostname,':')) return "ipv6_unsupported";
  if(address.fromString(hostname)) return "skipped_literal";
  if(strlen(hostname)>253) return "dns_name_size";
  DnsSlot* slot=nullptr;
  portENTER_CRITICAL(&mux);
  for(auto& candidate:dnsSlots) if(!candidate.used) { slot=&candidate; break; }
  if(slot) {
    *slot=DnsSlot{}; slot->used=true; slot->id=cmd.id; slot->epoch=cmd.epoch;
    memcpy(slot->hostname,hostname,strlen(hostname)+1);
  }
  portEXIT_CRITICAL(&mux);
  if(!slot) { result.counted=false; return "dns_slots_busy"; }
  const err_t submitted=tcpip_try_callback(submitDns,slot);
  if(submitted!=ERR_OK) {
    portENTER_CRITICAL(&mux); slot->used=false; portEXIT_CRITICAL(&mux);
    result.error=submitted; result.counted=false; return "dns_submit_busy";
  }
  for(;;) {
    vTaskDelay(1);
    const bool expired=nowMs()-cmd.started>=DNS_MS || !current(cmd.epoch);
    portENTER_CRITICAL(&mux);
    const bool done=slot->done;
    const err_t error=slot->error;
    const ip_addr_t found=slot->address;
    if(done) slot->used=false;
    else if(expired) slot->abandoned=true;
    portEXIT_CRITICAL(&mux);
    if(expired) return current(cmd.epoch) ? "dns_timeout" : "cancelled";
    if(!done) continue;
    result.error=error;
    if(error==ERR_MEM) { result.counted=false; return "dns_resolver_busy"; }
    if(error!=ERR_OK || !IP_IS_V4(&found)) return "dns_failed";
    address=IPAddress(ip4_addr_get_u32(ip_2_ip4(&found))); return "ok";
  }
}

// Worker-only operation state: absolute deadlines, including continuous byte arrivals.
uint32_t operationEpoch=0;
uint64_t operationDeadline=0,attemptDeadline=0;
bool operationAllowed() {
  return current(operationEpoch) && nowMs()<operationDeadline &&
         (!attemptDeadline || nowMs()<attemptDeadline);
}
class ConnectedTransport final : public Client {
 public:
  Client* transport=nullptr;
  bool ready=false;
  int connect(IPAddress,uint16_t) override { return 0; }
  int connect(const char*,uint16_t) override { return 0; }
  size_t write(uint8_t value) override { return write(&value,1); }
  size_t write(const uint8_t* data,size_t size) override {
    return ready && operationAllowed() ? transport->write(data,size) : 0;
  }
  int available() override { return ready && operationAllowed() ? transport->available() : 0; }
  int read() override { return ready && operationAllowed() ? transport->read() : -1; }
  int read(uint8_t* data,size_t size) override { return ready && operationAllowed() ? transport->read(data,size) : -1; }
  int peek() override { return ready && operationAllowed() ? transport->peek() : -1; }
  void flush() override { /* No network drain/wait. */ }
  void stop() override { ready=false; if(transport) transport->stop(); }
  uint8_t connected() override { return ready && operationAllowed() && transport->connected(); }
  operator bool() override { return connected(); }
};
void receive(char* topic,byte* payload,unsigned int size) {
  int category=-1;
  const char* names[]={config.topics.image,config.topics.power,config.topics.energy};
  for(int i=0;i<3;++i) if(names[i] && !strcmp(names[i],topic)) category=i;
  const uint64_t at=nowMs();
  portENTER_CRITICAL(&mux);
  if(category<0 || size>512 || rxCount==4) ++status.rxDrops;
  else if(status.epoch==operationEpoch && status.connected && !status.stop) {
    Rx& item=queues->rx[(rxHead+rxCount)%4];
    item.epoch=operationEpoch; item.at=at; item.topic=category; item.length=size;
    memcpy(item.payload,payload,size); item.payload[size]=0; ++rxCount;
  } else ++status.rxDrops;
  portEXIT_CRITICAL(&mux);
}
void workerSample(bool phaseBoundary=false) {
  static uint64_t stackAt=0, heapAt=0;
  const uint64_t now=nowMs();
  // Stack scans walk PSRAM. Force a measurement at phase boundaries, otherwise
  // sample at most once per second (including idle/ONLINE turns).
  if(phaseBoundary || now-stackAt>=1000) {
    const uint32_t margin=uxTaskGetStackHighWaterMark(nullptr);
    const bool external=esp_ptr_external_ram(&margin);
    portENTER_CRITICAL(&mux);
    status.stackExternal=external;
    status.stackMin=status.stackMin ? min(status.stackMin,margin) : margin;
    portEXIT_CRITICAL(&mux);
    stackAt=now;
  }
  if(view().busy && (phaseBoundary || now-heapAt>=20)) { sample(); heapAt=now; }
}
void phase(Phase value) {
  const uint64_t at=nowMs();
  portENTER_CRITICAL(&mux);
  if(status.phase!=Phase::Fault) { status.phase=value; status.phaseAt=at; }
  portEXIT_CRITICAL(&mux); workerSample(true);
}
bool admitPhase(const Command& cmd,Phase value,uint32_t allowance,Result& result) {
  if(!current(cmd.epoch)) { result.reason="cancelled"; return false; }
  if(nowMs()+allowance>cmd.started+ATTEMPT_MS) { result.reason="attempt_budget"; return false; }
  result.failedPhase=value; phase(value); operationDeadline=nowMs()+allowance; return true;
}
bool acquireLease(const Command& cmd,Result& result) {
  phase(Phase::Lease);
  const uint64_t until=nowMs()+100;
  portENTER_CRITICAL(&mux);
  if(status.epoch==cmd.epoch && !status.stop && status.phase!=Phase::Fault) status.requestedLease=true;
  portEXIT_CRITICAL(&mux);
  for(;;) {
    vTaskDelay(1);
    const bool valid=current(cmd.epoch);
    portENTER_CRITICAL(&mux);
    bool granted=status.lease;
    bool pending=status.requestedLease;
    bool expired=nowMs()>=until;
    if(!valid || expired) {
      if(expired && pending) ++status.leaseTimeouts;
      status.requestedLease=false; status.lease=false; granted=false;
    }
    portEXIT_CRITICAL(&mux);
    if(!valid || expired || !pending) {
      if(granted && valid && !expired) return true;
      result.counted=false; result.reason=valid ? "lease_deferred" : "cancelled"; return false;
    }
  }
}
void worker(void*) {
  // Constructors, configuration and all client calls stay on the owner task.
  WiFiClient plain;
  WiFiClientSecure secure;
  ConnectedTransport facade;
  OwnedPubSubClient client(facade);
  client.setOperationGuard(operationAllowed);
  const bool bufferOk=client.setBufferSize(WIRE_BYTES);
  client.setSocketTimeout(5); client.setCallback(receive);
  plain.setConnectionTimeout(5000);
  secure.setConnectionTimeout(5000); secure.setHandshakeTimeout(5);
  bool online=false;
  for(;;) {
    vTaskDelay(pdMS_TO_TICKS(10)); // Idle/ONLINE cadence; active waits use one tick.
    workerSample();
    portENTER_CRITICAL(&mux); status.rxPacketDrops=client.rejectedPackets(); portEXIT_CRITICAL(&mux);
    Command cmd{};
    portENTER_CRITICAL(&mux);
    const bool waiting=resultReady;
    bool start=commandReady && !waiting;
    if(start) { cmd=pending; commandReady=false; }
    const bool acknowledged=readyAck; readyAck=false;
    portEXIT_CRITICAL(&mux);
    if(acknowledged) online=true;
    if(start) {
      Result result{}; result.id=cmd.id; result.epoch=cmd.epoch; result.started=cmd.started;
      result.test=cmd.test; result.counted=!cmd.test;
      operationEpoch=cmd.epoch; attemptDeadline=cmd.started+ATTEMPT_MS; operationDeadline=attemptDeadline;
      client.disconnect(); facade.stop(); plain.stop(); secure.stop();
      const char* host=cmd.test ? "192.0.2.1" : cmd.connection==1 ? config.server1 : config.server2;
      const uint16_t port=cmd.connection==1 ? config.serverPort1 : config.serverPort2;
      const bool tls=port==9735 || port==8883;
      client.setServer(host,port); // Facade rejects any hidden hostname reconnect.
      IPAddress address;
      if(!bufferOk) { result.reason="client_alloc"; result.counted=false; }
      else if(!current(cmd.epoch)) result.reason="cancelled";
      else {
        phase(Phase::Dns); result.failedPhase=Phase::Dns;
        const uint64_t dnsAt=nowMs();
        result.dnsResult=resolve(cmd,host,address,result);
        result.dnsMs=nowMs()-dnsAt; result.valid|=1; result.reason=result.dnsResult;
        if((!strcmp(result.dnsResult,"ok") || !strcmp(result.dnsResult,"skipped_literal")) && acquireLease(cmd,result)) {
          if(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)<INTERNAL_GATE) {
            result.reason="memory_refused"; result.counted=false;
          } else if(admitPhase(cmd,Phase::Tcp,5000,result)) {
            const uint64_t at=nowMs();
            facade.transport=tls ? static_cast<Client*>(&secure) : static_cast<Client*>(&plain);
            if(tls) secure.setPlainStart();
            const int connected=tls ? secure.connect(address,port,host,config.caCert,nullptr,nullptr) : plain.connect(address,port);
            result.tcpMs=nowMs()-at; result.valid|=2;
            if(!connected) {
              result.reason="tcp_setup_failed";
              if(tls) { char ignored[1]; result.error=secure.lastError(ignored,sizeof(ignored)); result.errorFresh=true; }
            } else if(!current(cmd.epoch)) result.reason="cancelled";
            else {
              bool tlsOk=!tls;
              if(tls && admitPhase(cmd,Phase::Tls,5000,result)) {
                const uint64_t tlsAt=nowMs();
                tlsOk=secure.startTLS(); result.tlsMs=nowMs()-tlsAt; result.valid|=4;
                if(!tlsOk) result.reason="tls_failed"; // startTLS does not refresh lastError.
              }
              if(tlsOk && admitPhase(cmd,Phase::Mqtt,5000,result)) {
                facade.ready=true; // No plaintext MQTT before verified TLS success.
                const uint64_t mqttAt=nowMs();
                const bool ok=cmd.test ? client.connect("companion-bench-test") : client.connect(CLIENT_ID,USERNAME,KEY);
                result.mqttMs=nowMs()-mqttAt; result.valid|=8; result.state=client.state();
                result.reason=ok ? "ok" : "mqtt_failed";
                if(ok && admitPhase(cmd,Phase::Subscribe,2000,result)) {
                  const char* topics[]={config.topics.image,config.topics.power,config.topics.energy};
                  bool all=true;
                  for(unsigned i=0;i<3;++i) {
                    vTaskDelay(1);
                    if(!operationAllowed()) { all=false; break; }
                    if(!topics[i] || client.subscribe(topics[i],1)) result.subscriptions|=1<<i;
                    else all=false;
                  }
                  result.ok=all && operationAllowed(); result.reason=result.ok ? "ok" : "subscribe_failed";
                }
              }
            }
          }
        }
      }
      // Publish READY atomically with epoch validation. Otherwise close before the
      // result/cleanup acknowledgement is visible, including a just-cancelled success.
      if(result.ok) {
        workerSample(true); // Subscription completion / transition to READY.
        result.ended=nowMs(); result.when=diag::stamp();
        portENTER_CRITICAL(&mux);
        const bool valid=status.epoch==cmd.epoch && status.link && !status.stop && status.phase!=Phase::Fault;
        if(valid) {
          status.requestedLease=false; status.busy=false; status.connected=false;
          status.state=result.state; status.phase=Phase::Online;
          finalResult=result; resultReady=true; // retain lease until main adopts READY
        }
        portEXIT_CRITICAL(&mux);
        if(valid) { attemptDeadline=0; continue; }
        result.ok=false; result.reason="cancelled"; result.counted=false;
      }
      if(!current(cmd.epoch)) { result.reason="cancelled"; result.counted=false; }
      phase(Phase::Cleanup); facade.stop(); plain.stop(); secure.stop(); online=false;
      workerSample(true); // Include cleanup stack depth before becoming idle.
      result.ended=nowMs(); result.when=diag::stamp(); attemptDeadline=0;
      portENTER_CRITICAL(&mux);
      if(status.epoch!=cmd.epoch || status.stop) { result.reason="cancelled"; result.counted=false; }
      if(!strcmp(result.reason,"cancelled")) ++status.cancelled;
      status.requestedLease=false; status.lease=false; status.busy=false; status.connected=false;
      status.state=result.state; status.phase=status.stop ? Phase::Stopped : Phase::Idle;
      finalResult=result; resultReady=true;
      portEXIT_CRITICAL(&mux);
      continue;
    }
    // A revoked success is cleaned even while its result awaits main acknowledgement.
    if(!current(operationEpoch) && facade.ready) {
      phase(Phase::Cleanup); facade.stop(); plain.stop(); secure.stop();
      portENTER_CRITICAL(&mux);
      status.lease=false; status.connected=false; status.busy=false;
      lossReady=true; lossState=-3; // cleanup acknowledgement also for revoked READY
      status.phase=status.stop ? Phase::Stopped : Phase::Idle;
      portEXIT_CRITICAL(&mux); online=false;
    }
    if(waiting || !online) continue;
    operationDeadline=nowMs()+5000;
    const bool alive=client.loop() && operationAllowed();
    if(!alive) {
      const int state=client.state();
      phase(Phase::Cleanup); facade.stop(); plain.stop(); secure.stop();
      portENTER_CRITICAL(&mux);
      status.connected=false; status.busy=false; status.state=state; status.phase=status.stop ? Phase::Stopped : Phase::Idle;
      clearMessagesLocked(); lossState=state; lossReady=true;
      portEXIT_CRITICAL(&mux); online=false; continue;
    }
    Tx message{}; bool send=false;
    portENTER_CRITICAL(&mux);
    if(txCount) { message=queues->tx[txHead]; txHead=(txHead+1)%4; --txCount; send=true; }
    portEXIT_CRITICAL(&mux);
    if(send && message.epoch==operationEpoch && current(operationEpoch)) {
      operationDeadline=nowMs()+5000;
      const char* topic=message.topic==0 ? config.motionTopic : message.topic==1 ? config.imuTopic : "companion/calibration";
      const bool accepted=client.publish(topic,message.payload);
      Sent sent{}; sent.epoch=operationEpoch; sent.when=diag::stamp(); sent.accepted=accepted;
      memcpy(sent.category,message.category,sizeof(sent.category)); memcpy(sent.trigger,message.trigger,sizeof(sent.trigger));
      portENTER_CRITICAL(&mux);
      if(accepted) ++status.txAccepted; else ++status.txRejected;
      if(sentCount==4) ++status.completionDrops;
      else { queues->sent[(sentHead+sentCount)%4]=sent; ++sentCount; }
      portEXIT_CRITICAL(&mux);
    }
  }
}
} // namespace

const char* phaseName(Phase value) {
  switch(value) {
    case Phase::Dns:return "dns"; case Phase::Lease:return "lease"; case Phase::Tcp:return "tcp_setup";
    case Phase::Tls:return "tls"; case Phase::Mqtt:return "mqtt_exchange"; case Phase::Subscribe:return "subscribe";
    case Phase::Online:return "online"; case Phase::Cleanup:return "cleanup"; case Phase::Fault:return "fault_held";
    case Phase::Stopped:return "stopped"; default:return "idle";
  }
}
bool init(const NetConfig& value) { config=value; return true; }
View view() { portENTER_CRITICAL(&mux); const View copy=status; portEXIT_CRITICAL(&mux); return copy; }
void linkEvent(bool up) {
  portENTER_CRITICAL(&mux);
  // A repeated GOT_IP (e.g. DHCP renewal) is not a new association. Genuine
  // same-IP recovery first passes through CONNECTED/DISCONNECTED with link=false.
  if(!up || !status.link) { invalidateLocked(false); status.link=up; }
  portEXIT_CRITICAL(&mux);
}
void invalidate(bool stopping) {
  portENTER_CRITICAL(&mux); invalidateLocked(stopping); portEXIT_CRITICAL(&mux);
}
bool request(uint32_t id,int connection,bool test) {
  if(allocationFailed) return false;
  if(!workerHandle) {
    queues=static_cast<Queues*>(heap_caps_calloc(1,sizeof(Queues),MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    workerStack=static_cast<StackType_t*>(heap_caps_malloc(STACK_BYTES,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
    const bool placement=queues && workerStack && esp_ptr_external_ram(workerStack) && esp_ptr_internal(&workerTcb);
    if(placement) workerHandle=xTaskCreateStaticPinnedToCore(worker,"mqtt_owner",STACK_BYTES,nullptr,1,workerStack,&workerTcb,0);
    if(!workerHandle) {
      heap_caps_free(queues); heap_caps_free(workerStack); queues=nullptr; workerStack=nullptr; allocationFailed=true;
      portENTER_CRITICAL(&mux); status.allocationFailed=true; portEXIT_CRITICAL(&mux); return false;
    }
    portENTER_CRITICAL(&mux); status.tcbInternal=true; portEXIT_CRITICAL(&mux);
  }
  const uint64_t at=nowMs();
  portENTER_CRITICAL(&mux);
  const bool allowed=!status.stop && status.link && !status.busy && !status.connected && !status.lease && !resultReady && !lossReady && status.phase!=Phase::Fault;
  if(allowed) {
    ++status.epoch; // A new connection also gets a distinct message generation.
    pending={id,status.epoch,at,connection,test}; commandReady=true;
    status.id=id; status.attemptEpoch=status.epoch; status.started=at; status.phaseAt=at;
    status.busy=true; status.phase=Phase::Dns;
    status.internalMin=status.largestMin=status.dmaMin=status.dmaLargestMin=UINT32_MAX;
  }
  portEXIT_CRITICAL(&mux); return allowed;
}
void arbitrate(bool available) {
  portENTER_CRITICAL(&mux);
  if(status.requestedLease) {
    status.lease=available && status.busy && status.attemptEpoch==status.epoch && status.link && !status.stop && status.phase==Phase::Lease;
    status.requestedLease=false;
  }
  if(status.busy && status.phase!=Phase::Fault && nowMs()-status.started>=STUCK_MS) {
    ++status.epoch; ++status.stuck; status.phase=Phase::Fault; status.connected=false; // Logical only: no release/free.
  }
  portEXIT_CRITICAL(&mux);
}
void sample() {
  const auto before=view();
  if(!before.busy) return; // Neither caller walks heaps during idle/ONLINE operation.
  const uint32_t free=heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
  const uint32_t largest=heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
  const uint32_t dma=heap_caps_get_free_size(MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
  const uint32_t dmaLargest=heap_caps_get_largest_free_block(MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
  portENTER_CRITICAL(&mux);
  if(status.busy && status.id==before.id) {
    status.internalMin=min(status.internalMin,free); status.largestMin=min(status.largestMin,largest); status.dmaMin=min(status.dmaMin,dma); status.dmaLargestMin=min(status.dmaLargestMin,dmaLargest);
  }
  portEXIT_CRITICAL(&mux);
}
bool takeResult(Result& result) {
  portENTER_CRITICAL(&mux); const bool ready=resultReady;
  if(ready) { result=finalResult; resultReady=false; }
  portEXIT_CRITICAL(&mux); return ready;
}
void acknowledgeReady(uint32_t epoch) {
  portENTER_CRITICAL(&mux);
  if(status.epoch==epoch && status.link && !status.stop && status.phase==Phase::Online) {
    status.connected=true; status.lease=false; readyAck=true;
  }
  portEXIT_CRITICAL(&mux);
}
bool takeLoss(int& state) {
  portENTER_CRITICAL(&mux); bool ready=lossReady;
  if(ready) { state=lossState; lossReady=false; }
  portEXIT_CRITICAL(&mux); return ready;
}
bool takeRx(Rx& message) {
  portENTER_CRITICAL(&mux); bool ready=rxCount!=0;
  if(ready) { message=queues->rx[rxHead]; rxHead=(rxHead+1)%4; --rxCount; ready=message.epoch==status.epoch && status.connected && !status.stop; if(!ready) ++status.rxDrops; }
  portEXIT_CRITICAL(&mux); return ready;
}
bool publish(uint8_t topic,const char* payload,const char* category,const char* trigger) {
  const char* name=topic==0 ? config.motionTopic : topic==1 ? config.imuTopic : "companion/calibration";
  if(!name || !payload || !category || !trigger) return false;
  const size_t size=strnlen(payload,768);
  const bool fits=size<768 && 7+strlen(name)+size<=WIRE_BYTES && strlen(category)<16 && strlen(trigger)<16;
  const uint64_t at=nowMs();
  portENTER_CRITICAL(&mux);
  const bool accepted=fits && queues && status.connected && !status.stop && txCount<4;
  if(accepted) {
    Tx& message=queues->tx[(txHead+txCount)%4];
    message.epoch=status.epoch; message.at=at; message.topic=topic; message.length=size;
    memcpy(message.payload,payload,size+1); strcpy(message.category,category); strcpy(message.trigger,trigger); ++txCount;
  } else { ++status.txDrops; if(!fits) ++status.txOversize; }
  portEXIT_CRITICAL(&mux); return accepted;
}
bool takeSent(Sent& completion) {
  portENTER_CRITICAL(&mux); const bool ready=sentCount!=0;
  if(ready) { completion=queues->sent[sentHead]; sentHead=(sentHead+1)%4; --sentCount; }
  portEXIT_CRITICAL(&mux); return ready;
}
} // namespace mqttowner
