#include "diagnostics_http.h"
#include "diagnostics_inventory.h"
#include "diagnostics_reader.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <esp_heap_caps.h>
#include <esp_memory_utils.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/idf_additions.h>
#include <freertos/portmacro.h>
#include <lwip/sockets.h>
#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <lvgl.h>
#include "HWCDC.h"
extern HWCDC USBSerial;

namespace diaghttp {
namespace {
constexpr size_t WORKER_STACK = 4096, PAGE_CAP = 32768, CLIENTS = 3;
constexpr uint64_t IO_MS = 5000;
static_assert(256*112+2048+1 <= PAGE_CAP,"bounded listing fits PSRAM page");
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
enum class Phase : uint8_t { Off, Starting, Listening, Active, Stopping, Failed };
struct Shared {
  Phase phase = Phase::Off;
  uint64_t generation = 0, userActivity = 0;
  bool cancel = true, ready = false;
  unsigned handlers = 0;
  uint32_t accepted = 0, rejected = 0;
  const char* lastReject = "none";
  const char* failure = nullptr;
  const char* stage = "off";
  uint32_t workerMargin = 0, httpMargin = 0;
  bool stackExternal = false, tcbInternal = false;
} shared;
static StaticTask_t workerTcb;
StackType_t* workerStack = nullptr;
TaskHandle_t workerHandle = nullptr; // main publishes once, never deleted
char* page = nullptr; // worker owns lifetime; HTTP task owns contents
httpd_handle_t server = nullptr; // lifecycle worker only
struct SessionIo {
  uint64_t generation = 0, serial = 0, receiveAt = 0, headerAt = 0, outputAt = 0;
  int fd = -1;
  bool used = false, receiveStarted = false, headerStarted = false, headerComplete = false, outputStarted = false;
};
static_assert(sizeof(SessionIo) <= 96,"bounded per-client state");
SessionIo clients[CLIENTS]; // HTTP-only; component invokes context cleanup on HTTP task
uint64_t connectionSerial = 0;
uint64_t nowMs() { return esp_timer_get_time()/1000; }
void nap() { vTaskDelay(pdMS_TO_TICKS(20) ? pdMS_TO_TICKS(20) : 1); }
Shared snapshot() { portENTER_CRITICAL(&mux); Shared s = shared; portEXIT_CRITICAL(&mux); return s; }
bool cancelled(uint64_t generation) {
  portENTER_CRITICAL(&mux);
  const bool value = shared.cancel || shared.generation != generation;
  portEXIT_CRITICAL(&mux); return value;
}
void setStage(const char* stage) { portENTER_CRITICAL(&mux); shared.stage = stage; portEXIT_CRITICAL(&mux); }
void setFailure(const char* reason) {
  portENTER_CRITICAL(&mux);
  shared.failure = reason; shared.cancel = true; shared.ready = false;
  portEXIT_CRITICAL(&mux);
}
void sampleWorker() {
  const uint32_t margin = uxTaskGetStackHighWaterMark(nullptr);
  portENTER_CRITICAL(&mux); shared.workerMargin = margin; portEXIT_CRITICAL(&mux);
}
void sampleHttp() {
  const uint32_t margin = uxTaskGetStackHighWaterMark(nullptr);
  portENTER_CRITICAL(&mux);
  if (!shared.httpMargin || margin < shared.httpMargin) shared.httpMargin = margin;
  portEXIT_CRITICAL(&mux);
}
SessionIo* session(httpd_handle_t hd, int fd) {
  auto* io = static_cast<SessionIo*>(httpd_sess_get_transport_ctx(hd,fd));
  if (!io || !io->used || io->fd != fd || cancelled(io->generation)) return nullptr;
  return io;
}
void freeSession(void* context) {
  auto* io = static_cast<SessionIo*>(context);
  if (io) *io = SessionIo{};
}
// Terminal expiry maps to FAIL, not TIMEOUT: the parser may retry TIMEOUT.
bool receiveExpired(const SessionIo& io, uint64_t now) {
  if (io.headerComplete) return false;
  if (io.headerStarted) return now-io.headerAt >= IO_MS;
  return io.receiveStarted && now-io.receiveAt >= IO_MS;
}
int receive(httpd_handle_t hd, int fd, char* buf, size_t length, int flags) {
  SessionIo* io = session(hd,fd);
  if (!io) return HTTPD_SOCK_ERR_FAIL;
  if (!length) return 0;
  if (!io->receiveStarted) { io->receiveStarted = true; io->receiveAt = nowMs(); }
  for (;;) {
    if (cancelled(io->generation) || receiveExpired(*io,nowMs())) return HTTPD_SOCK_ERR_FAIL;
    const int n = recv(fd,buf,length,flags | MSG_DONTWAIT);
    if (n > 0) {
      if (!io->headerStarted) { io->headerStarted = true; io->headerAt = nowMs(); }
      return n;
    }
    if (n == 0) return 0;
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return HTTPD_SOCK_ERR_FAIL;
    nap();
  }
}
int transmit(httpd_handle_t hd, int fd, const char* buf, size_t length, int flags) {
  SessionIo* io = session(hd,fd);
  if (!io) return HTTPD_SOCK_ERR_FAIL;
  if (!length) return 0;
  if (!io->outputStarted) { io->outputStarted = true; io->outputAt = nowMs(); }
  for (;;) {
    if (cancelled(io->generation) || nowMs()-io->outputAt >= IO_MS) return HTTPD_SOCK_ERR_FAIL;
    const int n = send(fd,buf,length,flags | MSG_DONTWAIT);
    if (n > 0) return n;
    if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) return HTTPD_SOCK_ERR_FAIL;
    nap();
  }
}
// IDF's dual-stack listener reports IPv4 peers with AF_INET6 mapped addresses.
// Compare network-order octets, never reinterpret native IPv6 as a station IPv4.
bool addressMatches(const sockaddr_storage& local, socklen_t length, const uint8_t expected[4]) {
  if (length < 2) return false; // lwIP sockaddr starts with length and family bytes.
  if (local.ss_family == AF_INET) {
    if (length < sizeof(sockaddr_in)) return false;
    const auto& v4 = reinterpret_cast<const sockaddr_in&>(local);
    return memcmp(&v4.sin_addr.s_addr,expected,4) == 0;
  }
  if (local.ss_family == AF_INET6) {
    if (length < sizeof(sockaddr_in6)) return false;
    const auto& v6 = reinterpret_cast<const sockaddr_in6&>(local);
    const uint8_t* bytes = v6.sin6_addr.s6_addr;
    for (unsigned i=0; i<10; ++i) if (bytes[i] != 0) return false;
    if (bytes[10] != 0xff || bytes[11] != 0xff) return false;
    return memcmp(bytes+12,expected,4) == 0;
  }
  return false;
}
esp_err_t rejectSession(httpd_handle_t hd, int fd, const char* reason) {
  portENTER_CRITICAL(&mux);
  if (shared.rejected != UINT32_MAX) ++shared.rejected;
  shared.lastReject = reason;
  portEXIT_CRITICAL(&mux);
  // IDF 5.5.5 closes twice if open_fn returns failure (sess_new and accept).
  // Keep open_fn successful; close once on the HTTP task through its control queue.
  // Overrides already fail on missing/cancelled context if queueing is unavailable.
  if (httpd_sess_trigger_close(hd,fd) != ESP_OK) setFailure("session_close_queue");
  return ESP_OK;
}
esp_err_t openSession(httpd_handle_t hd, int fd) {
  // Install the fail-closed path before checking interface/readiness or allocating a slot.
  const bool recvSet = httpd_sess_set_recv_override(hd,fd,receive) == ESP_OK;
  const bool sendSet = httpd_sess_set_send_override(hd,fd,transmit) == ESP_OK;
  if (!recvSet || !sendSet) { setFailure("session_hooks"); return rejectSession(hd,fd,"session_hooks"); }
  const Shared s = snapshot();
  if (s.cancel || !s.ready) return rejectSession(hd,fd,"not_ready");
  if (!interfaceAllowed() || WiFi.status() != WL_CONNECTED) return rejectSession(hd,fd,"interface");
  sockaddr_storage local{}; socklen_t size = sizeof(local);
  if (getsockname(fd,reinterpret_cast<sockaddr*>(&local),&size)) return rejectSession(hd,fd,"getsockname");
  const IPAddress ip = WiFi.localIP();
  const uint8_t expected[4] = {ip[0],ip[1],ip[2],ip[3]};
  if (!addressMatches(local,size,expected)) return rejectSession(hd,fd,"local_address");
  if (connectionSerial == UINT64_MAX) return rejectSession(hd,fd,"serial_limit");
  SessionIo* io = nullptr;
  for (auto& candidate : clients) if (!candidate.used) { io = &candidate; break; }
  if (!io) return rejectSession(hd,fd,"session_limit");
  *io = SessionIo{}; io->used = true; io->fd = fd;
  io->generation = s.generation; io->serial = ++connectionSerial;
  httpd_sess_set_transport_ctx(hd,fd,io,freeSession);
  if (httpd_sess_get_transport_ctx(hd,fd) != io) { freeSession(io); return rejectSession(hd,fd,"context"); }
  // Once attached, only component cleanup releases the slot.
  // pending_fn stays null: plain TCP uses select plus parser pending_len.
  portENTER_CRITICAL(&mux); if (shared.accepted != UINT32_MAX) ++shared.accepted; portEXIT_CRITICAL(&mux);
  sampleHttp(); return ESP_OK;
}
struct HandlerGuard {
  bool admitted = false;
  HandlerGuard(SessionIo* io, bool activity) {
    if (!io || receiveExpired(*io,nowMs())) return;
    portENTER_CRITICAL(&mux);
    if (shared.ready && !shared.cancel && shared.generation == io->generation) {
      ++shared.handlers; admitted = true;
      if (activity) shared.userActivity = nowMs();
    }
    portEXIT_CRITICAL(&mux);
    if (admitted) io->headerComplete = true;
  }
  ~HandlerGuard() {
    if (!admitted) return;
    sampleHttp();
    portENTER_CRITICAL(&mux); --shared.handlers; portEXIT_CRITICAL(&mux);
  }
};
bool append(size_t& used, const char* format, ...) {
  if (used >= PAGE_CAP) return false;
  va_list args; va_start(args,format);
  const int n = vsnprintf(page+used,PAGE_CAP-used,format,args); va_end(args);
  if (n < 0 || size_t(n) >= PAGE_CAP-used) return false;
  used += n; return true;
}
// One fixed page, no stack-sized inventory or page. Pin ends before network output.
bool formatPage(bool resultOnly, size_t& used) {
  used = 0;
  bool ok = append(used,"<!doctype html><html><head><meta name=viewport content='width=device-width'><title>Companion logs</title></head><body><h1>Companion logs</h1><p>No HTTP transfer yet.</p><a href='/'>Logs</a> <a href='/result'>Last result</a>");
  if (!resultOnly) {
    const auto v = diaginventory::pin(nowMs());
    const bool busy = diagreader::busy();
    ok = ok && append(used,"<p>Inventory: %s; age %llu ms; stale=%u; busy=%u; status=%s. Sizes are advisory.</p><pre>",
      v.valid ? "available" : "pending",(unsigned long long)(v.valid ? nowMs()-v.at : 0),unsigned(v.stale || busy),unsigned(busy),v.error);
    for (size_t i=0; ok && i<v.count; ++i) {
      char name[24]; diagreader::nameFor(v.entries[i],name,sizeof(name));
      ok = append(used,"%s  %llu bytes\n",name,(unsigned long long)v.entries[i].size);
    }
    diaginventory::unpin(v);
    ok = ok && append(used,"</pre><p>File downloads are not implemented yet.</p>");
  }
  return ok && append(used,"</body></html>");
}
void headers(httpd_req_t* req) {
  httpd_resp_set_hdr(req,"Connection","close");
  httpd_resp_set_hdr(req,"Cache-Control","no-store");
  httpd_resp_set_hdr(req,"Referrer-Policy","no-referrer");
}
// Always return failure after output: documented and binary-verified close path.
esp_err_t handle(httpd_req_t* req) {
  SessionIo* io = session(req->handle,httpd_req_to_sockfd(req));
  const bool listing = !strcmp(req->uri,"/");
  const bool result = !strcmp(req->uri,"/result");
  HandlerGuard guard(io,req->method == HTTP_GET && req->content_len == 0 && (listing || result));
  if (!guard.admitted) return ESP_FAIL;
  headers(req);
  if (req->method != HTTP_GET) {
    httpd_resp_set_status(req,"405 Method Not Allowed");
    httpd_resp_set_hdr(req,"Allow","GET");
    httpd_resp_send(req,nullptr,0); return ESP_FAIL;
  }
  if (req->content_len != 0) {
    httpd_resp_set_status(req,"400 Bad Request");
    httpd_resp_send(req,nullptr,0); return ESP_FAIL;
  }
  if (listing || result) {
    size_t used = 0;
    if (!formatPage(result,used)) {
      httpd_resp_set_status(req,"500 Internal Server Error");
      httpd_resp_send(req,"format_limit",12); return ESP_FAIL;
    }
    httpd_resp_set_type(req,"text/html; charset=utf-8");
    httpd_resp_send(req,page,used); return ESP_FAIL;
  }
  if (!strcmp(req->uri,"/favicon.ico")) httpd_resp_set_status(req,"204 No Content");
  else if (!strncmp(req->uri,"/f/",3)) httpd_resp_set_status(req,"503 Service Unavailable");
  else httpd_resp_set_status(req,"404 Not Found");
  httpd_resp_send(req,nullptr,0); return ESP_FAIL;
}
esp_err_t errorHandler(httpd_req_t* req, httpd_err_code_t code) {
  // Parser errors are SD-free, do not reset activity, and use cancellable output.
  HandlerGuard guard(session(req->handle,httpd_req_to_sockfd(req)),false);
  if (!guard.admitted) return ESP_FAIL;
  headers(req);
  httpd_resp_send_err(req,code,nullptr); return ESP_FAIL;
}
void worker(void*) {
  uint8_t local = 0;
  portENTER_CRITICAL(&mux);
  shared.stackExternal = esp_ptr_external_ram(pxTaskGetStackStart(nullptr)) && esp_ptr_external_ram(&local);
  shared.tcbInternal = esp_ptr_internal(&workerTcb);
  portEXIT_CRITICAL(&mux);
  for (;;) {
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    Shared s = snapshot();
    if (s.phase != Phase::Starting) continue;
    const uint64_t generation = s.generation;
    setStage("startup");
    if (!cancelled(generation)) {
      page = static_cast<char*>(heap_caps_malloc(PAGE_CAP,MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (!page) setFailure("page_allocation");
      else if (!diaginventory::start(generation)) setFailure("inventory_start");
      else if (!cancelled(generation)) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.stack_size = 4096; config.task_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
        config.core_id = tskNO_AFFINITY; config.max_open_sockets = CLIENTS;
        config.lru_purge_enable = false; config.keep_alive_enable = false; config.enable_so_linger = false;
        config.max_resp_headers = 8; config.open_fn = openSession;
        config.uri_match_fn = httpd_uri_match_wildcard;
        if (httpd_start(&server,&config) != ESP_OK) setFailure("server_start");
        else {
          httpd_uri_t route{}; route.uri = "/*"; route.method = static_cast<httpd_method_t>(HTTP_ANY); route.handler = handle;
          bool ok = httpd_register_uri_handler(server,&route) == ESP_OK;
          for (int code=0; ok && code<HTTPD_ERR_CODE_MAX; ++code)
            ok = httpd_register_err_handler(server,static_cast<httpd_err_code_t>(code),errorHandler) == ESP_OK;
          if (!ok) setFailure("route_registration");
          else {
            portENTER_CRITICAL(&mux);
            if (!shared.cancel) { shared.phase = Phase::Listening; shared.stage = "listening"; }
            portEXIT_CRITICAL(&mux);
          }
        }
      }
    }
    sampleWorker();
    while (!cancelled(generation)) { ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(100)); sampleWorker(); }
    diaginventory::stop();
    setStage("handler");
    while (snapshot().handlers) nap();
    if (server) {
      setStage("server_stop");
      if (httpd_stop(server) != ESP_OK) {
        setFailure("server_stop");
        portENTER_CRITICAL(&mux); shared.phase = Phase::Failed; portEXIT_CRITICAL(&mux);
        // No unsafe free/retry on a still-live handle. Main retains exclusion until reboot.
        for (;;) ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
      }
      server = nullptr;
    }
    setStage("inventory_close");
    while (!diaginventory::dispose(generation)) nap();
    heap_caps_free(page); page = nullptr;
    sampleWorker();
    portENTER_CRITICAL(&mux);
    shared.phase = Phase::Off; shared.stage = "off"; shared.ready = false;
    portEXIT_CRITICAL(&mux);
  }
}
} // namespace
bool interfaceAllowed() {
  wifi_mode_t mode = WIFI_MODE_NULL;
  return esp_wifi_get_mode(&mode) == ESP_OK && mode == WIFI_MODE_STA;
}
bool start() {
  const Shared before = snapshot();
  if (before.phase != Phase::Off || before.generation == UINT64_MAX) return false;
  if (!workerHandle) {
    workerStack = static_cast<StackType_t*>(heap_caps_malloc(WORKER_STACK,MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!workerStack) return false;
    if (!esp_ptr_external_ram(workerStack) || !esp_ptr_internal(&workerTcb) || !esp_ptr_byte_accessible(&workerTcb)) {
      heap_caps_free(workerStack); workerStack = nullptr; return false;
    }
    workerHandle = xTaskCreateStaticPinnedToCore(worker,"log_http_ctl",WORKER_STACK,nullptr,1,workerStack,&workerTcb,tskNO_AFFINITY);
    if (!workerHandle) { heap_caps_free(workerStack); workerStack = nullptr; return false; }
  }
  portENTER_CRITICAL(&mux);
  ++shared.generation; shared.phase = Phase::Starting; shared.cancel = false;
  shared.ready = false; shared.failure = nullptr; shared.userActivity = 0; shared.stage = "startup";
  shared.accepted = shared.rejected = 0; shared.lastReject = "none";
  portEXIT_CRITICAL(&mux);
  xTaskNotifyGive(workerHandle); return true;
}
void stop() {
  portENTER_CRITICAL(&mux); shared.cancel = true; shared.ready = false; portEXIT_CRITICAL(&mux);
  if (workerHandle) xTaskNotifyGive(workerHandle);
}
bool activate() {
  portENTER_CRITICAL(&mux);
  const bool ok = shared.phase == Phase::Listening && !shared.cancel;
  if (ok) { shared.ready = true; shared.phase = Phase::Active; shared.stage = "active"; }
  portEXIT_CRITICAL(&mux); return ok;
}
bool stopped() { return snapshot().phase == Phase::Off; }
const char* failure() { return snapshot().failure; }
const char* stage() { return snapshot().stage; }
uint64_t activity() { return snapshot().userActivity; }
bool idleExpired(uint64_t now, uint64_t panelActivity, uint64_t limit) {
  portENTER_CRITICAL(&mux);
  const uint64_t latest = shared.userActivity > panelActivity ? shared.userActivity : panelActivity;
  const bool expired = now >= latest && now-latest >= limit;
  if (expired) { shared.cancel = true; shared.ready = false; }
  portEXIT_CRITICAL(&mux); return expired;
}
void report() {
  const Shared s = snapshot();
  const IPAddress ip = WiFi.localIP();
  USBSerial.printf("[LOG HTTP] server=%s generation=%llu error=%s worker_stack=psram worker_bytes=4096 worker_min=%u worker_external=%u tcb_internal=%u http_min=%u accepted=%u rejected=%u last_reject=%s clients=3 url=http://%u.%u.%u.%u/\n",
    s.stage,(unsigned long long)s.generation,s.failure ? s.failure : "none",unsigned(s.workerMargin),unsigned(s.stackExternal),unsigned(s.tcbInternal),unsigned(s.httpMargin),unsigned(s.accepted),unsigned(s.rejected),s.lastReject,ip[0],ip[1],ip[2],ip[3]);
}
void notice(bool stuck) {
  static lv_obj_t* label = nullptr;
  static uint64_t retryAt = 0;
  if (!stuck) { if (label) { lv_obj_del(label); label = nullptr; } return; }
  if (label || nowMs() < retryAt) return;
  retryAt = nowMs()+1000;
  label = lv_label_create(lv_layer_top());
  if (!label) return;
  lv_obj_set_width(label,lv_disp_get_hor_res(nullptr)-16);
  lv_obj_align(label,LV_ALIGN_TOP_MID,0,8);
  lv_obj_set_style_bg_opa(label,LV_OPA_COVER,0);
  lv_obj_set_style_bg_color(label,lv_color_hex(0x602020),0);
  lv_obj_set_style_text_color(label,lv_color_white(),0);
  lv_obj_clear_flag(label,LV_OBJ_FLAG_CLICKABLE);
  lv_label_set_text(label,"Log download could not close.\nWaiting; restart if it persists.");
}
} // namespace diaghttp
