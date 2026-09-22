'use strict';
// Host source simulations, not a firmware compile or lwIP/RTOS proof.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const http=fs.readFileSync('src/diagnostics/diagnostics_http.cpp','utf8');
const cache=fs.readFileSync('src/diagnostics/diagnostics_inventory.cpp','utf8');
const sd=fs.readFileSync('src/diagnostics/sd_diagnostics.cpp','utf8');
const usb=fs.readFileSync('src/diagnostics/diagnostics_usb.cpp','utf8');
function body(s,sig){let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);const a=++i;let d=1;while(d){if(s[i]=='{')d++;if(s[i]=='}')d--;i++;}return s.slice(a,i-1);}
function adapt(s){return s.replace(/portENTER_CRITICAL\(&mux\);|portEXIT_CRITICAL\(&mux\);/g,'')
.replace(/io->/g,'io.').replace(/\*io(?=[,)])/g,'io').replace(/nullptr/g,'null')
.replace(/const (?:bool|int|uint64_t|uint32_t|auto) /g,'const ').replace(/SessionIo\* io =/g,'let io =')
.replace(/diagtransfer::/g,'diagtransfer.').replace(/Phase::/g,'Phase.').replace(/const Shared (\w+) =/g,'const $1 =')
.replace(/const auto\* /g,'const ').replace(/auto\* /g,'const ').replace(/Slot& s =/g,'const s =').replace(/View v;/g,'let v = makeView();')
.replace(/slots\[0\] = \{\}/g,'slots[0] = makeSlot()').replace(/slots\[1\] = \{\}/g,'slots[1] = makeSlot()');}
function install(c,s,name,sig,args){vm.runInContext(`function ${name}(${args}){${adapt(body(s,sig))}}`,c);}
function context(){
 const c={IO_MS:5000,HTTPD_SOCK_ERR_FAIL:-1,MSG_DONTWAIT:8,EAGAIN:11,EWOULDBLOCK:11,EINTR:4,errno:0,t:0,
 io:{used:true,fd:7,generation:1,receiveStarted:false,headerStarted:false,headerComplete:false,outputStarted:false},
 diagtransfer:{view:()=>({})},strcmp:(a,b)=>a===b?0:1,cancel:false,steps:[],calls:[],napHook:null};
 c.nowMs=()=>c.t;c.cancelled=()=>c.cancel;c.session=()=>c.io;c.nap=()=>{c.t+=20;if(c.napHook)c.napHook();};
 c.send=c.recv=(fd,buf,len,flags)=>{c.calls.push({fd,len,flags});const x=c.steps.shift()??{n:-1,e:11};c.errno=x.e??0;return x.n;};
 vm.createContext(c);install(c,http,'receiveExpired','bool receiveExpired(','io,now');
 install(c,http,'receive','int receive(','hd,fd,buf,length,flags');install(c,http,'transmit','int transmit(','hd,fd,buf,length,flags');return c;
}
let count=0;function test(name,fn){fn();count++;console.log('PASS '+name);}
test('positive partial sends preserve caller flags and return exact count',()=>{const c=context();c.steps=[{n:17}];assert.equal(c.transmit(1,7,'body',100,2),17);assert.equal(c.calls[0].flags,10);});
test('send retry yields then returns positive progress',()=>{const c=context();c.steps=[{n:-1,e:11},{n:3}];assert.equal(c.transmit(1,7,'abc',3,0),3);assert.equal(c.t,20);});
test('send cancellation interrupts retry without another socket access',()=>{const c=context();c.napHook=()=>c.cancel=true;assert.equal(c.transmit(1,7,'abc',3,0),-1);assert.equal(c.calls.length,1);});
test('output deadline spans distinct calls including timestamp zero',()=>{const c=context();c.steps=[{n:1}];assert.equal(c.transmit(1,7,'a',1,0),1);c.t=5000;assert.equal(c.transmit(1,7,'b',1,0),-1);assert.equal(c.calls.length,1);});
test('file body deadline slides only on positive send while header deadline remains absolute',()=>{const c=context();c.io.bodyOutput=true;c.steps=[{n:1}];assert.equal(c.transmit(1,7,'a',1,0),1);c.t=4999;c.steps=[{n:1}];assert.equal(c.transmit(1,7,'b',1,0),1);assert.equal(c.io.outputAt,4999);c.t=9999;assert.equal(c.transmit(1,7,'c',1,0),-1);});
test('writer cancellation interrupts a blocked body send before another socket call',()=>{const c=context();c.io.transfer=9;c.io.bodyOutput=true;let closed=false;c.diagtransfer.view=()=>({id:9,closed,result:'pruned'});c.napHook=()=>closed=true;assert.equal(c.transmit(1,7,'a',1,0),-1);assert.equal(c.calls.length,1);});
test('zero send and terminal send error fail without retry',()=>{for(const x of [{n:0},{n:-1,e:99}]){const c=context();c.steps=[x];assert.equal(c.transmit(1,7,'a',1,0),-1);assert.equal(c.calls.length,1);}});
test('EINTR rechecks cancellation',()=>{const c=context();c.steps=[{n:-1,e:4}];c.napHook=()=>c.cancel=true;assert.equal(c.receive(1,7,'',10,0),-1);assert.equal(c.calls.length,1);});
test('initial receive wait has absolute deadline',()=>{const c=context();assert.equal(c.receive(1,7,'',10,0),-1);assert.equal(c.t,5000);});
test('header deadline never slides on incoming bytes',()=>{const c=context();c.steps=[{n:1}];assert.equal(c.receive(1,7,'',10,0),1);c.t=4900;c.steps=[{n:1}];assert.equal(c.receive(1,7,'',10,0),1);c.t=5000;assert.equal(c.receive(1,7,'',10,0),-1);assert.equal(c.calls.length,2);});
test('EOF is distinguished from retry and cancellation',()=>{const c=context();c.steps=[{n:0}];assert.equal(c.receive(1,7,'',10,0),0);assert.equal(c.calls.length,1);});
test('missing context fails before socket use',()=>{const c=context();c.session=()=>null;assert.equal(c.receive(1,7,'',10,0),-1);assert.equal(c.transmit(1,7,'a',1,0),-1);assert.equal(c.calls.length,0);});
function lifecycle(){const c={Phase:{Off:0,Starting:1,Listening:2,Active:3,Failed:4},shared:{generation:5,phase:2,cancel:false,ready:false,userActivity:0},workerHandle:null};vm.createContext(c);install(c,http,'activate','bool activate()','');install(c,http,'idleExpired','bool idleExpired(','now,panelActivity,limit');return c;}
test('activate is conditional on uncancelled listening',()=>{const c=lifecycle();c.shared.cancel=true;assert.equal(c.activate(),false);assert.equal(c.shared.ready,false);c.shared.cancel=false;assert.equal(c.activate(),true);assert.equal(c.shared.phase,3);});
test('atomic expiry considers newest HTTP activity before cancellation',()=>{const c=lifecycle();c.shared.userActivity=299999;assert.equal(c.idleExpired(300000,0,300000),false);assert.equal(c.shared.cancel,false);assert.equal(c.idleExpired(599999,0,300000),true);assert.equal(c.shared.ready,false);});
function inventory(){const c={quiet:true,wanted:false,activeGeneration:7,quietGeneration:7,published:0,valid:true,stale:false,error:'none',freed:[],slots:[{entries:['a'],pins:0,count:1,at:10},{entries:['b'],pins:0,count:0,at:0}]};c.makeSlot=()=>({entries:null,pins:0,count:0,at:0});c.makeView=()=>({entries:null,count:0,at:0,valid:false,stale:true,error:'pending',slot:-1});c.heap_caps_free=x=>{if(x)c.freed.push(x);};vm.createContext(c);install(c,cache,'pin','View pin(','now');install(c,cache,'unpin','void unpin(','v');install(c,cache,'dispose','bool dispose(','generation');return c;}
test('cache pins retain immutable published storage until release',()=>{const c=inventory();const v=c.pin(20);assert.equal(v.count,1);assert.equal(c.slots[0].pins,1);assert.equal(c.dispose(7),false);assert.equal(c.freed.length,0);c.unpin(v);assert.equal(c.dispose(7),true);assert.equal(c.freed.length,2);});
test('cache cleanup requires exact generation acknowledgement',()=>{const c=inventory();assert.equal(c.dispose(6),false);c.quietGeneration=6;assert.equal(c.dispose(7),false);assert.equal(c.freed.length,0);});
test('cache cleanup waits for writer and refuses an enabled cache',()=>{const c=inventory();c.quiet=false;assert.equal(c.dispose(7),false);c.quiet=true;c.wanted=true;assert.equal(c.dispose(7),false);});
test('missing cache is explicitly pending and stale cache age is visible',()=>{const c=inventory();assert.equal(c.pin(5011).stale,true);c.valid=false;c.published=-1;const v=c.pin(6000);assert.equal(v.valid,false);assert.equal(v.slot,-1);assert.equal(v.count,0);});
test('worker owns lifecycle API and retains resources on stop failure',()=>{const b=body(http,'void worker(');assert.match(b,/httpd_start\(&server/);assert.match(b,/while \(snapshot\(\)\.handlers \|\| diagtransfer::busy\(\)\)/);assert.match(b,/if \(httpd_stop\(server\) != ESP_OK\)[\s\S]*?for \(;;\) ulTaskNotifyTake/);assert(b.indexOf('httpd_stop(server)')<b.indexOf('diaginventory::dispose(generation)'));assert(b.indexOf('diaginventory::dispose(generation)')<b.indexOf('heap_caps_free(page)'));});
test('lazy persistent worker requests external stack and internal static TCB',()=>{assert.match(http,/static StaticTask_t workerTcb/);assert.match(body(http,'bool start()'),/heap_caps_malloc\(WORKER_STACK,MALLOC_CAP_SPIRAM \| MALLOC_CAP_8BIT\)/);assert.doesNotMatch(http,/vTaskDelete|shutdown\(/);});
test('accepted sessions restrict interface and attach fresh bounded context',()=>{const b=body(http,'esp_err_t openSession(');assert.match(b,/getsockname/);assert.match(b,/sockaddr_storage local/);assert.match(b,/addressMatches\(local,size,expected\)/);assert.match(b,/\*io = SessionIo\{\}/);assert.match(b,/httpd_sess_set_transport_ctx\(hd,fd,io,freeSession\)/);assert.match(b,/httpd_sess_set_recv_override/);assert.match(b,/httpd_sess_set_send_override/);assert.doesNotMatch(http,/httpd_sess_set_pending_override\(/);});
test('full listing fits bounded PSRAM allocation and sends nonchunked',()=>{assert(256*112+2048+1<=32768);assert.match(http,/heap_caps_malloc\(PAGE_CAP,MALLOC_CAP_SPIRAM \| MALLOC_CAP_8BIT\)/);assert.match(http,/httpd_resp_send\(req,page,used\)/);assert.doesNotMatch(http,/httpd_resp_send_chunk|char page\[/);});
test('listing pin is released before handler sends',()=>{const b=body(http,'bool formatPage(');assert.match(b,/diaginventory::pin/);assert.match(b,/diaginventory::unpin\(v\)/);assert.doesNotMatch(b,/httpd_resp_send|send\(/);});
test('only user views reset activity; HEAD and body-bearing requests do not',()=>{assert.match(body(http,'esp_err_t handle('),/HandlerGuard guard\(io,req->method == HTTP_GET && req->content_len == 0 && \(listing \|\| result\)\)/);assert.match(body(http,'esp_err_t errorHandler('),/,false\)/);});
test('incidental routes do not reserve or read files and every response closes',()=>{const b=body(http,'esp_err_t handle(');assert.match(b,/405 Method Not Allowed/);assert.match(b,/204 No Content/);assert.match(body(http,'esp_err_t download(httpd_req_t* req, SessionIo* io, uint32_t number, bool current) {'),/503 Service Unavailable/);assert.match(b,/404 Not Found/);assert.doesNotMatch(b,/reserve|readChunk|opendir|return ESP_OK/);assert.doesNotMatch(http,/diagreader::(?:view|progress|release|reserve)\(/);});
test('USB reader start preempts directory scan without changing progress',()=>{const b=body(usb,'void start(const diagreader::Accepted&');assert(b.indexOf('diaginventory::writerPreempt()')<b.indexOf('diagreader::start('));assert.match(sd,/diaginventory::writerOffline\(\);\s*diagnosticsUsbStop/);assert.match(sd,/if \(diagnosticsUsbPaused\(\)\) \{\s*diaginventory::writerTick/);});
test('inventory scan has bounded batches, USB preemption and exact cleanup acknowledgement',()=>{const b=body(cache,'void writerTick(');assert.match(b,/batch < 8/);assert.match(b,/diagreader::busy\(\)/);assert.match(b,/seen > diagreader::MAX_ENTRIES/);assert.match(b,/quietGeneration = activeGeneration/);assert.doesNotMatch(cache,/diagreader::reserve|httpd_|send\(/);});
test('notice belongs to top layer and clears only on non-stuck main call',()=>{const b=body(http,'void notice(');assert.match(b,/lv_label_create\(lv_layer_top\(\)\)/);assert.match(b,/if \(!stuck\).*lv_obj_del\(label\); label = nullptr/);assert.match(b,/retryAt = nowMs\(\)\+1000/);});

function workerContext(){
 const c={Phase:{Off:0,Starting:1,Listening:2,Active:3,Stopping:4,Failed:5},shared:{phase:1,generation:1,cancel:false,ready:false,handlers:0},
 page:null,server:null,workerTcb:{},PAGE_CAP:32768,CLIENTS:3,MALLOC_CAP_SPIRAM:1,MALLOC_CAP_8BIT:2,MALLOC_CAP_INTERNAL:4,
 pdTRUE:1,portMAX_DELAY:-1,tskNO_AFFINITY:-1,HTTP_ANY:99,ESP_OK:0,HTTPD_ERR_CODE_MAX:2,
 calls:[],allocation:true,inventoryStart:true,inventoryQuiet:true,startResult:0,registerResult:0,stopResult:0,onStart:null,
 openSession:()=>{},httpd_uri_match_wildcard:()=>{},handle:()=>{},errorHandler:()=>{},sampleWorker:()=>{}};
 c.esp_ptr_external_ram=c.esp_ptr_internal=()=>true;c.pxTaskGetStackStart=()=>1;c.pdMS_TO_TICKS=x=>x;
 c.snapshot=()=>({...c.shared});c.cancelled=g=>c.shared.cancel||g!==c.shared.generation;
 c.setStage=x=>c.shared.stage=x;c.setFailure=x=>{c.shared.failure=x;c.shared.cancel=true;c.shared.ready=false;};
 c.heap_caps_malloc=()=>{c.calls.push('allocate');return c.allocation?{}:null;};c.heap_caps_free=x=>{if(x)c.calls.push('free');};
 c.diagtransfer={busy:()=>false};
 c.inv={start:g=>{c.calls.push('inventory_start');return c.inventoryStart;},stop:()=>c.calls.push('inventory_stop'),dispose:g=>{c.calls.push('inventory_dispose');return c.inventoryQuiet;}};
 c.HTTPD_DEFAULT_CONFIG=()=>({});c.httpd_start=()=>{c.calls.push('start');if(c.startResult===0)c.server={};if(c.onStart)c.onStart();return c.startResult;};
 c.httpd_register_uri_handler=()=>c.registerResult;c.httpd_register_err_handler=()=>0;
 c.httpd_stop=()=>{c.calls.push('stop');return c.stopResult;};
 vm.createContext(c);
 let b=adapt(body(http,'void worker(')).replace(/Shared s =/g,'let s =').replace(/uint8_t local =/,'let local =').replace(/&workerTcb/g,'workerTcb').replace(/&local/g,'local')
 .replace(/ulTaskNotifyTake\(([^;]+)\);/g,'yield [$1];').replace(/nap\(\);/g,'yield [20];')
 .replace(/static_cast<char\*>\((heap_caps_malloc\([^;]+\))\)/g,'$1')
 .replace(/httpd_config_t config =/g,'let config =').replace(/httpd_uri_t route\{\};/g,'let route = {};')
 .replace(/static_cast<httpd_method_t>\(HTTP_ANY\)/g,'HTTP_ANY').replace(/static_cast<httpd_err_code_t>\(code\)/g,'code')
 .replace(/bool ok =/g,'let ok =').replace(/int code=/g,'let code=').replace(/&server/g,'server').replace(/&config/g,'config').replace(/&route/g,'route')
 .replace(/diaginventory::/g,'inv.');
 vm.runInContext(`function* worker(){${b}}`,c);c.run=c.worker();c.run.next();return c;
}
test('actual worker rolls back page allocation failure without starting server',()=>{const c=workerContext();c.allocation=false;c.run.next();assert.equal(c.shared.phase,c.Phase.Off);assert.equal(c.shared.failure,'page_allocation');assert(!c.calls.includes('start'));});
test('actual worker rolls back inventory allocation failure',()=>{const c=workerContext();c.inventoryStart=false;c.run.next();assert.equal(c.shared.phase,c.Phase.Off);assert.equal(c.shared.failure,'inventory_start');assert(c.calls.includes('free'));assert(!c.calls.includes('start'));});
test('actual worker rolls back server start failure',()=>{const c=workerContext();c.startResult=-1;c.run.next();assert.equal(c.shared.phase,c.Phase.Off);assert.equal(c.shared.failure,'server_start');assert(!c.calls.includes('stop'));assert(c.calls.includes('free'));});
test('actual worker registration failure stops server before freeing page',()=>{const c=workerContext();c.registerResult=-1;c.run.next();assert.equal(c.shared.phase,c.Phase.Off);assert.equal(c.shared.failure,'route_registration');assert(c.calls.indexOf('stop')<c.calls.indexOf('free'));});
test('actual worker cancellation during start never publishes Listening',()=>{const c=workerContext();c.onStart=()=>c.shared.cancel=true;c.run.next();assert.equal(c.shared.phase,c.Phase.Off);assert(c.calls.includes('stop'));});
test('actual worker cancellation before start acquires no resources',()=>{const c=workerContext();c.shared.cancel=true;c.run.next();assert.equal(c.shared.phase,c.Phase.Off);assert(!c.calls.includes('allocate'));assert(!c.calls.includes('start'));});
test('actual worker waits for handler release before stop and cache acknowledgement before free',()=>{const c=workerContext();c.run.next();assert.equal(c.shared.phase,c.Phase.Listening);c.shared.cancel=true;c.shared.handlers=1;c.run.next();assert(!c.calls.includes('stop'));c.shared.handlers=0;c.inventoryQuiet=false;c.run.next();assert(c.calls.includes('stop'));assert(!c.calls.includes('free'));c.inventoryQuiet=true;c.run.next();assert(c.calls.includes('free'));assert.equal(c.shared.phase,c.Phase.Off);});
test('actual worker stop failure retains handle and storage without retry',()=>{const c=workerContext();c.run.next();c.shared.cancel=true;c.stopResult=-1;c.run.next();assert.equal(c.shared.phase,c.Phase.Failed);assert(c.server);assert(!c.calls.includes('free'));c.run.next();assert.equal(c.calls.filter(x=>x==='stop').length,1);});

test('actual formatter handles 256 maximum-width entries and releases pin on overflow',()=>{
 const c={diagtransfer:{last:()=>({id:0})},PAGE_CAP:32768,t:9999,pins:0,text:'',limit:32768,entries:Array.from({length:256},()=>({number:99999999,size:18446744073709551615n}))};
 c.nowMs=()=>c.t;c.reader={busy:()=>false,nameFor:(entry,name)=>name.value='archive-99999999.log'};
 c.inv={pin:()=>{c.pins++;return {valid:true,stale:false,at:1,count:256,entries:c.entries,error:'none'};},unpin:()=>c.pins--};
 c.append=(used,fmt,...args)=>{let i=0;const out=fmt.replace(/%08lu|%08lX|%llu|%s|%u/g,()=>{const a=args[i++];return String(a&&typeof a==='object'&&'value'in a?a.value:a);});if(used.value+out.length>=c.limit)return false;used.value+=out.length;c.text+=out;return true;};
 vm.createContext(c);
 let b=adapt(body(http,'bool formatPage(')).replace('used = 0;','used.value = 0;').replace(/bool ok =/g,'let ok =').replace(/const auto v =/g,'const v =')
 .replace(/diaginventory::/g,'inv.').replace(/diagreader::/g,'reader.').replace(/\(unsigned long(?: long)?\)/g,'').replace(/unsigned\(/g,'Number(')
 .replace(/size_t i=/g,'let i=').replace(/char name\[24\];/g,'let name = {};').replace(/sizeof\(name\)/g,'24');
 vm.runInContext(`function formatPage(resultOnly,used){${b}}`,c);
 const used={value:0};assert.equal(c.formatPage(false,used),true);assert(used.value<32768);assert.equal((c.text.match(/archive-99999999.log/g)||[]).length,256);assert.equal(c.pins,0);
 c.limit=300;c.text='';assert.equal(c.formatPage(false,{value:0}),false);assert.equal(c.pins,0);
});
test('open rejection avoids SDK double close and has fail-closed I/O before queueing',()=>{const b=body(http,'esp_err_t openSession(');assert.doesNotMatch(b,/return ESP_FAIL/);assert(b.indexOf('httpd_sess_set_recv_override')<b.indexOf('const Shared s'));assert(b.indexOf('httpd_sess_set_send_override')<b.indexOf('const Shared s'));const r=body(http,'esp_err_t rejectSession(');assert.match(r,/httpd_sess_trigger_close/);assert.match(r,/return ESP_OK/);assert.doesNotMatch(r,/(?<!trigger_)close\(fd\)|shutdown\(/);});

function addressContext(){const c={AF_INET:2,AF_INET6:10};c.memcmp=(a,b,n)=>a.slice(0,n).every((v,i)=>v===b[i])?0:1;vm.createContext(c);
 let b=body(http,'bool addressMatches(').replace(/sizeof\(sockaddr_in\)/g,'16').replace(/sizeof\(sockaddr_in6\)/g,'28')
 .replace(/const auto& v4 = reinterpret_cast<const sockaddr_in&>\(local\);/g,'const v4 = local;')
 .replace(/const auto& v6 = reinterpret_cast<const sockaddr_in6&>\(local\);/g,'const v6 = local;')
 .replace(/const uint8_t\* bytes =/g,'const bytes =').replace(/unsigned i=/g,'let i=')
 .replace(/&v4.sin_addr.s_addr/g,'v4.sin_addr.s_addr').replace(/bytes\+12/g,'bytes.slice(12)');
 vm.runInContext(`function addressMatches(local,length,expected){${b}}`,c);return c;
}
const sta=[172,20,10,2];
function v4(ip=sta){return {ss_family:2,sin_addr:{s_addr:ip}};}
function mapped(ip=sta){return {ss_family:10,sin6_addr:{s6_addr:[...Array(10).fill(0),255,255,...ip]}};}
test('native IPv4 local station address is accepted',()=>{const c=addressContext();assert.equal(c.addressMatches(v4(),16,sta),true);});
test('dual-stack IPv4-mapped station address is accepted',()=>{const c=addressContext();assert.equal(c.addressMatches(mapped(),28,sta),true);});
test('native and mapped other-interface addresses remain refused',()=>{const c=addressContext();assert.equal(c.addressMatches(v4([192,168,4,1]),16,sta),false);assert.equal(c.addressMatches(mapped([192,168,4,1]),28,sta),false);});
test('native IPv6 and IPv4-compatible non-mapped addresses remain refused',()=>{const c=addressContext();const native=mapped();native.sin6_addr.s6_addr[0]=0x20;assert.equal(c.addressMatches(native,28,sta),false);const compatible=mapped();compatible.sin6_addr.s6_addr[10]=0;compatible.sin6_addr.s6_addr[11]=0;assert.equal(c.addressMatches(compatible,28,sta),false);});
test('truncated or unknown socket address fails closed',()=>{const c=addressContext();assert.equal(c.addressMatches(v4(),1,sta),false);assert.equal(c.addressMatches(v4(),15,sta),false);assert.equal(c.addressMatches(mapped(),16,sta),false);assert.equal(c.addressMatches({ss_family:99},28,sta),false);});
console.log(`${count} HTTP lifecycle checks passed; source simulations and integration assertions only.`);
