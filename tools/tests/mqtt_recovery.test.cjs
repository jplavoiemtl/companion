'use strict';
// Execute changed C++ bodies through the project's source-adaptation harness.
// This is not a firmware compile or an ESP32 scheduling/ABI test.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const read=p=>fs.readFileSync(p,'utf8').replace(/\r/g,'');
const net=read('src/net/net_module.cpp'),worker=read('src/net/net_worker.cpp'),header=read('src/net/net_worker.h'),sketch=read('companion.ino');
const strip=s=>s.replace(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|\/\*[\s\S]*?\*\//g,m=>m.startsWith('//')||m.startsWith('/*')?' ':m);
function body(s,sig){s=strip(s);let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let a=++i,d=1;while(d){if(s[i]==='{')d++;if(s[i]==='}')d--;i++;}return s.slice(a,i-1);}
const adapt=s=>s.replace(/\bconst (?:bool|uint\d+_t|String) /g,'const ').replace(/\b(?:uint\d+_t|int|bool) /g,'let ').replace(/BenchPhase::/g,'BenchPhase.').replace(/Phase::/g,'Phase.').replace(/ProbeWindow::/g,'ProbeWindow.').replace(/diagnet::/g,'diagnet.').replace(/mqttowner::/g,'mqttowner.').replace(/diagmqtt::/g,'diagmqtt.').replace(/diag::/g,'diag.').replace(/\(unsigned long long\)|\(unsigned long\)/g,'').replace(/\bconst char\* /g,'let ').replace(/\bconst auto /g,'const ').replace(/\bunsigned i=/g,'let i=').replace(/nullptr/g,'null');
let count=0;function test(n,f){f();count++;console.log('PASS '+n);}
function policy(){
 const c={t:0,observedConnected:false,onlineAdoptedAtMs:0,everConnected:false,failureCount:0,benchPhase:'Idle',BenchPhase:{Idle:'Idle',Outage:'Outage',Restoring:'Restoring'},restorePending:false,benchFirstPending:false,lastMqttAttempt:0,MQTT_RECONNECT_INTERVAL:15000,BENCH_FIRST_ATTEMPT_MS:5000,PROMPT_MIN_ONLINE_MS:60000,lostState:-3,configuredConnection:1,activePort:9735,netMqttState:()=>0,
 attemptLinkValid:false,sessionLinkValid:false,linkRetryPending:false,attemptLinkDowns:0,attemptLinkId:0,sessionLinkDowns:0,readyLinkDowns:0,attemptId:0,initialized:true,giveUp:false,MAX_INITIAL_FAILURES:5,lastResult:'none',benchAttempt:0,
 state:{},status:{linkDowns:0,epoch:1,link:true,phase:'Idle',connected:false,busy:false,lease:false,stop:false},Phase:{Idle:'Idle',Online:'Online',Fault:'Fault'},events:[],dispatches:0,clearMessagesLocked(){},ProbeWindow:{MqttConnect:1},diagnosticsProbeBegin(){},diagnosticsProbeEnd(){},diagmqtt:{begin(){},finish(){}},diag:{recordAt(){}},USBSerial:{printf(){},println(){}},calibReportStatus(){},restoreBenchMqtt(){},isSecurePort:()=>true};
 c.diagnet={event:(...args)=>c.events.push(args),mqttLoss(){},associatedConnection:()=>1};
 c.strcmp=(a,b)=>a===b?0:1;c.esp_timer_get_time=()=>c.t*1000;c.millis=()=>c.t>>>0;c.nowMs=()=>c.t;
 c.mqttowner={view:()=>({...c.status}),invalidate:(stop=false)=>c.invalidateLocked(stop),phaseName:()=>'',
 acknowledgeReady:epoch=>{if(c.beforeAck)c.beforeAck();c.acknowledgeReady(epoch);},
 request:(id,connection,test)=>{if(c.beforeRequest)c.beforeRequest();if(!c.allowed(c.status,c.resultReady||false,c.lossReady||false,c.Phase))return false;c.status.id=id;c.status.connected=false;c.status.busy=true;c.status.epoch++;c.dispatches++;if(c.afterRequest)c.afterRequest();return true;}};
 c.netIsMqttConnected=()=>c.status.connected;c.netMqttBusy=()=>c.status.busy||c.status.lease;
 c.allowed=new Function('status','resultReady','lossReady','Phase','return '+body(worker,'bool request(').match(/const bool allowed=([^;]+);/)[1].replace(/Phase::/g,'Phase.')+';');
 vm.createContext(c);
 const tr=s=>adapt(s).replace(/port(?:ENTER|EXIT)_CRITICAL\(&mux\);/g,'');
 for(const [sig,name,args] of [['void invalidateLocked(','invalidateLocked','stopping'],['void linkEvent(','linkEvent','up'],['void acknowledgeReady(','acknowledgeReady','epoch']])vm.runInContext(`function ${name}(${args}){${tr(body(worker,sig))}}`,c);
 for(const [sig,name,args] of [['void clearLinkRetry(','clearLinkRetry',''],['void scheduleRetry(','scheduleRetry','source,linkChanged,stable,report=true'],['void adoptResultRetry(','completion','result={ok:false,reason:"failed",id:attemptLinkId,test:false}'],['void intentionalDisconnect(','intentionalDisconnect','reason'],['void netCheckMqtt(','dispatch','bypassRateLimit=false'],['void netShutdown(','shutdown',''],['void netConfigureMqttClient(','configure','connection']])
 vm.runInContext(`function ${name}(${args}){${adapt(body(net,sig))}}`,c);
 c.cfg={serverPort1:9735,serverPort2:1883};c.adoptResultRetry=c.completion;
 vm.runInContext(`function loss(){${adapt(body(net,'if(mqttowner::takeLoss(lostState))'))}}`,c);
 const adoption=body(net,'else if(netIsMqttConnected())');
 const stamps=adoption.slice(adoption.indexOf('everConnected=true;'),adoption.indexOf('const char* categories'));
 vm.runInContext(`function adopt(){readyLinkDowns=status.linkDowns;${stamps}}`,c);
 let result=body(net,'if(mqttowner::takeResult(result))');
 result=adapt(result).replace('char fields[456];','let fields="";').replace('sizeof(fields)','456').replace('let categories[]={"image","power","energy"};','const categories=["image","power","energy"];');
 c.snprintf=()=>{};
 vm.runInContext(`function takeResult(result){${result}}`,c);
 c.remaining=()=>{const elapsed=((c.t>>>0)-(c.lastMqttAttempt>>>0))>>>0;return Math.max(0,15000-elapsed);};
 c.finish=(overrides={})=>{c.status.busy=false;c.status.lease=false;const r={id:c.attemptId,epoch:c.status.epoch,ok:false,reason:'failed',counted:false,test:false,ended:c.t,started:0,...overrides};c.takeResult(r);};
 return c;
}
for(const age of [0,59999,60000,60001])test('READY adoption at zero: ONLINE '+age+'ms',()=>{
 const c=policy();c.adopt();c.t=age;c.loss();assert.equal(c.remaining(),age>=60000?0:15000);assert.equal(c.observedConnected,false);assert.equal(c.onlineAdoptedAtMs,0);
 c.t+=1;c.loss();assert.equal(c.remaining(),15000); // consumed, not a reusable credit
});
test('short sessions never accumulate; a fresh stable session can earn eligibility',()=>{
 const c=policy();for(let i=0;i<5;i++){c.adopt();c.t+=59000;c.loss();assert.equal(c.remaining(),15000);c.t+=15000;}
 c.adopt();c.t+=60000;c.loss();assert.equal(c.remaining(),0);
});
test('main adoption time, not worker completion or old session, starts the clock',()=>{
 const c=policy();c.t=120000;c.adopt();assert.equal(c.onlineAdoptedAtMs,120000);c.t=179999;c.loss();assert.equal(c.remaining(),15000);
 c.t=195000;c.adopt();assert.equal(c.onlineAdoptedAtMs,195000);c.t=255000;c.loss();assert.equal(c.remaining(),0);
 assert.equal((net.match(/onlineAdoptedAtMs=esp_timer_get_time\(\)\/1000/g)||[]).length,1);
 const ready=body(net,'if(result.ok)');assert(ready.indexOf('acknowledgeReady')<ready.indexOf('netIsMqttConnected()'));assert(ready.indexOf('netIsMqttConnected()')<ready.indexOf('onlineAdoptedAtMs='));
 assert(!body(worker,'void linkEvent(').includes('onlineAdoptedAtMs'));
 assert(!body(net,'void printBenchStatus(').includes('onlineAdoptedAtMs'));
});
test('startup, revoked READY and intentional disconnect get no prompt retry',()=>{
 const c=policy();c.t=999999;c.loss();assert.equal(c.remaining(),15000);
 c.adopt();c.t+=60000;c.intentionalDisconnect('bench_off');assert.equal(c.observedConnected,false);assert.equal(c.onlineAdoptedAtMs,0);c.loss();assert.equal(c.remaining(),15000);
});
test('bench precedence is exclusive, preserving restoration and first-test delay',()=>{
 for(const [restore,first,want] of [[true,true,0],[true,false,0],[false,true,5000],[false,false,15000]]){
 const c=policy();c.adopt();c.t=60000;c.benchPhase='Outage';c.restorePending=restore;c.benchFirstPending=first;c.loss();assert.equal(c.remaining(),want);}
});
test('failure/cancel/defer adoption restores 15s after prompt attempt; explicit restore still bypasses',()=>{
 const c=policy();c.adopt();c.t=60000;c.loss();assert.equal(c.remaining(),0);
 for(const elapsed of [100,5000,10000]){c.t+=elapsed;c.completion();assert.equal(c.remaining(),15000);c.t+=14999;assert.equal(c.remaining(),1);c.t++;assert.equal(c.remaining(),0);}
 c.restorePending=true;c.completion();assert.equal(c.remaining(),0);
});
test('unsigned retry arithmetic works across millis wrap; ONLINE uses monotonic time',()=>{
 const c=policy();c.t=2**32-60000;c.adopt();c.t+=60000;c.loss();assert.equal(c.remaining(),0);
 c.t=2**32-100;c.completion();c.t+=200;assert.equal(c.remaining(),14800);
});
test('same-IP renewal and mid-attempt flaps never re-arm retry or change ONLINE origin',()=>{
 const c=policy();Object.assign(c,{status:{phase:'Online',busy:false,started:0,epoch:1,stop:false,connected:true,link:true,linkDowns:0},Phase:{Online:'Online'},nowMs:()=>c.t,clearMessagesLocked(){}});
 const tr=s=>adapt(s).replace(/port(?:ENTER|EXIT)_CRITICAL\(&mux\);/g,'');
 vm.runInContext(`function invalidateLocked(stopping){${tr(body(worker,'void invalidateLocked('))}} function linkEvent(up){${tr(body(worker,'void linkEvent('))}}`,c);
 c.adopt();c.t=60000;c.linkEvent(true);assert.equal(c.status.epoch,1);assert.equal(c.onlineAdoptedAtMs,0);
 c.linkEvent(false);c.loss();assert.equal(c.remaining(),0);c.linkEvent(true);assert.equal(c.remaining(),0);
 c.t+=100;c.completion();assert.equal(c.remaining(),15000);
 for(let i=0;i<3;i++){c.linkEvent(false);c.linkEvent(true);c.linkEvent(true);assert.equal(c.remaining(),15000);assert.equal(c.observedConnected,false);assert.equal(c.onlineAdoptedAtMs,0);}
});
test('shutdown and busy/link/lease/mailbox gates prevent dispatch even with eligible stamp',()=>{
 const expression=body(worker,'bool request(').match(/const bool allowed=([^;]+);/)[1].replace(/Phase::/g,'Phase.');
 const allowed=new Function('status','resultReady','lossReady','Phase',`return ${expression};`);
 const normal={stop:false,link:true,busy:false,connected:false,lease:false,phase:'Idle'};assert(allowed(normal,false,false,{Fault:'Fault'}));
 for(const [key,value] of [['stop',true],['link',false],['busy',true],['connected',true],['lease',true],['phase','Fault']])assert(!allowed({...normal,[key]:value},false,false,{Fault:'Fault'}));
 assert(!allowed(normal,true,false,{Fault:'Fault'}));assert(!allowed(normal,false,true,{Fault:'Fault'}));
 assert(sketch.includes('if (!imageFetcherIsBusy() && !videoStreamActive()) {\n      netCheckMqtt();'));
});
// P007: main policy uses the actual owner counter, and actual main terminal/dispatch bodies.
function cycle(c){c.linkEvent(false);c.linkEvent(true);}
function start(c){c.dispatch(true);assert.equal(c.attemptLinkValid,true);return c.status.epoch;}
function ready(c){c.status.phase='Online';c.finish({ok:true,reason:'ok',subscriptions:7});assert(c.observedConnected);}

test('P007 S1 counts only usable up-to-down, atomically inside owner mux',()=>{
 const c=policy();c.linkEvent(true);assert.equal(c.status.linkDowns,0);c.linkEvent(false);assert.equal(c.status.linkDowns,1);
 for(let i=0;i<4;i++)c.linkEvent(false);assert.equal(c.status.linkDowns,1);c.linkEvent(true);c.linkEvent(true);assert.equal(c.status.linkDowns,1);
 c.linkEvent(false);assert.equal(c.status.linkDowns,2);
 const b=body(worker,'void linkEvent(');assert(b.indexOf('portENTER_CRITICAL')<b.indexOf('++status.linkDowns'));assert(b.indexOf('++status.linkDowns')<b.indexOf('invalidateLocked'));assert(b.indexOf('invalidateLocked')<b.indexOf('portEXIT_CRITICAL'));
 assert(header.includes('uint32_t linkDowns=0;'));assert(body(worker,'View view(').includes('copy=status'));assert(!net.includes('portMUX'));
});
test('P007 short session link loss adopted after IP return is prompt',()=>{
 const c=policy();c.adopt();c.t=10000;cycle(c);c.t+=3500;c.loss();assert.equal(c.remaining(),0);assert(c.linkRetryPending);
 assert.deepEqual(c.events.at(-1).slice(2),['loss','wifi_return',true,true,0]);
});
test('P007 offline adoption waits only for usable link; failed request retains credit',()=>{
 const c=policy();c.adopt();c.t=1000;c.linkEvent(false);c.loss();c.dispatch();assert.equal(c.dispatches,0);assert(c.linkRetryPending);
 c.linkEvent(true);c.dispatch();assert.equal(c.dispatches,1);assert(!c.linkRetryPending);assert.equal(c.attemptLinkDowns,1);
});
test('P007 link-caused cancelled Result alone is prompt; unrelated cancellation stays 15s',()=>{
 for(const dropped of [false,true]){const c=policy();const epoch=start(c);c.t=1000;if(dropped)cycle(c);c.finish({epoch,reason:'cancelled'});assert.equal(c.remaining(),dropped?0:15000);assert.equal(c.linkRetryPending,dropped);}
});
test('P007 stale READY plus later cleanup preserves a single prompt opportunity',()=>{
 const c=policy();const epoch=start(c);c.t=1000;cycle(c);c.status.phase='Online';c.finish({epoch,ok:true,reason:'ok'});
 assert(!c.observedConnected);assert.equal(c.lastResult,'cancelled');assert.equal(c.remaining(),0);
 c.lossReady=true;c.dispatch();assert.equal(c.dispatches,1);assert(c.linkRetryPending);
 c.lossReady=false;c.t+=5;c.loss();assert.equal(c.remaining(),0);c.dispatch();assert.equal(c.dispatches,2);assert(!c.linkRetryPending);
 c.t+=10;c.finish({reason:'cancelled'});assert.equal(c.remaining(),15000);
});
test('P007 drop between stale validation and READY acknowledgement retains attempt baseline',()=>{
 const c=policy();start(c);c.status.phase='Online';c.beforeAck=()=>cycle(c);c.finish({ok:true,reason:'ok'});
 assert(!c.observedConnected);assert(c.attemptLinkValid);assert(!c.sessionLinkValid);c.loss();assert.equal(c.remaining(),0);
});
test('P007 READY snapshot precedes acknowledgement; post-snapshot cycle cannot hide loss',()=>{
 const b=body(net,'if(result.ok)');assert(b.indexOf('readyLinkDowns=mqttowner::view()')<b.indexOf('acknowledgeReady'));
 const c=policy();start(c);ready(c);const serial=c.sessionLinkDowns;cycle(c);assert.notEqual(c.status.linkDowns,serial);c.loss();assert.equal(c.remaining(),0);
});
test('P007 down racing with request refuses without consuming eligibility',()=>{
 const c=policy();c.adopt();cycle(c);c.loss();c.beforeRequest=()=>c.linkEvent(false);c.dispatch();assert.equal(c.dispatches,0);assert(c.linkRetryPending);
 c.beforeRequest=null;c.linkEvent(true);c.dispatch();assert.equal(c.dispatches,1);assert(!c.linkRetryPending);
});
test('P007 whole cycle during dispatch snapshot gap remains visible to cancellation',()=>{
 const c=policy();c.beforeRequest=()=>cycle(c);start(c);assert.equal(c.attemptLinkDowns,0);assert.equal(c.status.linkDowns,1);
 c.finish({reason:'cancelled'});assert.equal(c.remaining(),0);
 const d=policy();d.beforeRequest=()=>cycle(d);start(d);d.finish({reason:'tcp_failed'});assert.equal(d.remaining(),15000);assert(!d.attemptLinkValid);
});
test('P007 immediate post-accept drop is not swallowed by a post-request snapshot',()=>{
 const c=policy();c.afterRequest=()=>cycle(c);start(c);assert.equal(c.attemptLinkDowns,0);c.finish({reason:'cancelled'});assert.equal(c.remaining(),0);
});
test('P007 bursts coalesce; a subsequent genuine failure cannot reuse old credit',()=>{
 const c=policy();c.adopt();for(let i=0;i<4;i++)cycle(c);c.loss();assert(c.linkRetryPending);
 c.status.lease=true;c.dispatch();assert.equal(c.dispatches,0);cycle(c);c.status.lease=false;c.dispatch();assert.equal(c.dispatches,1);assert.equal(c.attemptLinkDowns,5);
 c.finish({reason:'tls_failed'});assert.equal(c.remaining(),15000);assert(!c.attemptLinkValid);assert(!c.linkRetryPending);
 cycle(c);c.dispatch();assert.equal(c.dispatches,1);assert.equal(c.remaining(),15000);c.t+=15000;c.dispatch();assert.equal(c.dispatches,2);
});
test('P007 fresh drop during a replacement attempt can earn one fresh opportunity',()=>{
 const c=policy();c.adopt();cycle(c);c.loss();c.dispatch();cycle(c);c.finish({reason:'cancelled'});c.dispatch();assert.equal(c.dispatches,2);
 c.finish({reason:'cancelled'});c.dispatch();assert.equal(c.dispatches,2);assert.equal(c.remaining(),15000);
});
test('P007 new READY clears old link history; short broker kick retains backoff',()=>{
 const c=policy();c.adopt();cycle(c);c.loss();c.dispatch();ready(c);assert.equal(c.sessionLinkDowns,1);assert(!c.attemptLinkValid);assert(!c.linkRetryPending);
 c.t+=10000;c.status.connected=false;c.loss();assert.equal(c.remaining(),15000);
});
test('P007 counter wrap uses inequality rather than ordering',()=>{
 const c=policy();c.status.linkDowns=4294967295;c.adopt();c.linkEvent(false);c.status.linkDowns>>>=0;c.linkEvent(true);assert.equal(c.status.linkDowns,0);c.loss();assert.equal(c.remaining(),0);
});
test('P007 cancellation without matching accepted id cannot earn credit',()=>{
 const c=policy();start(c);cycle(c);c.finish({id:99,reason:'cancelled'});assert.equal(c.remaining(),15000);assert(!c.linkRetryPending);
});
test('P007 bench phases ignore link credit on Result and retain restore/first precedence',()=>{
 for(const phase of ['Outage','Restoring'])for(const [restore,first,want] of [[true,true,0],[false,true,5000],[false,false,15000]]){
 const c=policy();c.benchPhase=phase;c.restorePending=restore;c.benchFirstPending=first;start(c);
 // Dispatch consumes bench flags; establish a new control request during this attempt.
 c.restorePending=restore;c.benchFirstPending=first;cycle(c);c.finish({reason:'cancelled',test:phase==='Outage'});assert.equal(c.remaining(),want);assert(!c.linkRetryPending);
 c.loss();assert.equal(c.remaining(),want);assert(!c.linkRetryPending);
 }
});
test('P007 intentional resets clear credit and baselines, late callbacks cannot revive dispatch on shutdown',()=>{
 for(const action of ['intentionalDisconnect','configure','shutdown']){
 const c=policy();c.adopt();cycle(c);c.loss();assert(c.linkRetryPending);c[action](action==='configure'?1:'bench_off');
 assert(!c.linkRetryPending);assert(!c.attemptLinkValid);assert(!c.sessionLinkValid);cycle(c);c.loss();assert.equal(c.remaining(),15000);
 if(action==='shutdown'){c.dispatch(true);assert.equal(c.dispatches,0);assert(c.status.stop);}
 }
});
test('P007 initial failure budget and cancellation debit are unchanged',()=>{
 const c=policy();for(let i=0;i<5;i++){start(c);cycle(c);c.finish({reason:'cancelled',counted:false});assert.equal(c.failureCount,i);start(c);c.finish({reason:'tcp_failed',counted:true});}
 assert.equal(c.failureCount,5);assert(c.giveUp);const before=c.dispatches;cycle(c);c.linkRetryPending=true;c.dispatch(true);assert.equal(c.dispatches,before);
});
test('P007 successful READY suppresses retry record without changing the scheduled stamp',()=>{
 const c=policy();start(c);c.t=1000;ready(c);
 assert.equal(c.remaining(),15000);assert(c.status.connected);
 assert.equal(c.events.filter(e=>e[0]==='MQTT_RETRY_DECISION').length,0);
 const d=policy();start(d);d.status.phase='Online';d.beforeAck=()=>cycle(d);d.finish({ok:true,reason:'ok'});
 assert.equal(d.events.filter(e=>e[0]==='MQTT_RETRY_DECISION').length,0);
 d.loss();const records=d.events.filter(e=>e[0]==='MQTT_RETRY_DECISION');assert.equal(records.length,1);assert.equal(records[0][3],'wifi_return');
});
test('P007 stale READY still records cancellation retry; resource deferrals remain uncounted 15s',()=>{
 const c=policy();const epoch=start(c);cycle(c);c.finish({epoch,ok:true,reason:'ok'});
 const records=c.events.filter(e=>e[0]==='MQTT_RETRY_DECISION');assert.equal(records.length,1);assert.equal(records[0][3],'wifi_return');
 for(const reason of ['lease_deferred','memory_refused','dns_slots_busy']){
 const d=policy();d.adopt();cycle(d);d.loss();d.dispatch();d.finish({reason,counted:false});
 assert.equal(d.remaining(),15000);assert.equal(d.failureCount,0);assert(!d.linkRetryPending);assert.equal(d.lastResult,reason);
 const rec=d.events.filter(e=>e[0]==='MQTT_RETRY_DECISION').at(-1);assert.equal(rec[3],'backoff');assert.equal(rec.at(-1),15000);
 }
});
test('P007 no synthetic event or periodic polling changes the retry stamp',()=>{
 for(const sig of ['void linkEvent('])assert(!body(worker,sig).includes('lastMqttAttempt'));
 for(const sig of ['void netLinkEvent(','void netObserveRetryPolicy(','void netMqttHealth('])assert(!body(net,sig).includes('scheduleRetry'));
 const calls=(net.match(/scheduleRetry\(/g)||[]).length;assert.equal(calls,3); // definition plus result and loss/cleanup
 assert(body(net,'void scheduleRetry(').includes('"MQTT_RETRY_DECISION"'));
 const fmt=net.match(/"MQTT_RETRY_DECISION","([^"]+)"/)[1];assert(fmt.replace(/%s/g,'bench_first').replace(/%lu|%u/g,'4294967295').length<456);
});

test('new constants and phase uses preserve 5s lifetime/ONLINE and full admission budgets',()=>{
 assert(net.includes('PROMPT_MIN_ONLINE_MS=60000'));
 for(const s of ['DNS_MS=15000','ATTEMPT_MS=45000','STUCK_MS=50000','SOCKET_MS=5000','TLS_MS=10000','MQTT_MS=10000','SUBSCRIBE_MS=2000'])assert(header.includes(s));
 for(const s of ['Phase::Tcp,SOCKET_MS','Phase::Tls,TLS_MS','Phase::Mqtt,MQTT_MS','Phase::Subscribe,SUBSCRIBE_MS','secure.setHandshakeTimeout(TLS_MS/1000)'])assert(worker.includes(s));
 assert.equal((worker.match(/operationDeadline=nowMs\(\)\+SOCKET_MS;/g)||[]).length,2);
 assert.equal((worker.match(/setConnectionTimeout\(5000\)/g)||[]).length,2);
 let b=adapt(body(worker,'bool admitPhase('));
 const c={t:0,current:()=>true,nowMs:()=>c.t,ATTEMPT_MS:45000,phase(){},operationDeadline:0};vm.createContext(c);
 // Same real function with an explicit allowance parameter.
 vm.runInContext(`function admit(cmd,value,allowance,result){${b}}`,c);
 for(const allowance of [5000,10000,2000]){const cmd={epoch:1,started:1000};c.t=46000-allowance;assert(c.admit(cmd,'phase',allowance,{}));assert.equal(c.operationDeadline,46000);c.t++;const r={};assert(!c.admit(cmd,'phase',allowance,r));assert.equal(r.reason,'attempt_budget');assert.equal(cmd.started,1000);}
});
test('connect scope executes 10s setting then restores 5s on success/failure/cancellation',()=>{
 const enter=body(worker,'explicit ConnectTimeoutScope('),leave=body(worker,'~ConnectTimeoutScope(');
 const exchange=body(worker,'bool connectExchange(');assert(exchange.includes('ConnectTimeoutScope timeout(client);'));
 const b=exchange.replace('ConnectTimeoutScope timeout(client);',enter+'\ntry {')+'\n} finally {'+leave+'}';
 const run=new Function('client','test','MQTT_MS','SOCKET_MS','CLIENT_ID','USERNAME','KEY',b);
 for(const outcome of [true,false,'cancelled'])for(const testEndpoint of [false,true]){
 const seen=[],client={timeout:5,setSocketTimeout(n){this.timeout=n;seen.push(n);},connect(...args){assert.equal(this.timeout,10);assert.equal(args.length,testEndpoint?1:3);return outcome===true;}};
 assert.equal(run(client,testEndpoint,10000,5000,'id','user','key'),outcome===true);assert.deepEqual(seen,[10,5]);assert.equal(client.timeout,5);
 }
 const w=body(worker,'void worker(');assert(w.indexOf('connectExchange(client,cmd.test)')<w.indexOf('Phase::Subscribe'));assert(w.indexOf('Phase::Subscribe')<w.indexOf('finalResult=result; resultReady=true;'));
});
function profile(priority=1,equal=false){
 const c={g_mqttConfiguredFromWifi:false,primarySsid:priority===1?'one':'two',secondarySsid:priority===1?'two':'one',primaryNetworkNum:priority,secondaryNetworkNum:3-priority,lastActual:-1,lastRequested:-1,lastInitial:false,WL_CONNECTED:3,connected:true,joined:'one',configured:[],events:[],reads:0};
 if(equal)c.secondarySsid=c.primarySsid;
 c.WiFi={status:()=>c.connected?3:0,SSID:()=>{c.reads++;if(c.loseDuringRead)c.connected=false;return c.joined;}};c.diagnet={event:(...a)=>c.events.push(a)};c.netConfigureMqttClient=n=>c.configured.push(n);vm.createContext(c);
 vm.runInContext(`function joinedMqttNetwork(joined){${adapt(body(sketch,'int joinedMqttNetwork(')).replace('joined.length()','joined.length')}}`,c);
 let b=body(sketch,'bool configureMqttFromJoinedWifi(').replace('static int lastActual=-1, lastRequested=-1;','').replace('static bool lastInitial=false;','');
 vm.runInContext(`function configureMqttFromJoinedWifi(requested,initial){${adapt(b)}}`,c);return c;
}
for(const priority of [1,2])for(const joined of ['one','two'])test('priority '+priority+' joined '+joined+' selects network, not requested role',()=>{
 const c=profile(priority);c.joined=joined;const actual=joined==='one'?1:2;assert(c.configureMqttFromJoinedWifi(3-actual,true));assert.deepEqual(c.configured,[actual]);assert.equal(c.events[0].at(-1),true);
 assert(c.configureMqttFromJoinedWifi(0,false));assert.deepEqual(c.configured,[actual]);assert.equal(c.reads,1);
});
for(const priority of [1,2])test('equal SSIDs select primary network number '+priority,()=>{
 const c=profile(priority,true);c.joined=c.primarySsid;assert(c.configureMqttFromJoinedWifi(priority,true));assert.deepEqual(c.configured,[priority]);assert.equal(c.events[0].at(-1),false);
});
test('unknown/empty/disconnected defers, coalesces and later selects recognized SSID',()=>{
 for(const mode of ['unknown','empty','disconnected','racing']){
 const c=profile(2);c.joined=mode==='unknown'?'other':mode==='empty'?'':'two';if(mode==='disconnected')c.connected=false;if(mode==='racing')c.loseDuringRead=true;
 assert(!c.configureMqttFromJoinedWifi(1,true));assert.equal(c.g_mqttConfiguredFromWifi,false);assert.deepEqual(c.configured,[]);
 for(let i=0;i<10;i++)assert(!c.configureMqttFromJoinedWifi(0,false));assert.equal(c.events.length,2);
 c.connected=true;c.loseDuringRead=false;c.joined='two';assert(c.configureMqttFromJoinedWifi(0,false));assert.deepEqual(c.configured,[2]);assert.equal(c.events.at(-1).at(-1),false);
 }
});
test('initial and late paths share selector; late retry also covers deferred boot selection',()=>{
 assert(body(sketch,'bool connectToWiFi(').includes('configureMqttFromJoinedWifi(connection, true)'));
 const loop=body(sketch,'void loop(');assert(loop.includes('if (!g_mqttConfiguredFromWifi && WiFi.status() == WL_CONNECTED)'));
 assert(loop.includes('configureMqttFromJoinedWifi(0, false)'));assert(!loop.includes('g_mqttConfiguredLate'));
 assert.equal((sketch.match(/netConfigureMqttClient\(/g)||[]).length,1);
 const helper=body(sketch,'bool configureMqttFromJoinedWifi(');assert(!helper.includes('ssid='));assert(!helper.includes('password'));assert(!helper.includes('broker'));
 assert(helper.includes('"source=%s requested=%d actual=%d result=%s mismatch=%u"'));
});
console.log(`${count} MQTT recovery checks passed; source simulations only, no firmware build.`);
