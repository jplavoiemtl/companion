'use strict';
// Execute changed C++ bodies through the project's source-adaptation harness.
// This is not a firmware compile or an ESP32 scheduling/ABI test.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const read=p=>fs.readFileSync(p,'utf8').replace(/\r/g,'');
const net=read('src/net/net_module.cpp'),worker=read('src/net/net_worker.cpp'),header=read('src/net/net_worker.h'),sketch=read('companion.ino');
const strip=s=>s.replace(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|\/\*[\s\S]*?\*\//g,m=>m.startsWith('//')||m.startsWith('/*')?' ':m);
function body(s,sig){s=strip(s);let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let a=++i,d=1;while(d){if(s[i]==='{')d++;if(s[i]==='}')d--;i++;}return s.slice(a,i-1);}
const adapt=s=>s.replace(/\bconst (?:bool|uint\d+_t|String) /g,'const ').replace(/\b(?:uint\d+_t|int|bool) /g,'let ').replace(/BenchPhase::/g,'BenchPhase.').replace(/Phase::/g,'Phase.').replace(/diagnet::/g,'diagnet.').replace(/nullptr/g,'null');
let count=0;function test(n,f){f();count++;console.log('PASS '+n);}
function policy(){
 const c={t:0,observedConnected:false,onlineAdoptedAtMs:0,everConnected:false,failureCount:0,benchPhase:'Idle',BenchPhase:{Idle:'Idle',Outage:'Outage',Restoring:'Restoring'},restorePending:false,benchFirstPending:false,lastMqttAttempt:0,MQTT_RECONNECT_INTERVAL:15000,BENCH_FIRST_ATTEMPT_MS:5000,PROMPT_MIN_ONLINE_MS:60000,lostState:-3,configuredConnection:1,activePort:9735,diagnet:{event(){},mqttLoss(){}},netMqttState:()=>0,mqttowner:{invalidate(){}}};
 c.esp_timer_get_time=()=>c.t*1000;c.millis=()=>c.t>>>0;vm.createContext(c);
 vm.runInContext(`function loss(){${adapt(body(net,'if(mqttowner::takeLoss(lostState))'))}}`,c);
 const adoption=body(net,'else if(netIsMqttConnected())');
 const stamps=adoption.slice(adoption.indexOf('everConnected=true;'),adoption.indexOf('const char* categories'));
 vm.runInContext(`function adopt(){${stamps}}`,c);
 vm.runInContext(`function intentionalDisconnect(reason){${adapt(body(net,'void intentionalDisconnect(')).replace(/mqttowner::/g,'mqttowner.')}}`,c);
 const result=body(net,'if(mqttowner::takeResult(result))');
 const backoff=result.slice(result.indexOf('lastMqttAttempt = millis();'),result.indexOf('if(benchPhase!=BenchPhase::Idle)'));
 vm.runInContext(`function completion(){${backoff}}`,c);
 c.remaining=()=>{const elapsed=((c.t>>>0)-(c.lastMqttAttempt>>>0))>>>0;return Math.max(0,15000-elapsed);};return c;
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
 const c=policy();Object.assign(c,{status:{phase:'Online',busy:false,started:0,epoch:1,stop:false,connected:true,link:true},Phase:{Online:'Online'},nowMs:()=>c.t,clearMessagesLocked(){}});
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
