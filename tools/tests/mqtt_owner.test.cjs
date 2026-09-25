'use strict';
// Real C++ function bodies executed with deterministic platform mocks. No firmware
// compilation. Static ownership/integration guards complement (not replace) simulations.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),path=require('path');
const read=p=>fs.readFileSync(p,'utf8').replace(/\r/g,'');
const worker=read('src/net/net_worker.cpp'),header=read('src/net/net_worker.h');
const net=read('src/net/net_module.cpp'),pub=read('src/net/mqtt_client/OwnedPubSubClient.cpp');
const strip=s=>s.replace(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|\/\*[\s\S]*?\*\//g,m=>m.startsWith('//')||m.startsWith('/*')?' ':m);
function body(s,sig){s=strip(s);let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let a=++i,d=1;while(d){if(s[i]=='{')d++;if(s[i]=='}')d--;i++;}return s.slice(a,i-1);}
let count=0;
function test(name,f){f();++count;console.log('PASS '+name);}
function adapt(s){return s.replace(/port(?:ENTER|EXIT)_CRITICAL\(&mux\);/g,'').replace(/Phase::/g,'Phase.').replace(/\bconst (?:bool|uint\d+_t|err_t|ip_addr_t) /g,'const ').replace(/\b(?:bool|uint\d+_t|err_t) /g,'let ').replace(/slot->/g,'slot.').replace('status.stop|=stopping','status.stop=Boolean(status.stop || stopping)');}
const phases=['Idle','Dns','Lease','Tcp','Tls','Mqtt','Subscribe','Online','Cleanup','Fault','Stopped'];
function context(){const c={t:0,status:{epoch:1,attemptEpoch:1,link:true,stop:false,phase:'Dns',busy:true,connected:false,lease:false,requestedLease:false,started:0,cancelled:0,stuck:0},Phase:Object.fromEntries(phases.map(x=>[x,x])),ATTEMPT_MS:35000,STUCK_MS:40000,readyAck:false,operationDeadline:0,clears:0};c.nowMs=()=>c.t;c.clearMessagesLocked=()=>c.clears++;c.phase=p=>c.status.phase=p;vm.createContext(c);
 for(const [sig,name,args] of [['void linkEvent(','linkEvent','up'],['bool current(','current','epoch'],['void invalidateLocked(','invalidateLocked','stopping'],['bool admitPhase(','admitPhase','cmd,value,allowance,result'],['void arbitrate(','arbitrate','available'],['void acknowledgeReady(','acknowledgeReady','epoch']])vm.runInContext(`function ${name}(${args}){${adapt(body(worker,sig))}}`,c);
 return c;}
test('pins allocation, scheduling and time bounds; no internal fallback',()=>{
 for(const s of ['DNS_MS=15000','ATTEMPT_MS=35000','STUCK_MS=40000','SOCKET_MS=5000'])assert(header.includes(s));
 for(const s of ['STACK_BYTES=12288','WIRE_BYTES=512','INTERNAL_GATE=20480','MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT','&workerTcb,0','nullptr,1,workerStack'])assert(worker.includes(s));
 assert(worker.includes('sizeof(Queues)<=8192'));assert(worker.includes('<=2048'));
 assert.equal((worker.match(/setConnectionTimeout\(5000\)/g)||[]).length,2);
});
for(const phase of phases.filter(x=>!['Idle','Stopped','Fault'].includes(x)))test('same-IP epoch flap cancels '+phase,()=>{
 const c=context();c.status.phase=phase;c.status.connected=phase==='Online';c.status.lease=['Tcp','Tls','Mqtt','Subscribe'].includes(phase);
 const held=c.status.lease;c.invalidateLocked(false);c.status.link=false;c.invalidateLocked(false);c.status.link=true;
 assert.equal(c.current(1),false);assert.equal(c.status.connected,false);assert.equal(c.status.lease,held);assert.equal(c.clears,2);
 c.status.phase='Online';c.acknowledgeReady(1);assert.equal(c.status.connected,false);
});
test('phase admission preserves complete allowance and never changes attempt origin',()=>{
 const c=context(),cmd={epoch:1,started:0},r={};c.t=30000;assert.equal(c.admitPhase(cmd,'Tcp',5000,r),true);assert.equal(c.operationDeadline,35000);
 c.t=30001;assert.equal(c.admitPhase(cmd,'Tls',5000,r),false);assert.equal(r.reason,'attempt_budget');assert.equal(cmd.started,0);
 c.invalidateLocked(false);assert.equal(c.admitPhase(cmd,'Mqtt',5000,r),false);assert.equal(r.reason,'cancelled');
});
test('only pending post-DNS matching lease can grant; stale grants roll back',()=>{
 const c=context();c.status.phase='Lease';c.status.requestedLease=true;c.arbitrate(true);assert.equal(c.status.lease,true);
 c.status.lease=false;c.status.requestedLease=true;c.status.phase='Dns';c.arbitrate(true);assert.equal(c.status.lease,false);
 c.status.phase='Lease';c.status.requestedLease=true;c.invalidateLocked(false);c.arbitrate(true);assert.equal(c.status.lease,false);
});
test('fault is one-shot, held resource stays held; stale READY cannot revive it',()=>{
 for(const held of [false,true]){const c=context();c.status.lease=held;c.t=40000;c.arbitrate(true);assert.equal(c.status.phase,'Fault');assert.equal(c.status.lease,held);assert.equal(c.status.stuck,1);c.t=80000;c.arbitrate(true);assert.equal(c.status.stuck,1);c.acknowledgeReady(1);assert.equal(c.status.connected,false);}
});
test('shutdown invalidates snapshot and queues without releasing worker resources',()=>{
 const c=context();c.status.connected=true;c.status.lease=true;c.invalidateLocked(true);assert.equal(c.status.stop,true);assert.equal(c.status.connected,false);assert.equal(c.status.lease,true);
 assert(!/stop\(|delete|wait|delay/i.test(body(net,'void netShutdown(')));
});
test('valid READY releases lease only after subscriptions; shutdown refuses READY',()=>{
 const c=context();c.status.phase='Online';c.status.lease=true;c.acknowledgeReady(1);assert.equal(c.status.connected,true);assert.equal(c.status.lease,false);assert.equal(c.readyAck,true);
 c.invalidateLocked(true);c.acknowledgeReady(c.status.epoch);assert.equal(c.status.connected,false);
 assert(worker.indexOf('client.subscribe(')<worker.indexOf('finalResult=result; resultReady=true;'));
});
// Execute real DNS resolution and callback state transitions; simulate callback scheduling.
function dnsContext(mode){const c={t:0,alive:true,mode,error:0,ERR_OK:0,ERR_INPROGRESS:-5,ERR_MEM:-1,ERR_VAL:-6,DNS_MS:15000,dnsSlots:[{used:false},{used:false}],Phase:{},ticks:0};
 c.nowMs=()=>c.t;c.current=()=>c.alive;c.strchr=(s,x)=>s.includes(x);c.strlen=s=>s.length;c.IP_IS_V4=a=>a.v4;c.IPAddress=x=>x;c.ip_2_ip4=x=>x;c.ip4_addr_get_u32=x=>x.ip;
 c.memcpy=(target,value)=>{c.active.hostname=value;};c.strcmp=(a,b)=>a===b?0:1;
 c.tcpip_try_callback=(fn,slot)=>{c.active=slot;if(mode==='submit_busy')return -1;c.callback=fn;return 0;};
 c.dns_gethostbyname_addrtype=()=>c.error;c.LWIP_DNS_ADDRTYPE_IPV4=0;
 c.vTaskDelay=n=>{assert(n>=1);c.t+=n*10;c.ticks++;if(c.t===10){if(mode==='cached'){c.completeDns(c.active,{v4:true,ip:7},0);}else if(mode==='resolver_busy')c.completeDns(c.active,null,-1);else if(mode==='fail')c.completeDns(c.active,null,-6);}if(mode==='success'&&c.t===100)c.completeDns(c.active,{v4:true,ip:7},0);if(mode==='cancel'&&c.t===100)c.alive=false;};
 vm.createContext(c);
 let complete=adapt(body(worker,'void completeDns(')).replace('slot.address=*address','slot.address=address');vm.runInContext(`function completeDns(slot,address,error){${complete}}`,c);
 let resolve=adapt(body(worker,'const char* resolve(')).replace('DnsSlot* slot=nullptr;','let slot=null;').replace('for(auto& candidate:dnsSlots)','for(const candidate of dnsSlots)').replace('slot=&candidate','slot=candidate').replace('*slot=DnsSlot{};','Object.assign(slot,{used:false,done:false,abandoned:false,error:0,address:{}});active=slot;').replace(/nullptr/g,'null').replace(/&found/g,'found').replace(/IPAddress\(/g,'IPAddress(');
 vm.runInContext(`function resolve(cmd,hostname,address,result){${resolve}}`,c);c.submitDns=()=>{};
 c.run=()=>{const result={counted:true};const reason=c.resolve({id:1,epoch:1,started:0},'broker.invalid',{fromString:()=>false},result);return {reason,result};};return c;
}
for(const [mode,reason,counted] of [['cached','ok',true],['success','ok',true],['fail','dns_failed',true],['submit_busy','dns_submit_busy',false],['resolver_busy','dns_resolver_busy',false],['timeout','dns_timeout',true],['cancel','cancelled',true]])test('actual DNS '+mode,()=>{
 const c=dnsContext(mode),r=c.run();assert.equal(r.reason,reason);assert.equal(r.result.counted,counted);if(mode!=='submit_busy')assert(c.ticks>0);
 if(['timeout','cancel'].includes(mode)){assert(c.active.used);assert(c.active.abandoned);c.t=60000;assert(c.active.used);c.completeDns(c.active,{v4:true,ip:9},0);assert.equal(c.active.used,false);assert.equal(c.alive,mode!=='cancel');}
 else assert.equal(c.dnsSlots.filter(s=>s.used).length,0);
});
test('two retained DNS tombstones defer without a third allocation or failure debit',()=>{
 const c=dnsContext('timeout');for(const slot of c.dnsSlots)slot.used=true;const r=c.run();assert.equal(r.reason,'dns_slots_busy');assert.equal(r.result.counted,false);assert.equal(c.dnsSlots.length,2);
});
// Real polling code, with scripted availability. Never reset the absolute operation clock.
function packetContext(arrivals,deadline=5000){const c={t:0,ticks:0,stops:0,bytes:arrivals.slice(),socketTimeout:5,bufferSize:512,buffer:[],stream:null,_state:0,packetDrops:0,MQTTPUBLISH:48,MQTTQOS1:2,MQTT_DISCONNECTED:-1};
 c.millis=()=>c.t;c.vTaskDelay=n=>{assert(n>=1);c.t+=n;c.ticks++;};c.operationAllowed=()=>{if(c.t<deadline && (c.cancelAt===undefined || c.t<c.cancelAt))return true;c.stops++;return false;};
 c._client={available:()=>c.bytes.length && c.bytes[0].at<=c.t,read:()=>c.bytes.shift().value,stop:()=>c.stops++};
 let rb=body(pub,'boolean OwnedPubSubClient::readByte(uint8_t * result)').replace(/this->/g,'').replace(/stream->/g,'stream.').replace(/_client->/g,'_client.').replace(/\buint32_t /g,'let ').replace(/\bint value/g,'let value').replace(/\(int32_t\)/g,'').replace(/(\d+)UL/g,'$1').replace('*result = static_cast<uint8_t>(value);','result.value=value;');
 vm.createContext(c);vm.runInContext(`function byte(result){${rb}}`,c);
 c.readByte=(target,index)=>{const out={};if(!c.byte(out))return false;if(index){target[index.value++]=out.value;}else target.value=out.value;return true;};
 let rp=body(pub,'uint32_t OwnedPubSubClient::readPacket(').replace(/this->/g,'').replace(/stream->/g,'stream.').replace(/_client->/g,'_client.').replace(/\bconst uint32_t /g,'const ').replace(/\b(?:uint32_t|uint16_t|uint8_t|bool) /g,'let ').replace('let len = 0;','let len = {value:0};').replace('let digit = 0;','let digit = {value:0};').replace(/\breadByte\(buffer, &len\)/g,'readByte(buffer,len)').replace(/readByte\(&digit\)/g,'readByte(digit)').replace(/\*lengthLength/g,'lengthLength.value');
 rp=rp.replace(/\blen\b/g,'len.value').replace('let len.value = {value:0}','let len = {value:0}').replaceAll('readByte(buffer,len.value)','readByte(buffer,len)');
 rp=rp.replace(/\bdigit\b/g,'digit.value').replace('let digit.value = {value:0}','let digit = {value:0}').replaceAll('readByte(digit.value)','readByte(digit)');
 vm.runInContext(`function packet(lengthLength){${rp}}`,c);return c;
}
test('readByte yields whole ticks and stops missing data at absolute 5s',()=>{const c=packetContext([]);assert.equal(c.byte({}),false);assert(c.ticks>=500);assert.equal(c.t,5000);});
test('delayed CONNACK wait yields IDLE time and honors cancellation deadline',()=>{
 const c=packetContext([{at:1500,value:32}]);
 const wait=body(pub,'while (!_client->available())').replace(/_client->/g,'_client.').replace(/this->/g,'').replace(/stream->/g,'stream.').replace(/unsigned long /g,'let ').replace(/\(int32_t\)/g,'').replace(/(\d+)UL/g,'$1');
 c.MQTT_CONNECTION_TIMEOUT=-4;c.lastInActivity=0;
 vm.runInContext(`function wait(){while(!_client.available()){${wait}}return true;}`,c);
 assert.equal(c.wait(),true);assert.equal(c.t,1500);assert.equal(c.ticks,1500);
 const d=packetContext([],1200);d.MQTT_CONNECTION_TIMEOUT=-4;d.lastInActivity=0;vm.runInContext(`function wait(){while(!_client.available()){${wait}}return true;}`,d);assert.equal(d.wait(),false);assert.equal(d.t,1200);
});
test('partial CONNACK trickle cannot extend the absolute packet deadline',()=>{
 const c=packetContext([{at:0,value:32},{at:2000,value:2},{at:4000,value:0},{at:6000,value:0}]);assert.equal(c.packet({}),0);assert.equal(c.t,5000);assert(c.ticks>=500);
});
test('buffered CONNACK consumes exact four bytes without per-byte delays',()=>{const c=packetContext([32,2,0,0].map(value=>({at:0,value})));assert.equal(c.packet({}),4);assert.equal(c.ticks,0);assert.deepEqual(Array.from(c.buffer),[32,2,0,0]);});
test('hard-cap and malformed PUBLISH close before unbounded discard',()=>{
 for(const data of [[48,129,128,1],[48,128,128,128,128],[48,1,0],[48,2,0,5],[50,2,0,0]]){const c=packetContext(data.map(value=>({at:0,value})));assert.equal(c.packet({}),0);assert(c.stops>=1);assert(c.packetDrops>=1);assert(c.t<5000);}
});
function publishBytes(length){let n=length,encoded=[];do{let b=n%128;n=Math.floor(n/128);encoded.push(b|(n?128:0));}while(n);return [48,...encoded,0,1,97,...Array(length-3).fill(120)];}
for(const length of [600,16384])test('oversized '+length+'-byte body is counted, drained, and keeps next packet aligned',()=>{
 const c=packetContext([...publishBytes(length),32,2,0,0].map(value=>({at:0,value})));
 assert.equal(c.packet({}),0);assert.equal(c.packetDrops,1);assert.equal(c.stops,0);assert(c.t<5000);assert(c.buffer.length<=512);
 assert.equal(c.packet({}),4);assert.deepEqual(Array.from(c.buffer).slice(0,4),[32,2,0,0]);assert.equal(c.bytes.length,0);assert.equal(c.stops,0);
});
test('oversized discard retains absolute deadline under trickle and cancellation',()=>{
 for(const deadline of [5000,1200]){const data=publishBytes(600).map((value,i)=>({at:i<8?0:6000,value}));const c=packetContext(data);if(deadline<5000)c.cancelAt=deadline;
 assert.equal(c.packet({}),0);assert.equal(c.packetDrops,1);assert.equal(c.t,deadline);assert(c.stops>0);assert(c.bytes.length>0);}
});
test('buffered normal body yields once per 64 bytes, not once per byte',()=>{
 const c=packetContext(publishBytes(500).map(value=>({at:0,value})));assert.equal(c.packet({}),503);assert.equal(c.ticks,7);assert.equal(c.packetDrops,0);assert.equal(c.stops,0);
});
test('malformed oversized topic is counted once and closes',()=>{
 const c=packetContext([48,216,4,255,255].map(value=>({at:0,value})));assert.equal(c.packet({}),0);assert.equal(c.packetDrops,1);assert.equal(c.stops,1);
});

test('all network polling loop sites keep a tick delay, no bare yield',()=>{
 assert(!/\byield\s*\(/.test(strip(pub+worker)));
 for(const sig of ['while (!_client->available())','while(!_client->available())','for (uint32_t i = start;','while((bytesRemaining > 0)'])assert(body(pub,sig).includes('vTaskDelay(1)'));
 assert(!body(pub,'uint32_t OwnedPubSubClient::readPacket(').includes('do {\n        vTaskDelay(1);')); // Four-byte length parser delegates empty waits to readByte.
 for(const sig of ['const char* resolve(','bool acquireLease('])assert(body(worker,sig).includes('for(;;) {\n    vTaskDelay(1);'));
});
test('idle and ONLINE turns wait 10ms',()=>{assert(body(worker,'void worker(').includes('for(;;) {\n    vTaskDelay(pdMS_TO_TICKS(10));'));});

test('no direct MQTT access outside owner; facade never reconnects or sends pre-TLS',()=>{
 function files(d){return fs.readdirSync(d,{withFileTypes:true}).flatMap(e=>e.isDirectory()?files(path.join(d,e.name)):[path.join(d,e.name)]);}
 for(const f of ['companion.ino',...files('src')].filter(f=>/\.(?:cpp|h|ino)$/.test(f) && !f.replace(/\\/g,'/').startsWith('src/net/mqtt_client/') && !f.endsWith('net_worker.cpp')))assert(!/\bmqttClient\s*[.>-]|\b(?:Owned)?PubSubClient\s+\w+/.test(strip(read(f))),f);
 const facade=body(worker,'class ConnectedTransport');assert(facade.includes('int connect(IPAddress,uint16_t) override { return 0; }'));assert(facade.includes('int connect(const char*,uint16_t) override { return 0; }'));assert(!facade.includes('transport->connect('));
 assert(worker.indexOf('secure.startTLS()')<worker.indexOf('facade.ready=true'));
 assert(worker.includes('secure.connect(address,port,host,config.caCert,nullptr,nullptr)'));assert(!worker.includes('setInsecure'));
});
test('callbacks, calibration and UI are main-only; dispatch bounded and publications report completion',()=>{
 assert(!/\b(?:lv_\w+|calib\w+|diagnosticsProbe\w*)\s*\(|\b(?:USBSerial|Preferences|diagnet::)/.test(strip(worker)));
 assert(!worker.includes('config.mqttCallback('));assert(net.includes('cfg.mqttCallback('));assert(net.includes('i<2 && esp_timer_get_time()-dispatchAt<2000'));
 assert(body(net,'bool netPublish(').includes('mqttowner::publish('));assert(!body(net,'bool netPublish(').includes('diagnet::publish'));
 assert(net.includes('completion.category,completion.trigger,completion.accepted'));
 assert(net.includes('diag::recordAt(completion.when,"MQTT_PUBLISH",fields,false)'));
});
test('lease covers media and pending handover and both retrieval entry callers',()=>{
 assert(net.includes('!imageFetcherHasPendingDisplay() && !logRetrievalActive()'));
 for(const file of ['src/image/image_fetcher.cpp','src/video/video_stream.cpp','src/diagnostics/diagnostics_retrieval.cpp'])assert(read(file).includes('netMqttLeaseHeld()'));
 assert(read('src/diagnostics/diagnostics_retrieval_ui.cpp').includes('Reconnecting. Try again.'));
 assert(worker.indexOf('resolve(cmd,host,address,result)')<worker.indexOf('acquireLease(cmd,result)'));
 assert(worker.indexOf('acquireLease(cmd,result)')<worker.indexOf('secure.connect('));
});
test('bounded records preserve worker completion clock and cannot include secrets',()=>{
 const format=net.match(/snprintf\(fields,sizeof\(fields\),"([^"]+)"/)[1];
 const worst=format.replace(/%llu/g,'18446744073709551615').replace(/%lu|%u/g,'4294967295').replace(/%d/g,'-2147483648').replace(/%s/g,'dns_resolver_busy');
 assert(worst.length<456,worst.length);assert(net.includes('diag::recordAt(result.when,"MQTT_CONNECT_END",fields,true)'));
 assert(worker.includes('result.when=diag::stamp()'));
 for(const field of ['password','hostname=','ssid=','payload=','certificate='])assert(!format.includes(field));
});

function queueContext(){const c=context();Object.assign(c,{WIRE_BYTES:512,operationEpoch:1,config:{motionTopic:'motion',imuTopic:'imu',topics:{image:'image',power:'power',energy:'energy'}},queues:{tx:Array.from({length:4},()=>({})),rx:Array.from({length:4},()=>({})),sent:Array.from({length:4},()=>({}))},txCount:0,txHead:0,rxCount:0,rxHead:0,sentCount:0,sentHead:0});
 Object.assign(c.status,{connected:true,txDrops:0,txOversize:0,rxDrops:0});
 c.strlen=s=>s.length;c.strnlen=(s,n)=>Math.min(s.length,n);c.strcmp=(a,b)=>a===b?0:1;
 let publish=adapt(body(worker,'bool publish(')).replace(/const char\* /g,'const ').replace(/const size_t /g,'const ').replace('Tx& message=queues->tx','const message=queues.tx');
 publish=publish.replace('memcpy(message.payload,payload,size+1);','message.payload=payload;').replace('strcpy(message.category,category);','message.category=category;').replace('strcpy(message.trigger,trigger);','message.trigger=trigger;');
 vm.runInContext(`function publish(topic,payload,category,trigger){${publish}}`,c);
 let receive=adapt(body(worker,'void receive(')).replace(/\bint /g,'let ').replace('const char* names[]={','const names=[').replace('config.topics.energy};','config.topics.energy];').replace('Rx& item=queues->rx','const item=queues.rx').replace('memcpy(item.payload,payload,size); item.payload[size]=0;','item.payload=payload.slice(0,size);');
 vm.runInContext(`function receive(topic,payload,size){${receive}}`,c);
 let take=adapt(body(worker,'bool takeRx(')).replace('message=queues->rx[rxHead];','Object.assign(message,queues.rx[rxHead]);');
 vm.runInContext(`function takeRx(message){${take}}`,c);return c;
}
test('TX full rejects newest, payload cap honors 512-byte wire limit, acceptance is only enqueue',()=>{
 const c=queueContext();for(let i=0;i<4;i++)assert.equal(c.publish(0,'one','motion','periodic'),true);
 assert.equal(c.publish(0,'fifth','motion','periodic'),false);assert.equal(c.status.txDrops,1);assert.equal(c.txCount,4);assert.equal(c.queues.tx[0].payload,'one');
 c.txCount=0;assert.equal(c.publish(0,'x'.repeat(499),'motion','periodic'),true);assert.equal(c.publish(0,'x'.repeat(500),'motion','periodic'),false);assert.equal(c.status.txOversize,1);
 c.status.connected=false;assert.equal(c.publish(0,'one','motion','periodic'),false);
 assert.equal(c.sentCount,0); // No transport acceptance invented by enqueue.
});
test('RX full/unknown/oversize drop, stale epoch never dispatches callback',()=>{
 const c=queueContext();for(let i=0;i<4;i++)c.receive('image','yes',3);assert.equal(c.rxCount,4);
 c.receive('image','fifth',5);c.receive('unknown','x',1);c.receive('image','x'.repeat(513),513);assert.equal(c.status.rxDrops,3);
 const first={};assert.equal(c.takeRx(first),true);assert.equal(first.payload,'yes');assert.equal(first.epoch,1);
 c.status.epoch=2;assert.equal(c.takeRx({}),false);c.receive('image','stale',5);assert.equal(c.rxCount,2);assert.equal(c.status.rxDrops,5);
});
test('late lease grants after 100ms are revoked before TCP; denials never debit failure budget',()=>{
 for(const mode of ['grant','deny','late','cancel']){const c=context();c.status.phase='Dns';let ticks=0;
 c.vTaskDelay=n=>{assert(n>=1);c.t+=10;ticks++;if((mode==='grant'||mode==='deny')&&c.t===10)c.arbitrate(mode==='grant');if(mode==='late'&&c.t===100)c.arbitrate(true);if(mode==='cancel'&&c.t===20)c.invalidateLocked(false);};
 vm.runInContext(`function acquireLease(cmd,result){${adapt(body(worker,'bool acquireLease('))}}`,c);
 const r={counted:true};const ok=c.acquireLease({epoch:1},r);assert(ticks>0);assert.equal(ok,mode==='grant');assert.equal(c.status.lease,mode==='grant');
 if(!ok){assert.equal(r.counted,false);assert.equal(c.status.requestedLease,false);c.arbitrate(true);assert.equal(c.status.lease,false);}
 }
});
test('memory gate refuses before TCP setup and gives an uncounted resource result',()=>{
 const s=body(worker,'void worker(');const gate=s.indexOf('heap_caps_get_largest_free_block(');assert(gate>=0);assert(gate<s.indexOf('secure.connect('));
 const check=s.slice(gate,s.indexOf('} else if(admitPhase',gate));assert(check.includes('result.reason="memory_refused"; result.counted=false;'));
});
test('failure budget counts only genuine real failures and retry time follows owner completion',()=>{
 let failure=body(net,'else if(result.counted && !result.test)');
 failure=failure.replace(/diagnet::event\([^;]+;/g,'recorded++;');
 const run=new Function('result','everConnected','failureCount','MAX_INITIAL_FAILURES',`let giveUp=false,recorded=0;if(result.counted && !result.test){${failure}}return {giveUp,failureCount,recorded};`);
 for(const result of [{counted:false,test:false},{counted:true,test:true}])assert.equal(run(result,false,4,5).giveUp,false);
 assert.equal(run({counted:true,test:false},false,4,5).giveUp,true);assert.equal(run({counted:true,test:false},true,4,5).giveUp,false);
 assert(net.indexOf('lastMqttAttempt = millis();',net.indexOf('mqttowner::takeResult'))>net.indexOf('mqttowner::takeResult'));
 assert(worker.indexOf('phase(Phase::Cleanup); facade.stop(); plain.stop(); secure.stop(); online=false;')<worker.lastIndexOf('finalResult=result; resultReady=true;'));
});
test('setup dispatch services UI/IMU and stops waiting on held faults; power hooks never wait',()=>{
 const sketch=read('companion.ino'),setup=body(sketch,'void initMQTT(');
 assert(setup.includes('while (netMqttBusy())'));assert(setup.includes('runBackgroundTick(); delay(10)'));assert(setup.includes('if (netMqttFaulted()) break;'));
 for(const sig of ['void goToShutdown(','void goToDeepSleep('])assert(body(sketch,sig).includes('netShutdown();'));
 assert(!/stop\(|wait|vTaskDelay|delay\(/.test(body(net,'void netShutdown(')));
});
test('every new unclassified PubSub loop fails the static polling audit',()=>{
 // Fixed encoders/copies are bounded by the checked wire buffer; all other loops
 // perform transport polling and must yield. Adding a loop requires classification.
 const loops=strip(pub).match(/\b(?:while|for)\s*\([^\n]+/g)||[];
 assert.equal(loops.length,12);
 const workerLoops=strip(worker).match(/for\(;;\)/g)||[];assert.equal(workerLoops.length,3);
});
test('repeated GOT_IP preserves live epoch; real down/up still invalidates',()=>{
 const c=context();c.status.phase='Online';c.status.busy=false;c.status.connected=true;
 c.linkEvent(true);assert.equal(c.status.epoch,1);assert.equal(c.status.connected,true);assert.equal(c.clears,0);
 c.linkEvent(false);assert.equal(c.status.epoch,2);assert.equal(c.status.connected,false);
 c.linkEvent(true);assert.equal(c.status.epoch,3);assert.equal(c.clears,2);c.linkEvent(true);assert.equal(c.status.epoch,3);
});
test('disconnected/stopped RX receives and dequeues are counted; empty dequeue is not',()=>{
 for(const flag of ['disconnected','stopped']){const c=queueContext();c.receive('image','yes',3);
 if(flag==='disconnected')c.status.connected=false;else c.status.stop=true;
 c.receive('image','no',2);assert.equal(c.status.rxDrops,1);assert.equal(c.takeRx({}),false);assert.equal(c.status.rxDrops,2);
 assert.equal(c.takeRx({}),false);assert.equal(c.status.rxDrops,2);}
});
test('invalidation counts queued RX once before clearing',()=>{
 const c=queueContext();c.receive('image','yes',3);c.receive('image','yes',3);
 vm.runInContext(`function clearMessagesLocked(){${adapt(body(worker,'void clearMessagesLocked('))}}`,c);
 c.invalidateLocked(false);assert.equal(c.status.rxDrops,2);assert.equal(c.rxCount,0);c.invalidateLocked(false);assert.equal(c.status.rxDrops,2);
});
function samplingContext(){const c={t:0,stackAt:0,heapAt:0,scans:0,heaps:0,status:{busy:false,stackMin:0},min:Math.min};
 c.nowMs=()=>c.t;c.view=()=>({...c.status});c.uxTaskGetStackHighWaterMark=()=>{c.scans++;return 4096;};c.esp_ptr_external_ram=()=>true;c.sample=()=>c.heaps++;
 vm.createContext(c);let s=adapt(body(worker,'void workerSample(')).replace('static let stackAt=0, heapAt=0;','').replace('nullptr','null').replace('&margin','margin');
 vm.runInContext(`function workerSample(phaseBoundary=false){${s}}`,c);return c;
}
test('idle/ONLINE stack scans are limited to 1Hz and heap walks stop',()=>{
 const c=samplingContext();for(c.t=0;c.t<=3000;c.t++)c.workerSample();assert.equal(c.scans,3);assert.equal(c.heaps,0);
 c.workerSample(true);assert.equal(c.scans,4);assert.equal(c.heaps,0);assert.equal(c.status.stackMin,4096);
});
test('busy attempt samples heap at 20ms and stack at phase boundaries plus 1Hz',()=>{
 const c=samplingContext();c.status.busy=true;c.workerSample(true);assert.equal(c.scans,1);assert.equal(c.heaps,1);
 for(c.t=1;c.t<=1000;c.t++)c.workerSample();assert.equal(c.scans,2);assert.equal(c.heaps,51);
 c.workerSample(true);assert.equal(c.scans,3);assert.equal(c.heaps,52);
 assert(body(worker,'void phase(').includes('workerSample(true)'));
 assert(body(net,'void netMainTick(').includes('mqttowner::view().busy && millis()-sampledAt>=20'));
});
test('shared heap sampler refuses idle and ignores completion or new attempt racing its walk',()=>{
 const c={status:{busy:false,id:1,internalMin:999,largestMin:999,dmaMin:999},walks:0,min:Math.min,MALLOC_CAP_INTERNAL:1,MALLOC_CAP_8BIT:2,MALLOC_CAP_DMA:4};
 c.view=()=>({...c.status});c.heap_caps_get_free_size=()=>{c.walks++;return 100;};c.heap_caps_get_largest_free_block=()=>{c.walks++;if(c.race==='end')c.status.busy=false;if(c.race==='new')c.status.id++;return 80;};
 vm.createContext(c);vm.runInContext(`function sample(){${adapt(body(worker,'void sample(')).replace('const auto before','const before')}}`,c);
 c.sample();assert.equal(c.walks,0);c.status.busy=true;c.sample();assert.equal(c.walks,3);assert.equal(c.status.largestMin,80);
 for(const race of ['end','new']){c.status.busy=true;c.status.largestMin=999;c.race=race;c.sample();assert.equal(c.status.largestMin,999);}
});
console.log(`${count} MQTT owner checks passed; source simulations only, no firmware build.`);
