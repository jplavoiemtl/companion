'use strict';
// Source-contract regression checks, not a C++ build or hardware validation.
// Compare network policy calls to the explicitly accepted Stage 1B checkpoint.
const fs = require('fs');
const cp = require('child_process');
const assert = require('assert/strict');
const read = p => fs.readFileSync(p, 'utf8').replace(/\r/g, '');
const baseline = p => cp.execFileSync('git', ['show', `f899e3c:${p}`], {encoding:'utf8'}).replace(/\r/g, '');
const net = read('src/net/net_module.cpp');
const owner = read('src/net/net_worker.cpp');
const sketch = read('companion.ino');
const observer = read('src/diagnostics/diagnostics_network.cpp');
const image = read('src/image/image_fetcher.cpp');
const video = read('src/video/video_stream.cpp');
const clock = read('src/diagnostics/diagnostics_clock.cpp');
let checks = 0;
function test(name, f) { f(); ++checks; console.log('PASS ' + name); }
// Strip comments without damaging string literals, then select calls in source order.
const clean = s => s.replace(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|\/\*[\s\S]*?\*\//g,
  token => token.startsWith('//') || token.startsWith('/*') ? '' : token);
function calls(source, receiver, method) {
 const escaped = receiver.replace(/[.*+?^${}()|[\]\\]/g,'\\$&');
 const pattern = new RegExp(escaped+'\\s*(?:\\.|->)\\s*('+method+')\\s*\\(', 'g');
 const s=clean(source), result=[];
 for (const m of s.matchAll(pattern)) {
  let i=m.index+m[0].length, start=i, depth=1, quote=null;
  while (i<s.length && depth) {
   const c=s[i++];
   if (quote) { if(c==='\\') ++i; else if(c===quote) quote=null; }
   else if(c==='"' || c==="'") quote=c;
   else if(c==='(') ++depth; else if(c===')') --depth;
  }
  result.push(m[1]+'('+s.slice(start,i-1).replace(/\s+/g,'')+')');
 }
 return result;
}
function between(s,a,b) { const start=s.indexOf(a); assert(start>=0,a); const end=s.indexOf(b,start+a.length); assert(end>start,b); return s.slice(start,end); }
test('WiFi scan/begin/disconnect policy calls unchanged from accepted checkpoint',()=>{
 assert.deepEqual(calls(sketch,'WiFi','begin|scanNetworks|scanDelete|disconnect|setSleep|mode'),
                  calls(baseline('companion.ino'),'WiFi','begin|scanNetworks|scanDelete|disconnect|setSleep|mode'));
});
test('MQTT endpoints, credentials, subscriptions and fixed timeouts migrate to sole owner',()=>{
 assert(owner.includes('cmd.connection==1 ? config.server1 : config.server2'));
 assert(owner.includes('cmd.connection==1 ? config.serverPort1 : config.serverPort2'));
 assert(owner.includes('cmd.test ? "192.0.2.1"'));
 assert.deepEqual(calls(owner,'client','connect'),['connect("companion-bench-test")','connect(CLIENT_ID,USERNAME,KEY)']);
 assert.deepEqual(calls(owner,'client','setSocketTimeout'),['setSocketTimeout(MQTT_MS/1000)','setSocketTimeout(SOCKET_MS/1000)','setSocketTimeout(SOCKET_MS/1000)']);
 assert.deepEqual(calls(owner,'secure','setConnectionTimeout|setHandshakeTimeout'),['setConnectionTimeout(5000)','setHandshakeTimeout(TLS_MS/1000)']);
 assert.deepEqual(calls(owner,'plain','setConnectionTimeout'),['setConnectionTimeout(5000)']);
 assert(owner.includes('secure.connect(address,port,host,config.caCert,nullptr,nullptr)'));
 assert(owner.includes('config.topics.image,config.topics.power,config.topics.energy'));
 assert(owner.includes('client.subscribe(topics[i],1)'));
 assert(!net.includes('cfg.mqttClient'));
});
test('retry budgets, pacing and media guard remain at accepted values',()=>{
 for(const pattern of [/MQTT_RECONNECT_INTERVAL\s*=\s*15000/,/MAX_INITIAL_FAILURES\s*=\s*5/,/BENCH_FIRST_ATTEMPT_MS\s*=\s*5000/]) assert(pattern.test(net));
 assert(/WIFI_RETRY_INTERVAL_MS\s*=\s*30000/.test(sketch));
 assert(sketch.includes('if (!imageFetcherIsBusy() && !videoStreamActive()) {\n      netCheckMqtt();'));
 assert(net.indexOf('lastMqttAttempt = millis();',net.indexOf('void netMainTick')) > net.indexOf('mqttowner::takeResult(result)'));
});
test('media hostname TLS and HTTP transport calls are unchanged',()=>{
 assert.deepEqual(calls(video,'vidClient','connect|setCACert|setConnectionTimeout|setHandshakeTimeout|stop'),
                  calls(baseline('src/video/video_stream.cpp'),'vidClient','connect|setCACert|setConnectionTimeout|setHandshakeTimeout|stop'));
 assert.deepEqual(calls(image,'httpClient','begin|GET|end|setTimeout|setConnectTimeout'),
                  calls(baseline('src/image/image_fetcher.cpp'),'httpClient','begin|GET|end|setTimeout|setConnectTimeout'));
});
test('every real/test MQTT attempt is bracketed through worker result adoption',()=>{
 const request=between(net,'void netCheckMqtt(', 'bool netIsMqttConnected');
 assert(request.indexOf('mqttowner::request(')<request.indexOf('"MQTT_CONNECT_BEGIN"'));
 assert(request.includes('execution=worker'));
 const adopt=between(net,'if(mqttowner::takeResult(result))', 'int lostState;');
 assert(adopt.indexOf('"MQTT_CONNECT_END"')<adopt.indexOf('diagnosticsProbeEnd('));
 assert(adopt.indexOf('diagnosticsProbeEnd(')<adopt.indexOf('lastMqttAttempt = millis();'));
 assert(owner.indexOf('client.connect(CLIENT_ID')<owner.indexOf('client.subscribe('));
 assert(owner.indexOf('client.subscribe(')<owner.indexOf('finalResult=result; resultReady=true;'));
 assert(!net.includes('diagnet::Span attempt(')); // Worker latency is not main blocking.
});
test('HTTP and Live error snapshots precede application teardown',()=>{
 const request=between(image,'int httpCode = httpClient.GET();', 'int contentLength =');
 assert(request.indexOf('request.end(') < request.indexOf('httpClient.end()'));
 assert(request.includes('isSecureConnection ? &httpsClient : nullptr'));
 const connect=between(video,'const bool connected = vidClient->connect(', '// The single teardown.');
 assert(connect.indexOf('connect.end(') < connect.indexOf('return false;'));
 assert(owner.indexOf('secure.lastError(')<owner.indexOf('phase(Phase::Cleanup); facade.stop(); plain.stop(); secure.stop(); online=false;'));
 assert(owner.includes('startTLS does not refresh lastError'));
});
test('driver callback has no SD, serial, UI, NVS or driver-query calls',()=>{
 const callback=clean(between(observer,'void wifiEvent(', '\n#endif\n}'));
 assert(!/\b(?:USBSerial|Serial|SD_MMC|Preferences|String|malloc|new|delay|vTaskDelay)\b|\bWiFi\s*\.|\blv_/.test(callback));
 for(const event of ['WIFI_ASSOC','WIFI_DISCONNECT','WIFI_GOT_IP','WIFI_LOST_IP','WIFI_SCAN_DONE']) assert(callback.includes('"'+event+'"'));
});
test('no new task, queue, filesystem path or secret serialization in observer',()=>{
 const s=clean(observer);
 assert(!/\b(?:xTaskCreate\w*|xQueueCreate\w*|fopen|open|malloc|calloc|realloc|new)\s*\(/.test(s));
 assert(!/\b(?:API_TOKEN|PASSWORD|USERNAME|CLIENT_ID|KEY)\b/.test(s));
 assert(s.includes('diag::record(name, fields, !routine)'));
 assert(!s.includes('event("WIFI_ASSOC", "ssid='));
});
test('bounded fields reject truncation and C++ format strings produce quoted TLS text',()=>{
 assert(observer.includes('char fields[456]'));
 assert(observer.includes('size_t(length) >= sizeof(fields)'));
 const tokens=clean(observer).match(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'/g) || [];
 const literals=tokens.filter(t=>t.startsWith('"')).map(t=>JSON.parse(t));
 const formats=literals.filter(s=>s.includes('tls_text='));
 assert.equal(formats.length,2);
 for(const f of formats) {assert(f.includes('tls_text="%s"')); assert(!f.includes('\\'));}
 for(const m of observer.matchAll(/event\("([A-Z_]+)"/g)) assert(m[1].length<24,m[1]);
});
test('nested breadcrumbs restore phase and operation, with appended phase IDs',()=>{
 const header=read('src/diagnostics/diagnostics_internal.h');
 assert(header.includes('TestPanic, TestWatchdog,\n                             WifiSetup'));
 assert(clock.includes('Phase::LiveTls'));
 assert(observer.includes('previous_ = diag::mainBreadcrumb()'));
 assert(observer.includes('static_cast<diag::Phase>(previous_.phase), previous_.operation'));
 assert(observer.includes('Span::~Span() { if (!done_) end(false, -1); }'));
});
test('callback aggregates exclude payloads and use unknown inbound age before first message',()=>{
 assert(sketch.includes('diagnet::inbound(!strcmp(topic, HILO_POWER)'));
 assert(observer.includes('h.mqttInboundKnown = inboundCount != 0'));
 const writer=read('src/diagnostics/sd_diagnostics.cpp');
 assert(writer.includes('s.health.mqttInboundKnown ? static_cast<long long>(up - s.health.mqttInboundAt) : -1LL'));
 assert(writer.includes('"NET_HEALTH", fields'));
 assert(!between(writer,'void writeHealth(', 'bool pop(').includes('WiFi.'));
});
test('repeat suppression preserves fresh disconnect after recovery and cumulative counts',()=>{
 assert(observer.includes('for (auto& slot : disconnectGates) slot.used = false;'));
 assert(observer.includes('now - slot.last < 5000'));
 assert(observer.includes('h.wifiSuppressed = suppressedWifi'));
 assert(observer.includes('h.mqttImageSuppressed = suppressedImage'));
});
test('notification reasons and actual library acceptance results are recorded',()=>{
 for(const reason of ['ignored_live','ignored_echo','ignored_screen','accepted']) assert(image.includes('imageNotification("'+reason+'")'));
 assert(sketch.includes('imageNotification("ignored_payload")'));
 assert(net.includes('accepted=%u ack=unobserved'));
 assert(observer.includes('accepted=%u ack=unobserved'));
});
test('normal flags and build identity are correct; fault hooks remain off',()=>{
 const config=read('src/diagnostics/diagnostics_config.h');
 for(const [name,value] of [['DIAG_ENABLED',1],['DIAG_TEST_HOOKS',0],['DIAG_USB_TEST_FIXTURE',0],['DIAG_WRITER_STACK_PSRAM',1]])
 assert(config.includes('#define '+name+' '+value));
 assert(config.includes('"stage3-context"'));
});

test('actual disconnect policy suppresses alternating reasons and handles profile/recovery boundaries',()=>{
 const body=between(observer,'bool allowDisconnect(int reason, int profile, uint64_t now) {','\n// Main-task-only');
 const translated=body.replace('bool allowDisconnect(int reason, int profile, uint64_t now)', 'function allowDisconnect(reason, profile, now)')
   .replace(/DisconnectGate& /g,'let ').replace(/\bint /g,'let ');
 const gates=Array.from({length:4},()=>({used:false,reason:0,profile:0,last:0}));
 const allow=new Function('disconnectGates',translated+'; return allowDisconnect;')(gates);
 assert.equal(allow(201,1,0),true); assert.equal(allow(36,1,0),true);
 for(const t of [2415,4830]) {assert.equal(allow(201,1,t),false);assert.equal(allow(36,1,t),false);}
 assert.equal(allow(201,1,7245),true);assert.equal(allow(36,1,7245),true);
 assert.equal(allow(201,2,7246),true);
 for(const slot of gates) slot.used=false;
 assert.equal(allow(201,1,7250),true);
 assert.equal(allow(36,1,7250),true);
 for(let reason=1;reason<=12;reason++) assert.equal(allow(reason,1,8000),true);
 assert.equal(gates.length,4); // Unseen causes remain visible with bounded state.
});
test('actual RSSI predicate rejects -128 and nonnegative values',()=>{
 const m=observer.match(/bool validRssi\(int rssi\) \{ return ([^;]+); \}/);assert(m);
 const valid=new Function('rssi','return '+m[1]);
 for(const rssi of [-128,-129,0,1,127]) assert.equal(valid(rssi),false);
 for(const rssi of [-127,-61,-39,-1]) assert.equal(valid(rssi),true);
 assert(observer.includes('if (rawValid) { lastRssi = disconnected.rssi;'));
 assert(observer.includes('h.wifi && validRssi(h.rssi)'));
});

console.log(`${checks} Stage 2 source-contract checks passed; no firmware compiled or hardware accessed.`);