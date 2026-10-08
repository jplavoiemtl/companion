'use strict';
// Execute the actual orchestration, cleanup and record bodies with a sticky-flag
// platform mock. This is not compilation or validation of the native TLS engine.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),cp=require('child_process');
const read=p=>fs.readFileSync(p,'utf8').replace(/\r/g,'');
const video=read('src/video/video_stream.cpp'),guard=read('src/image/media_secure_client.h');
const observer=read('src/diagnostics/diagnostics_network.cpp'),header=read('src/diagnostics/diagnostics_network.h');
const image=read('src/image/image_fetcher.cpp'),imageHeader=read('src/image/image_fetcher.h');
const clean=s=>s.replace(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|\/\*[\s\S]*?\*\//g,m=>m.startsWith('//')||m.startsWith('/*')?' ':m);
function body(s,sig){s=clean(s);let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);const a=++i;let d=1;while(d){if(s[i]==='{')d++;if(s[i]==='}')d--;i++;}return s.slice(a,i-1);}
let count=0;function test(name,f){f();count++;console.log('PASS '+name);}
const guardMethod=sig=>body(guard,sig).replace(/\b(client_|tcpOpen_|tlsAttempted_)\b/g,'this.$1');
function context(options={}){
 const c={t:0,epHost:'media.test',epPort:443,remote_server_ca_cert:'test-ca',liveId:19,liveFailure:'none',events:[],trace:[],probes:[],options};
 c.esp_timer_get_time=()=>c.t*1000;c.ESP={getFreeHeap:()=>90000};c.MALLOC_CAP_INTERNAL=1;c.heap_caps_get_largest_free_block=()=>45000;
 c.USBSerial={printf:(...args)=>c.trace.push(['serial',...args])};c.diag={Phase:{LiveTls:7}};c.ProbeWindow={LiveTls:1};
 c.diagnosticsProbeBegin=x=>c.probes.push('begin');c.diagnosticsProbeEnd=x=>c.probes.push('end');
 c.Network={hostByName:(host,address)=>{c.trace.push(['dns',host]);c.t+=options.dnsMs??11;address.value='192.0.2.5';return options.dnsFail?0:1;}};
 c.diagnet={LiveConnectResult:class{constructor(){this.dnsMs=this.tcpMs=this.tlsMs=0;this.valid=0;this.failedPhase='none';this.tlsQueried=false;this.tlsCode=0;}},Span:class{
  constructor(...args){c.trace.push(['span',...args]);}
  endLiveConnect(ok,id,result){c.events.push({ok,id,...result});assert.equal(c.vidClient._stillinPlainStart,false);}
 }};
 class Client {
  constructor(){this._stillinPlainStart=!!options.plain;this.open=!!options.open;this.encrypted=this.open&&!this._stillinPlainStart;this.error=options.staleError??-777;this.queries=0;this.stops=0;this.handshakes=0;}
  connected(){return this.open;}
  stillInPlainStart(){return this._stillinPlainStart;}
  setPlainStart(){this._stillinPlainStart=true;c.trace.push(['plain']);}
  stop(){this.open=false;this.encrypted=false;this.stops++;c.trace.push(['stop']);/* core leaves flag and lastError unchanged */}
  setCACert(ca){this.ca=ca;}
  setConnectionTimeout(ms){this.tcpLimit=ms;}
  setHandshakeTimeout(s){this.tlsLimit=s;}
  connect(...args){c.trace.push(['tcp',...args]);c.t+=options.tcpMs??22;
   if(options.tcpFail){this.error=-123;this.stop();return false;}
   this.open=true;this.error=7; // socket result retained by native IP connect
   if(!this._stillinPlainStart){this.handshakes++;this.encrypted=true;}
   return true;
  }
  startTLS(){c.trace.push(['tls']);this.handshakes++;c.t+=options.tlsMs??33;
   if(options.tlsFail){this.stop();return false;} // sticky flag AND stale error
   this._stillinPlainStart=false;this.encrypted=true;return true;
  }
  lastError(){this.queries++;return this.error;}
  write(){assert(this.open);return this._stillinPlainStart?'plaintext':'tls';}
 }
 Client.prototype.clearPlainStart=new Function(body(guard,'void clearPlainStart(').replace(/_stillinPlainStart/g,'this._stillinPlainStart'));
 c.vidClient=new Client();c.Client=Client;vm.createContext(c);
 vm.runInContext(`class MediaPlainStartGuard {
 constructor(client){this.client_=client;this.tcpOpen_=false;this.tlsAttempted_=false;${guardMethod('explicit MediaPlainStartGuard(')}}
 tcpComplete(ok){${guardMethod('void tcpComplete(')}}
 tlsStarted(){${guardMethod('void tlsStarted(')}}
 dispose(){${guardMethod('~MediaPlainStartGuard(')}}
 }`,c);
 let b=body(video,'static bool ensureConnected() {')
  .replace(/vidClient->/g,'vidClient.').replace(/plainStart\./g,'plainStart.')
  .replace(/diagnet::Span connect\(/,'const connect = new diagnet.Span(')
  .replace(/diag::Phase::/g,'diag.Phase.').replace(/ProbeWindow::/g,'ProbeWindow.')
  .replace('diagnet::LiveConnectResult result;','const result = new diagnet.LiveConnectResult();')
  .replace('IPAddress address;','const address = {};')
  .replace(/const (?:bool|uint32_t) /g,'const ').replace(/uint64_t |bool /g,'let ')
  .replace('char ignored[1];','const ignored = [];').replace('sizeof(ignored)','1')
  .replace(/nullptr/g,'null');
 // JS has no C++ automatic destructors: preserve the exact guard lifetime using
 // finally, while executing the real constructor, methods and destructor above.
 assert(b.includes('MediaPlainStartGuard plainStart(*vidClient);'));
 b=b.replace('MediaPlainStartGuard plainStart(*vidClient);','const plainStart = new MediaPlainStartGuard(vidClient); try {');
 assert(/\n  }\s*connect\.endLiveConnect/.test(b));
 b=b.replace(/\n  }\s*connect\.endLiveConnect/,'\n } finally { plainStart.dispose(); }\n }\n connect.endLiveConnect');
 vm.runInContext(`function run(){${b}}`,c);return c;
}
for(const [name,opts,phase,valid,queries,stops] of [
 ['DNS failure',{dnsFail:true},'dns',1,0,1],['TCP failure',{tcpFail:true},'tcp_setup',3,1,2],
 ['TLS failure',{tlsFail:true},'tls',7,0,2],['success',{},'none',7,0,1]
])test(name+' captures executed phases and safely releases plain-start',()=>{
 const c=context(opts);assert.equal(c.run(),phase==='none');const r=c.events[0];
 assert.equal(r.failedPhase,phase);assert.equal(r.valid,valid);assert.equal(r.dnsMs,11);assert.equal(r.tcpMs,valid&2?22:0);assert.equal(r.tlsMs,valid&4?33:0);
 assert.equal(r.tlsQueried,queries===1);assert.equal(r.tlsCode,queries?-123:0);
 assert.equal(c.vidClient.queries,queries);assert.equal(c.vidClient.stops,stops);assert.equal(c.vidClient._stillinPlainStart,false);
 assert.equal(c.vidClient.open,phase==='none');assert.equal(c.vidClient.encrypted,phase==='none');assert.deepEqual(c.probes,['begin','end']);assert.equal(c.events.length,1);
 assert.equal(c.trace.filter(x=>x[0]==='tcp').length,valid&2?1:0);assert.equal(c.trace.filter(x=>x[0]==='tls').length,valid&4?1:0);
});
for(const fail of ['tcpFail','tlsFail'])test(fail+' followed by still request performs TLS',()=>{
 const options={[fail]:true},c=context(options);assert.equal(c.run(),false);options[fail]=false;
 const before=c.vidClient.handshakes;assert(c.vidClient.connect('still.test',443));
 assert.equal(c.vidClient.handshakes,before+1);assert.equal(c.vidClient.write(),'tls');
});
test('negative control: unguarded native failure leaks plaintext on next borrower',()=>{
 const opts={tcpFail:true},c=context(opts);c.vidClient.setPlainStart();assert(!c.vidClient.connect('live.test',443));opts.tcpFail=false;
 assert(c.vidClient._stillinPlainStart);assert(c.vidClient.connect('still.test',443));assert.equal(c.vidClient.write(),'plaintext');
});
test('injected early exit after TCP closes before clearing the flag',()=>{
 const c=context();vm.runInContext(`function early(){const guard=new MediaPlainStartGuard(vidClient);try{guard.tcpComplete(vidClient.connect('test',443));return;}finally{guard.dispose();}};early();`,c);
 assert.equal(c.vidClient.open,false);assert.equal(c.vidClient._stillinPlainStart,false);assert.equal(c.vidClient.stops,1);assert.equal(c.vidClient.handshakes,0);
});
test('encrypted keep-alive is reused without DNS, stop or logging',()=>{
 const c=context({open:true});assert(c.run());assert.equal(c.trace.length,0);assert.equal(c.events.length,0);assert.equal(c.vidClient.stops,0);
});
test('unexpected plaintext connection is closed and re-handshaken',()=>{
 const c=context({open:true,plain:true});assert(c.run());assert.equal(c.vidClient.stops,1);assert.equal(c.vidClient.handshakes,1);assert.equal(c.vidClient.write(),'tls');
});
test('null borrower returns without creating a client',()=>{const c=context();c.vidClient=null;assert.equal(c.run(),false);assert.equal(c.trace.length,0);});
test('hostname, CA, limits, sequential clocks and no application data before TLS',()=>{
 const c=context({dnsMs:8000,tcpMs:4999,tlsMs:5001});assert(c.run());const tcp=c.trace.find(x=>x[0]==='tcp');
 assert.equal(tcp[1].value,'192.0.2.5');assert.deepEqual(tcp.slice(2),[443,'media.test','test-ca',null,null]);
 assert.equal(c.vidClient.tcpLimit,5000);assert.equal(c.vidClient.tlsLimit,5);assert.equal(c.t,18000);assert.equal(c.events[0].tlsMs,5001);
 assert.equal(c.trace.filter(x=>x[0]==='dns').length,1);assert(c.trace.findIndex(x=>x[0]==='tcp')<c.trace.findIndex(x=>x[0]==='tls'));
 assert(!/\b(?:write|print|printf)\s*\(/.test(body(video,'static bool ensureConnected() {').split('connect.endLiveConnect')[0]));
});
function recordContext(enabled=true){
 const c={done_:false,id_:9,start_:100,kind_:'live_connect',previous_:{phase:3,operation:7},nowMs:()=>166,events:[],blocks:[],crumbs:[]};
 c.event=(...x)=>c.events.push(x);c.diagop={block:(...x)=>c.blocks.push(x)};c.diag={breadcrumb:(...x)=>c.crumbs.push(x)};
 let b=body(observer,'void Span::endLiveConnect(');
 if(!enabled)b=b.replace(/#if DIAG_ENABLED[\s\S]*?#endif/g,'');else b=b.replace(/#if DIAG_ENABLED|#endif/g,'');
 b=b.replace(/const uint64_t |const char\* /g,'const ').replace(/static_cast<[^>]+>\(([^)]+)\)/g,'$1').replace(/unsigned\(([^)]+)\)/g,'Number($1)').replace(/diagop::/g,'diagop.').replace(/diag::/g,'diag.');
 vm.createContext(c);vm.runInContext(`function end(ok,liveId,result){${b}}`,c);return c;
}
test('one NET_END and LIVE_CONNECT, matching captured metrics and breadcrumb restore',()=>{
 const c=recordContext();const r={dnsMs:11,tcpMs:22,tlsMs:33,valid:7,failedPhase:'tls',tlsQueried:false,tlsCode:0};
 c.end(false,19,r);c.end(false,19,r);assert.equal(c.events.length,2);assert.equal(c.events[0][0],'NET_END');assert.equal(c.events[1][0],'LIVE_CONNECT');
 assert.deepEqual(c.events[0].slice(-8),[11,22,33,7,'tls',0,0,'unavailable']);assert.deepEqual(c.events[1].slice(-8),c.events[0].slice(-8));
 assert.deepEqual(c.blocks,[['live_connect',9,100,166]]);assert.deepEqual(c.crumbs,[[false,3,7]]);assert(c.done_);
});
test('only freshly queried TCP/setup failure is marked fresh in both records',()=>{
 const c=recordContext();c.end(false,1,{dnsMs:1,tcpMs:5002,tlsMs:0,valid:3,failedPhase:'tcp_setup',tlsQueried:true,tlsCode:-1});
 assert(c.events.every(e=>e.at(-1)==='fresh'));assert(c.events.every(e=>e.at(-2)===-1));
});
test('disabled diagnostics finishes span without records; cleanup has no DIAG guard',()=>{
 const c=recordContext(false);c.end(false,1,{});assert(c.done_);assert.equal(c.events.length,0);
 assert(!guard.includes('DIAG_ENABLED'));assert(!body(video,'static bool ensureConnected() {').includes('DIAG_ENABLED'));
});
test('maximum-width new record fields fit the 456-byte logger capacity',()=>{
 const c=recordContext();c.end(false,1,{dnsMs:1,tcpMs:1,tlsMs:1,valid:7,failedPhase:'tcp_setup',tlsQueried:true,tlsCode:-2147483648});
 for(const event of c.events){const rendered=event[1].replace(/%llu/g,'18446744073709551615').replace(/%lu|%u/g,'4294967295').replace(/%d/g,'-2147483648').replace(/%s/g,'unavailable');assert(rendered.length<456,rendered.length);}
});
test('only shared-client type changes in image source; no second client or virtual overrides',()=>{
 const old=cp.execFileSync('git',['show','125fc18:src/image/image_fetcher.cpp'],{encoding:'utf8'}).replace(/\r/g,'');
 assert.equal(image.replace('MediaSecureClient httpsClient;','WiFiClientSecure httpsClient;').replace('MediaSecureClient* imageFetcherSecureClient()','WiFiClientSecure* imageFetcherSecureClient()'),old);
 assert(imageHeader.includes('MediaSecureClient* imageFetcherSecureClient();'));assert(video.includes('MediaSecureClient* vidClient = nullptr'));
 const derived=guard.slice(0,guard.indexOf('// Main-task scope'));assert(!/\b(?:new|virtual|override)\b/.test(clean(derived)));assert(derived.includes('_stillinPlainStart = false'));
});
test('generic Span end remains byte-identical and Live duration contract documented',()=>{
 const old=cp.execFileSync('git',['show','125fc18:src/diagnostics/diagnostics_network.cpp'],{encoding:'utf8'});
 assert.equal(body(observer,'void Span::end('),body(old,'void Span::end('));assert(header.includes('uint8_t valid = 0;'));
 assert(video.includes('about 10 s + DNS'));assert(video.includes('no application'));assert(video.includes('startTLS does not update lastError'));
});
console.log(`${count} Live connect checks passed; source simulations only, no firmware build.`);
