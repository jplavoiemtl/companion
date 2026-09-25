'use strict';
// Actual media admission bodies with mocked UI/platform; not a firmware build.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),path=require('path');
const read=p=>fs.readFileSync(p,'utf8');
const image=read('src/image/image_fetcher.cpp'),video=read('src/video/video_stream.cpp');
const strip=s=>s.replace(/"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\/\/[^\n]*|\/\*[\s\S]*?\*\//g,m=>m.startsWith('//')||m.startsWith('/*')?' ':m);
function body(s,sig){let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let a=++i,d=1;while(d){if(s[i]=='{')d++;if(s[i]=='}')d--;i++;}return s.slice(a,i-1);}
function adapt(s){return s.replace(/diagnet::event/g,'event').replace(/diagnet::imageNotification/g,'notification').replace(/const unsigned long /g,'const ').replace(/lv_obj_t\* /g,'let ').replace(/lv_disp_t\* /g,'let ').replace(/\bNULL\b/g,'null');}
function context(){const c={blocked:true,pendingEndpoint:null,motionTriggered:false,imageDisplayTimeoutActive:false,requestInProgress:false,screen2TimeoutActive:false,lastImageLoadedTime:0,NOTIFICATION_ECHO_WINDOW_MS:2000,screen:1,ui_previous_screen:null,cfg:{screen1:1,screen2:2,screen3:3,inclinometerScreen:4},events:[],notes:[],begins:0,ends:0,mutations:0,active:false,WL_CONNECTED:1,LV_EVENT_CLICKED:1,LV_DISP_ROT_90:1};c.logRetrievalActive=()=>c.blocked;c.event=(...x)=>c.events.push(x);c.notification=x=>c.notes.push(x);c.videoStreamActive=()=>c.active;c.millis=()=>10000;c.lv_scr_act=()=>c.screen;c.imageBegin=()=>c.begins++;c.imageEnd=()=>c.ends++;c.USBSerial={println:()=>{},printf:()=>{}};c.lv_event_get_code=e=>e;c.cleanupImageRequest=()=>{c.mutations++;};c.lv_disp_load_scr=s=>{c.screen=s;c.mutations++;};c.lv_disp_get_default=()=>1;c.lv_disp_set_rotation=()=>c.mutations++;c.lv_refr_now=()=>c.mutations++;c.WiFi={status:()=>1};(c.netMqttLeaseHeld ??= (()=>false), c.netShowReconnectNotice ??= (()=>{}), vm.createContext(c));
const defs=[['buttonLatest_event_handler','e','void buttonLatest_event_handler('],['prepareForRequest','','static bool prepareForRequest() {'],['requestLatestImage','fromNotification=false','bool requestLatestImage('],['buttonBack_event_handler','e','void buttonBack_event_handler('],['imageFetcherHasPendingDisplay','','bool imageFetcherHasPendingDisplay()']];vm.runInContext(defs.map(([n,a,s])=>`function ${n}(${a}){${adapt(body(image,s))}}`).join('\n'),c);
// Execute the actual Live admission prefix; allowed allocation/rendering is outside this harness.
vm.runInContext(`function videoStreamStart(trigger='button'){${adapt(body(video,'bool videoStreamStart(').split('  diagnosticsProbeBegin(')[0])}return 'admitted';}`,c);return c;}
let count=0;function test(n,f){f(context());count++;console.log('PASS '+n);}
for(const remote of [false,true])test('Latest refusal precedes all state and UI effects remote='+remote,c=>{assert.equal(c.requestLatestImage(remote),false);assert.equal(c.begins,0);assert.equal(c.ends,0);assert.equal(c.pendingEndpoint,null);assert.equal(c.mutations,0);assert.equal(c.motionTriggered,false);assert.equal(c.screen,1);if(remote)assert.equal(c.notes[0],'ignored_download_mode');});
test('Back refusal preserves endpoint, lifecycle and screen',c=>{c.buttonBack_event_handler(1);assert.equal(c.begins,0);assert.equal(c.pendingEndpoint,null);assert.equal(c.mutations,0);assert.equal(c.screen,1);});
for(const trigger of ['button','motion_handover'])test('Live refusal precedes active fast-path and mutations '+trigger,c=>{c.active=true;assert.equal(c.videoStreamStart(trigger),false);assert.equal(c.begins,0);assert.equal(c.mutations,0);assert.match(c.events[0][1],/download_mode/);});
test('late prepare backstop records late and terminates lifecycle without preparation',c=>{assert.equal(c.prepareForRequest(),false);assert.equal(c.ends,1);assert.equal(c.mutations,0);assert.match(c.events[0][1],/path=late/);});
test('allowed Latest after mode exit retains preparation and motion assignment order',c=>{assert.equal(c.requestLatestImage(true),false);c.blocked=false;assert.equal(c.requestLatestImage(true),true);assert.equal(c.begins,1);assert.equal(c.pendingEndpoint,'latest');assert.equal(c.motionTriggered,true);assert.equal(c.requestInProgress,true);assert.equal(c.screen,2);assert.equal(c.notes.at(-1),'accepted');});
test('allowed Back after mode exit prepares once and clears motion',c=>{c.blocked=false;c.motionTriggered=true;c.buttonBack_event_handler(1);assert.equal(c.begins,1);assert.equal(c.pendingEndpoint,'back');assert.equal(c.motionTriggered,false);assert.equal(c.requestInProgress,true);});
test('pending display predicate covers both independent flags',c=>{assert.equal(c.imageFetcherHasPendingDisplay(),false);c.imageDisplayTimeoutActive=true;assert.equal(c.imageFetcherHasPendingDisplay(),true);c.imageDisplayTimeoutActive=false;c.motionTriggered=true;assert.equal(c.imageFetcherHasPendingDisplay(),true);});
test('allowed direct and handover Live reach unchanged admission body',c=>{c.blocked=false;assert.equal(c.videoStreamStart(),'admitted');assert.equal(c.videoStreamStart('motion_handover'),'admitted');});
function files(dir){return fs.readdirSync(dir,{withFileTypes:true}).flatMap(e=>e.isDirectory()?files(path.join(dir,e.name)):[path.join(dir,e.name)]);}
test('enumerate every production prepare/Live call site excluding comments and declarations',()=>{
 const hits=[];for(const f of ['companion.ino',...files('src')].filter(f=>/\.(cpp|ino|c)$/.test(f))){const s=strip(read(f));for(const name of ['prepareForRequest','videoStreamStart'])for(const m of s.matchAll(new RegExp('\\b'+name+'\\s*\\(','g'))){const line=s.slice(s.lastIndexOf('\n',m.index)+1,m.index);if(/\b(?:bool|void)\s*$/.test(line))continue;hits.push([f.replace(/\\/g,'/'),name,s.slice(m.index,m.index+80)]);}}
 assert.equal(hits.length,4);assert.equal(hits.filter(x=>x[1]=='prepareForRequest').length,2);assert(hits.every(x=>x[0]=='src/image/image_fetcher.cpp'));
 for(const sig of ['bool requestLatestImage(','void buttonBack_event_handler(']){const b=body(image,sig);assert(b.indexOf('logRetrievalActive()')<b.indexOf('imageBegin('));assert.match(b,/if \(!prepareForRequest\(\)\) return/);}
 const live=body(video,'bool videoStreamStart(');assert(live.indexOf('logRetrievalActive()')<live.indexOf('if (active)'));assert(live.indexOf('logRetrievalActive()')<live.indexOf('ps_malloc'));
 const ino=strip(read('companion.ino'));assert.match(ino,/requestLatestImage\(true\)/);assert.match(body(image,'void buttonLatest_event_handler('),/requestLatestImage\(\)/);
});
test('image busy and motion handover keep their existing semantics',()=>{
 const busy=body(image,'bool imageFetcherIsBusy()');assert.doesNotMatch(busy,/logRetrieval|imageDisplayTimeoutActive|motionTriggered/);
 const loop=body(image,'void imageFetcherLoop()');assert.match(loop,/imageDisplayTimeoutActive = false;\s*motionTriggered = false;[^]*?videoStreamStart\("motion_handover"\)/);
 assert.match(body(image,'bool requestLatestImage('),/if \(!prepareForRequest\(\)\) return false;[^]*?motionTriggered = fromNotification;/);
});
test('actual motion-handover branch clears pending flags and calls Live after exit',c=>{
 c.blocked=false;c.imageDisplayTimeoutActive=true;c.motionTriggered=true;
 c.returnToPreviousScreen=()=>assert.fail('unexpected handover failure');
 const handover=body(body(image,'void imageFetcherLoop()'),'if (elapsed > MOTION_STILL_TIMEOUT)');
 vm.runInContext(`function handover(){${adapt(handover)}}`,c);c.handover();
 assert.equal(c.motionTriggered,false);assert.equal(c.imageDisplayTimeoutActive,false);
});
test('repeated refused MQTT pushes use actual suppression with no per-push unsuppressed record',c=>{
 const network=read('src/diagnostics/diagnostics_network.cpp');
 const notificationBody=body(network,'void imageNotification(')
  .replace(/^#.*$/gm,'').replace(/static const char\* last = "";/,'')
  .replace(/static uint64_t lastAt = 0;/,'').replace(/static uint32_t suppressed = 0;/,'')
  .replace(/const uint64_t now/g,'const now').replace(/static_cast<unsigned long>\(suppressed\)/g,'suppressed');
 c.t=10000;c.nowMs=()=>c.t;c.strcmp=(a,b)=>a===b?0:1;
 vm.runInContext(`let last='',lastAt=0,suppressed=0,suppressedImage=0;function notification(result){${notificationBody}}`,c);
 for(let i=0;i<20;i++)assert.equal(c.requestLatestImage(true),false);
 assert.equal(c.events.length,1);assert.equal(c.events[0][0],'MQTT_IMAGE');
 assert.equal(vm.runInContext('suppressedImage',c),19);
 c.t=14999;c.requestLatestImage(true);assert.equal(c.events.length,1);
 c.t=15000;c.requestLatestImage(true);assert.equal(c.events.length,2);
 assert.equal(c.events[1][2],'ignored_download_mode');assert.equal(c.events[1][3],20);
 assert.equal(c.events.filter(e=>e[0]=='IMAGE_REFUSED').length,0);assert.equal(c.begins,0);
});
test('Latest button records processed only when admission succeeds',c=>{
 c.buttonLatest_event_handler(1);assert.equal(c.events.filter(e=>e[0]=='UI_ACTION').length,0);
 assert.equal(c.events.filter(e=>e[0]=='IMAGE_REFUSED').length,1);
 c.blocked=false;c.buttonLatest_event_handler(1);
 assert.equal(c.events.filter(e=>e[0]=='UI_ACTION').length,1);assert.equal(c.begins,1);
});
test('MQTT lease refuses every media entry before mutation, then permits after release',c=>{
 c.blocked=false;c.netMqttLeaseHeld=()=>true;
 assert.equal(c.requestLatestImage(true),false);assert.equal(c.notes.at(-1),'ignored_mqtt_reconnecting');
 assert.equal(c.requestLatestImage(false),false);c.buttonBack_event_handler(1);
 assert.equal(c.videoStreamStart('button'),false);assert.equal(c.videoStreamStart('motion_handover'),false);
 assert.equal(c.begins,0);assert.equal(c.mutations,0);
 c.netMqttLeaseHeld=()=>false;assert.equal(c.requestLatestImage(false),true);
});
console.log(`${count} media admission checks passed; source simulations only.`);
