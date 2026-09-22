'use strict';
// Execute actual mode bodies with platform mocks; no C++ compilation or hardware.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('src/diagnostics/diagnostics_retrieval.cpp','utf8');
function body(s,sig){let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let a=++i,d=1;while(d){if(s[i]=='{')d++;if(s[i]=='}')d--;i++;}return s.slice(a,i-1);}
function adapt(s){return s.replace(/Mode::/g,'Mode.').replace(/diagnet::event/g,'event').replace(/diaghttp::/g,'http.').replace(/const bool /g,'const ').replace(/nullptr/g,'null').replace(/if \(const char\* reason = entryRefusal\(\)\) \{/,'const reason=entryRefusal(); if(reason) {');}
const defs=[['entryRefusal','','const char* entryRefusal()'],['enter','','void enter()'],['logRetrievalActive','','bool logRetrievalActive()'],['logRetrievalExit','reason','void logRetrievalExit('],['logRetrievalTouch','','void logRetrievalTouch()'],['logRetrievalTick','','void logRetrievalTick()'],['logRetrievalCommand','command','bool logRetrievalCommand(']];
function context(){const c={Mode:{Off:0,Starting:1,Active:2,Stopping:3},mode:0,IDLE_MS:300000,activityAt:0,stoppingAt:0,releaseWarned:false,RELEASE_WARN_MS:Number(source.match(/RELEASE_WARN_MS = (\d+)/)[1]),linkUp:false,lastReason:'boot',exitReason:'none',t:100,vbusPresent:true,ready:true,closing:false,wifi:true,image:false,live:false,busy:false,pending:false,events:[],reports:[],aborts:0,WL_CONNECTED:1};
c.http={canStart:true,canActivate:true,isStopped:false,err:null,at:0,allowed:true,notices:[],
interfaceAllowed:()=>c.http.allowed,start:()=>{c.http.isStopped=false;return c.http.canStart;},
stop:()=>{c.http.isStopped=true;},activate:()=>c.http.canActivate,
stopped:()=>c.http.isStopped,failure:()=>c.http.err,stage:()=> 'server_stop',
activity:()=>c.http.at,idleExpired:(t,a,l)=>t-Math.max(a,c.http.at)>=l,
notice:x=>c.http.notices.push(x)};
c.nowMs=()=>c.t;c.report=r=>c.reports.push({result:r,reason:c.lastReason,state:c.mode});c.event=(...a)=>c.events.push(a);c.WiFi={status:()=>c.wifi?1:0};c.diagnosticsStorageClosing=()=>c.closing;c.diagnosticsStorageReady=()=>c.ready;c.imageFetcherIsBusy=()=>c.image;c.videoStreamActive=()=>c.live;c.diagnosticsUsbBusy=()=>c.busy;c.imageFetcherHasPendingDisplay=()=>c.pending;c.diagnosticsUsbCommand=s=>{assert.equal(s,'log abort');c.aborts++;};c.strcmp=(a,b)=>a===b?0:1;c.strncmp=(a,b,n)=>a.slice(0,n)===b.slice(0,n)?0:1;vm.createContext(c);vm.runInContext(defs.map(([n,a,s])=>`function ${n}(${a}){${adapt(body(source,s))}}`).join('\n'),c);return c;}
let count=0;function test(n,f){f(context());count++;console.log('PASS '+n);}
for(const [field,value,reason] of [['vbusPresent',false,'usb_power_required'],['closing',true,'logger_closing'],['ready',false,'logger_unavailable'],['wifi',false,'wifi_offline'],['image',true,'image_busy'],['live',true,'live_busy'],['busy',true,'retrieval_busy'],['pending',true,'display_pending']])test('entry refuses '+reason,c=>{c[field]=value;c.logRetrievalCommand('log mode on');assert.equal(c.mode,0);assert.equal(c.lastReason,reason);assert.equal(c.aborts,0);});
test('entry, repeated on, off and repeated off have explicit results',c=>{c.logRetrievalCommand('log mode on');assert.equal(c.mode,1);c.logRetrievalTick();assert.equal(c.mode,2);c.t++;c.logRetrievalCommand('log mode on');assert.equal(c.lastReason,'not_off');assert.equal(c.activityAt,100);c.logRetrievalCommand('log mode off');assert.equal(c.mode,3);assert.ok(c.logRetrievalActive());c.logRetrievalCommand('log mode off');assert.equal(c.reports.at(-1).result,'already_stopping');c.logRetrievalTick();assert.equal(c.mode,0);c.logRetrievalCommand('log mode off');assert.equal(c.reports.at(-1).result,'already_off');});
test('all non-OFF states exclude media and refuse entry',c=>{for(const m of [1,2,3]){c.mode=m;assert.ok(c.logRetrievalActive());assert.equal(c.entryRefusal(),'not_off');}});
test('STOPPING retains exclusion until writer release and posts abort only once',c=>{c.enter();c.logRetrievalTick();c.busy=true;c.logRetrievalExit('usb_command');c.logRetrievalTick();c.logRetrievalExit('other');assert.equal(c.aborts,1);assert.equal(c.mode,3);c.enter();c.logRetrievalTick();assert.equal(c.lastReason,'not_off');c.busy=false;c.logRetrievalTick();assert.equal(c.mode,0);assert.equal(c.lastReason,'usb_command');});
test('USB loss exits immediately without waiting for an active reader',c=>{c.enter();c.logRetrievalTick();c.busy=true;c.vbusPresent=false;c.logRetrievalTick();assert.equal(c.mode,3);assert.equal(c.aborts,1);assert.equal(c.exitReason,'usb_power_lost');});
test('logger close and failure exit',c=>{c.enter();c.logRetrievalTick();c.closing=true;c.logRetrievalTick();assert.equal(c.mode,0);assert.equal(c.lastReason,'logger_closing');c.closing=false;c.enter();c.logRetrievalTick();c.ready=false;c.logRetrievalTick();assert.equal(c.lastReason,'logger_unavailable');});
test('link loss/recovery retains ACTIVE and does not reset activity',c=>{c.enter();c.logRetrievalTick();c.t=100000;c.wifi=false;c.logRetrievalTick();assert.equal(c.mode,2);assert.equal(c.linkUp,false);c.wifi=true;c.logRetrievalTick();assert.equal(c.linkUp,true);assert.equal(c.activityAt,100);});
test('idle expires at five minutes even with transfer active',c=>{c.enter();c.logRetrievalTick();c.t=300099;c.logRetrievalTick();assert.equal(c.mode,2);c.busy=true;c.t++;c.logRetrievalTick();assert.equal(c.mode,3);assert.equal(c.exitReason,'idle_timeout');assert.equal(c.aborts,1);});
test('touch resets idle; status, repeated on and progress do not',c=>{c.enter();c.logRetrievalTick();c.t=200000;c.logRetrievalTouch();assert.equal(c.activityAt,200000);c.t=400000;c.logRetrievalCommand('log mode status');c.logRetrievalCommand('log mode on');assert.equal(c.activityAt,200000);c.t=500000;c.logRetrievalTick();assert.equal(c.mode,0);});
test('shutdown exit is nonwaiting, does not print or poll for OFF',c=>{c.enter();c.logRetrievalTick();c.busy=true;c.reports=[];c.logRetrievalExit('shutdown');assert.equal(c.mode,3);assert.equal(c.reports.length,0);assert.doesNotMatch(body(source,'void logRetrievalExit('),/while|delay\(|USBSerial|report\(/);});
test('exact command matching does not steal USB or network commands',c=>{for(const x of ['log status','log get current','on','off','log model'])assert.equal(c.logRetrievalCommand(x),false);c.logRetrievalCommand('log mode bogus');assert.equal(c.reports.at(-1).result,'invalid_command');});
const ino=fs.readFileSync('companion.ino','utf8'),sd=fs.readFileSync('src/diagnostics/sd_diagnostics.cpp','utf8');
test('integration observes power, real touch, loop, and central close',()=>{assert.match(ino,/logRetrievalExit\("usb_power_lost"\)/);assert.match(ino,/logRetrievalTouch\(\);\s*data->state = LV_INDEV_STATE_PR/);assert.equal((ino.match(/logRetrievalTick\(\);/g)||[]).length,1);assert.match(body(ino,'void runBackgroundTick()'),/logRetrievalTick\(\);[^]*lv_timer_handler\(\);/);assert.match(sd,/bool diagnosticsClose[^]*?logRetrievalExit\(deepSleep/);assert.doesNotMatch(source,/httpd_|socket\(|Preferences|screenMem|xTaskCreate|delay\(/);});
test('stuck release escalates once at threshold while retaining exclusion',c=>{
 c.enter();c.logRetrievalTick();c.busy=true;c.logRetrievalExit('usb_command');
 c.t=100+c.RELEASE_WARN_MS-1;c.logRetrievalTick();assert.equal(c.releaseWarned,false);
 c.t++;c.logRetrievalTick();assert.equal(c.releaseWarned,true);assert.equal(c.mode,3);
 assert.equal(c.events.filter(e=>e[0]=='RETRIEVAL_STUCK').length,1);
 assert.equal(c.reports.at(-1).result,'release_timeout');assert.equal(c.aborts,1);
 c.t+=300000;c.logRetrievalTick();c.logRetrievalCommand('log mode off');c.logRetrievalTick();
 assert.equal(c.events.filter(e=>e[0]=='RETRIEVAL_STUCK').length,1);assert.ok(c.logRetrievalActive());
});
test('late release recovers after escalation and a new mode resets warning',c=>{
 c.enter();c.logRetrievalTick();c.busy=true;c.logRetrievalExit('usb_command');c.t+=c.RELEASE_WARN_MS;c.logRetrievalTick();
 c.busy=false;c.logRetrievalTick();assert.equal(c.mode,0);assert.equal(c.lastReason,'usb_command');
 c.enter();c.logRetrievalTick();assert.equal(c.releaseWarned,false);c.busy=true;c.logRetrievalExit('idle_timeout');
 c.t+=c.RELEASE_WARN_MS;c.logRetrievalTick();assert.equal(c.events.filter(e=>e[0]=='RETRIEVAL_STUCK').length,2);
});
test('release at warning deadline completes without a false stuck record',c=>{
 c.enter();c.logRetrievalTick();c.busy=true;c.logRetrievalExit('usb_command');c.t+=c.RELEASE_WARN_MS;c.busy=false;
 c.logRetrievalTick();assert.equal(c.mode,0);assert.equal(c.events.filter(e=>e[0]=='RETRIEVAL_STUCK').length,0);
});

test('STARTING excludes media until worker completion',c=>{c.http.canActivate=false;c.enter();c.logRetrievalTick();assert.equal(c.mode,1);assert.ok(c.logRetrievalActive());c.http.canActivate=true;c.logRetrievalTick();assert.equal(c.mode,2);});
test('cancel during startup cannot activate a late completion',c=>{c.http.canActivate=false;c.enter();c.logRetrievalExit('usb_command');c.http.canActivate=true;c.logRetrievalTick();assert.equal(c.mode,0);assert.equal(c.events.filter(e=>e[1].includes('reason=requested result=ok')).length,0);});
test('worker creation failure rolls back to OFF',c=>{c.http.canStart=false;c.enter();assert.equal(c.mode,0);assert.equal(c.lastReason,'worker_start');});
test('server release is independent of USB release',c=>{c.enter();c.logRetrievalTick();c.logRetrievalExit('usb_command');c.http.isStopped=false;c.logRetrievalTick();assert.equal(c.mode,3);c.http.isStopped=true;c.logRetrievalTick();assert.equal(c.mode,0);});
test('HTTP user activity postpones expiry but late activity cannot revive STOPPING',c=>{c.enter();c.logRetrievalTick();c.t=300101;c.http.at=300000;c.logRetrievalTick();assert.equal(c.mode,2);c.logRetrievalExit('usb_command');c.http.isStopped=false;c.http.at=900000;c.t=900001;c.logRetrievalTick();assert.equal(c.mode,3);});
test('server failure escalates before ten seconds without forced free',c=>{c.enter();c.logRetrievalTick();c.http.stop=()=>{};c.http.err='server_stop';c.logRetrievalTick();assert.equal(c.mode,3);assert.ok(c.releaseWarned);assert.equal(c.http.notices.at(-1),true);c.http.isStopped=true;c.logRetrievalTick();assert.equal(c.mode,0);assert.equal(c.http.notices.at(-1),false);});
test('interface change exits but link loss alone does not',c=>{c.enter();c.logRetrievalTick();c.http.allowed=false;c.logRetrievalTick();assert.equal(c.lastReason,'interface_changed');});
console.log(`${count} retrieval mode checks passed; source simulations only.`);
