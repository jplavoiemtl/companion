 'use strict';
// Execute the real UI bodies with LVGL mocks and the real mode bodies. No board build.
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const source=fs.readFileSync('src/diagnostics/diagnostics_retrieval_ui.cpp','utf8');
const ino=fs.readFileSync('companion.ino','utf8');
const modeHarness=fs.readFileSync('tools/tests/retrieval_mode.test.cjs','utf8').split('let count=0;')[0];
const makeMode=new Function('require',modeHarness+'\nreturn context;')(require);
function body(s,sig){let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let a=++i,d=1;while(d){if(s[i]=='{')d++;if(s[i]=='}')d--;i++;}return s.slice(a,i-1);}
function adapt(s){return s.replace(/RetrievalPhase::/g,'Mode.').replace(/RetrievalOrigin::/g,'RetrievalOrigin.')
 .replace(/diagnet::event/g,'diagnosticEvent').replace(/nullptr/g,'null')
 .replace(/const (?:CalibState|RetrievalEntry|RetrievalPhase|RetrievalView|lv_event_code_t|IPAddress|bool|int) /g,'const ')
 .replace(/const char\* /g,'const ').replace(/lv_obj_t\* /g,'let ')
 .replace(/&ui_Screen1/g,'ui_Screen1').replace(/&lv_font_/g,'lv_font_')
 .replace('char url[32] = "";','let url = "";')
 .replace(/snprintf\(url,sizeof\(url\),[^;]+;/,'url = `http://${ip.join(".")}/`;');}
const defs=[['readable','reason','const char* readable('],['setText','label,text','void setText('],['home','','void home()'],['showNotice','label,reason','void showNotice('],['calibrationBusy','','bool calibrationBusy()'],['requestPanel','','void requestPanel()'],['stopEvent','event','void stopEvent('],['labelAt','parent,y,width,text','lv_obj_t* labelAt('],['makeNotice','parent,width','lv_obj_t* makeNotice('],['logRetrievalUiInit','','void logRetrievalUiInit()'],['logRetrievalUiEntryEvent','event','void logRetrievalUiEntryEvent('],['logRetrievalUiTick','','void logRetrievalUiTick()'],['logRetrievalUiPowerDown','','void logRetrievalUiPowerDown()']];
function context(){const c=makeMode();c.diagnosticEvent=c.event;Object.assign(c,{screen:null,stateLabel:null,urlLabel:null,stopButton:null,calibrationNotice:null,homeNotice:null,armed:false,pressed:false,consumed:false,pressedAt:0,refreshedAt:0,noticeAt:0,visibleNotice:null,calibration:0,uiNow:0,allocated:[],loads:[],mem:[],touches:0,ip:[172,20,10,2],lv_font_montserrat_20:20,lv_font_montserrat_16:16});
 const flags=['LV_EVENT_PRESSED','LV_EVENT_PRESSING','LV_EVENT_RELEASED','LV_EVENT_PRESS_LOST','LV_EVENT_CLICKED','LV_EVENT_SCREEN_LOADED','LV_OBJ_FLAG_HIDDEN','LV_OBJ_FLAG_CLICKABLE','LV_OBJ_FLAG_SCROLLABLE','LV_STATE_DISABLED','LV_ALIGN_TOP_MID','LV_ALIGN_BOTTOM_MID','LV_SCR_LOAD_ANIM_NONE','LV_LABEL_LONG_WRAP','LV_TEXT_ALIGN_CENTER','LV_OPA_COVER','CALIB_GRAVITY_SAMPLING','CALIB_FORWARD_SAMPLING','CALIB_READY_TO_COMPUTE'];flags.forEach(x=>c[x]=x);
 const obj=parent=>{const o={parent,text:'',flags:new Set(),states:new Set(),callbacks:[]};c.allocated.push(o);return o;};
 c.ui_Screen1=obj(null);c.ui_calibrationScreen=obj(null);c.active=c.ui_calibrationScreen;
 c.lv_obj_create=obj;c.lv_label_create=obj;c.lv_btn_create=obj;
 c.lv_label_get_text=o=>o.text;c.lv_label_set_text=(o,t)=>o.text=t;
 c.lv_obj_add_flag=(o,f)=>o.flags.add(f);c.lv_obj_clear_flag=(o,f)=>o.flags.delete(f);
 c.lv_obj_add_state=(o,f)=>o.states.add(f);c.lv_obj_clear_state=(o,f)=>o.states.delete(f);
 c.lv_obj_add_event_cb=(o,fn,event)=>o.callbacks.push({fn,event});
 c.lv_obj_set_width=(o,v)=>o.width=v;c.lv_obj_align=(o,a,x,y)=>o.position={a,x,y};
 for(const n of ['lv_label_set_long_mode','lv_obj_set_style_text_align','lv_obj_set_style_bg_color','lv_obj_set_style_bg_opa','lv_obj_set_style_text_color','lv_obj_set_style_text_font','lv_obj_set_size','lv_obj_center'])c[n]=()=>{};
 c.lv_color_hex=x=>x;c.lv_disp_get_hor_res=()=>448;c.lv_event_get_code=e=>e;
 c.lv_tick_get=()=>c.uiNow;c.lv_tick_elaps=start=>(c.uiNow-start)>>>0;
 c.calibGetState=()=>c.calibration;c.memoryReport=p=>c.mem.push(p);
 c.activity_event_handler=()=>c.touches++;c.screenMemoryEventHandler=()=>{};c.ui_Screen1_screen_init=()=>{};
 c.lv_scr_act=()=>c.active;c.lv_disp_load_scr=o=>{c.active=o;c.loads.push(o);};
 c._ui_screen_change=o=>c.lv_disp_load_scr(o);c.WiFi.localIP=()=>c.ip;
 vm.runInContext(defs.map(([n,a,s])=>`function ${n}(${a}){${adapt(body(source,s))}}`).join('\n'),c);
 c.fire=n=>c.logRetrievalUiEntryEvent(c[n]);
 c.hold=()=>{c.fire('LV_EVENT_PRESSED');c.uiNow+=1000;c.fire('LV_EVENT_PRESSING');};
 c.tick=()=>{c.uiNow+=250;c.logRetrievalUiTick();};return c;}
let count=0;function test(name,fn){fn(context());count++;console.log('PASS '+name);}
test('pre-setup hold cannot enter; release tap retains navigation',c=>{
 c.hold();assert.equal(c.mode,0);assert.equal(c.consumed,false);c.fire('LV_EVENT_RELEASED');c.fire('LV_EVENT_CLICKED');assert.equal(c.active,c.ui_Screen1);
});
test('persistent construction registers activity and temporary-screen memory once',c=>{
 c.logRetrievalUiInit();const total=c.allocated.length;c.logRetrievalUiInit();assert.equal(c.allocated.length,total);
 assert.deepEqual(c.mem,['before','after']);assert.ok(c.armed);
 assert.equal(c.screen.callbacks.filter(x=>x.fn===c.screenMemoryEventHandler).length,1);
 assert.equal(c.screen.callbacks.filter(x=>x.fn===c.activity_event_handler).length,1);
 assert.equal(c.stopButton.callbacks[0].event,c.LV_EVENT_CLICKED);
});
test('short tap navigates exactly once with one diagnostic navigation record',c=>{
 c.logRetrievalUiInit();c.fire('LV_EVENT_PRESSED');c.uiNow=999;c.fire('LV_EVENT_PRESSING');assert.equal(c.mode,0);
 c.fire('LV_EVENT_RELEASED');c.fire('LV_EVENT_CLICKED');assert.equal(c.loads.length,1);assert.equal(c.events.filter(x=>x[0]==='UI_ACTION').length,1);
});
test('one-second hold enters once and consumed release never navigates',c=>{
 c.logRetrievalUiInit();c.hold();const entered=c.activityAt;assert.equal(c.mode,1);assert.equal(c.active,c.screen);
 c.uiNow+=2000;c.fire('LV_EVENT_PRESSING');c.fire('LV_EVENT_RELEASED');c.fire('LV_EVENT_CLICKED');
 assert.equal(c.active,c.screen);assert.equal(c.activityAt,entered);assert.equal(c.events.filter(x=>x[0]==='UI_ACTION').length,0);
 c.active=c.ui_calibrationScreen;c.fire('LV_EVENT_PRESSED');c.fire('LV_EVENT_RELEASED');c.fire('LV_EVENT_CLICKED');assert.equal(c.active,c.ui_Screen1);
});
test('press lost disarms hold and new press resets consumption',c=>{
 c.logRetrievalUiInit();c.fire('LV_EVENT_PRESSED');c.fire('LV_EVENT_PRESS_LOST');c.uiNow+=1200;c.fire('LV_EVENT_PRESSING');assert.equal(c.mode,0);
 c.hold();assert.equal(c.mode,1);c.fire('LV_EVENT_PRESS_LOST');c.fire('LV_EVENT_CLICKED');assert.equal(c.active,c.screen);
});
for(const state of ['CALIB_GRAVITY_SAMPLING','CALIB_FORWARD_SAMPLING','CALIB_READY_TO_COMPUTE'])test('panel refuses '+state+' without changing USB admission',c=>{
 c.logRetrievalUiInit();c.calibration=c[state];c.hold();assert.equal(c.mode,0);assert.equal(c.active,c.ui_calibrationScreen);assert.match(c.calibrationNotice.text,/calibration/);
 c.fire('LV_EVENT_RELEASED');c.fire('LV_EVENT_CLICKED');assert.equal(c.active,c.ui_calibrationScreen);
 c.logRetrievalCommand('log mode on');assert.equal(c.mode,1);
});
test('panel battery refusal leaves screen and mode; transient notice hides',c=>{
 c.logRetrievalUiInit();c.vbusPresent=false;c.hold();assert.equal(c.active,c.ui_calibrationScreen);assert.equal(c.mode,0);assert.match(c.calibrationNotice.text,/USB power/);
 c.uiNow+=3000;c.tick();assert.ok(c.calibrationNotice.flags.has(c.LV_OBJ_FLAG_HIDDEN));
});
test('reopen all non-OFF phases without re-entry, ticking, origin or idle changes',c=>{
 c.logRetrievalUiInit();for(const mode of [1,2,3]){c.mode=mode;c.entryOrigin=0;c.activityAt=88;c.lastReason='keep';c.active=c.ui_calibrationScreen;c.hold();assert.equal(c.mode,mode);assert.equal(c.active,c.screen);assert.equal(c.entryOrigin,0);assert.equal(c.activityAt,88);assert.equal(c.lastReason,'keep');}
});
test('OFF returns only an active retrieval screen, and stops once',c=>{
 c.logRetrievalUiInit();c.hold();c.logRetrievalTick();c.busy=true;c.stopEvent(1);c.stopEvent(1);assert.equal(c.aborts,1);assert.equal(c.exitReason,'panel_stop');c.tick();assert.equal(c.active,c.screen);
 c.busy=false;c.logRetrievalTick();c.tick();assert.equal(c.active,c.ui_Screen1);assert.match(c.homeNotice.text,/closed/);
 c.active=c.ui_calibrationScreen;c.tick();assert.equal(c.active,c.ui_calibrationScreen);
});
test('USB-enter panel-open and panel-enter USB-exit preserve persistent objects',c=>{
 c.logRetrievalUiInit();const total=c.allocated.length;
 for(let i=0;i<3;i++){c.active=c.ui_calibrationScreen;c.logRetrievalCommand('log mode on');c.hold();assert.equal(c.entryOrigin,0);c.logRetrievalCommand('log mode off');c.logRetrievalTick();c.tick();assert.equal(c.active,c.ui_Screen1);}
 c.active=c.ui_calibrationScreen;c.hold();assert.equal(c.entryOrigin,1);c.logRetrievalCommand('log mode off');c.logRetrievalTick();c.tick();assert.equal(c.active,c.ui_Screen1);assert.equal(c.allocated.length,total);
});
test('power-down loads home only from retrieval screen and never changes mode',c=>{
 c.logRetrievalUiPowerDown();assert.equal(c.loads.length,0);c.logRetrievalUiInit();c.hold();c.mode=3;c.logRetrievalUiPowerDown();assert.equal(c.active,c.ui_Screen1);assert.equal(c.mode,3);
 c.active=c.ui_calibrationScreen;const loads=c.loads.length;c.logRetrievalUiPowerDown();assert.equal(c.loads.length,loads);
});
test('render handles starting, link loss, address change and stuck stop without idle refresh',c=>{
 c.logRetrievalUiInit();c.hold();const at=c.activityAt;c.tick();assert.equal(c.urlLabel.text,'');c.logRetrievalTick();c.tick();assert.equal(c.urlLabel.text,'http://172.20.10.2/');
 c.wifi=false;c.logRetrievalTick();c.tick();assert.equal(c.urlLabel.text,'');assert.match(c.stateLabel.text,/disconnected/);
 c.wifi=true;c.ip=[172,20,10,4];c.logRetrievalTick();c.tick();assert.equal(c.urlLabel.text,'http://172.20.10.4/');assert.equal(c.activityAt,at);
 c.mode=3;c.releaseWarned=true;c.tick();assert.match(c.stateLabel.text,/restart/);assert.ok(c.stopButton.states.has(c.LV_STATE_DISABLED));
});
test('integration replaces generated handler, arms after setup, keeps global timing and power order',()=>{
 assert.doesNotMatch(ino,/lv_obj_add_event_cb\(ui_Button6, diagnosticNavigationEvent/);
 assert.match(ino,/lv_obj_remove_event_cb\(ui_Button6, ui_event_Button6\);\s*lv_obj_add_event_cb\(ui_Button6, logRetrievalUiEntryEvent, LV_EVENT_ALL/);
 assert.match(ino,/diagnosticsSetupComplete\(\);\s*logRetrievalUiInit\(\);/);
 for(const fn of ['goToShutdown','goToDeepSleep'])assert.match(body(ino,'void '+fn+'()'),/^\s*logRetrievalUiPowerDown\(\);/);
 assert.match(body(ino,'void runBackgroundTick()'),/logRetrievalTick\(\);[^]*logRetrievalUiTick\(\);[^]*lv_timer_handler\(\);/);
 assert.doesNotMatch(source,/logRetrievalTouch\(|httpd_stop|lv_obj_del|lv_timer_handler\(|Preferences|LONG_PRESS_TIME\s*=/);
 assert.match(body(source,'void memoryReport('),/lv_mem_monitor\(&memory\)/);
 assert.match(body(source,'void logRetrievalUiInit()'),/memoryReport\("before"\)[^]*memoryReport\("after"\)/);
});
console.log(`${count} retrieval UI checks passed; source simulations only.`);
