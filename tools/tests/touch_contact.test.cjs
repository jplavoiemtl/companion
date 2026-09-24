 'use strict';
// Actual read callback with I2C/LVGL mocks. No compilation or hardware access.
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const source=fs.readFileSync('companion.ino','utf8');
function body(s,sig){let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let a=++i,d=1;while(d){if(s[i]=='{')d++;if(s[i]=='}')d--;i++;}return s.slice(a,i-1);}
function adapt(s){return s.replace(/FT3168->Arduino_IIC_Touch::Value_Information::/g,'').replace(/->/g,'.').replace(/diagnet::event/g,'event').replace(/const int32_t /g,'const ').replace(/int32_t /g,'let ').replace(/lv_indev_t\* /g,'let ').replace(/uint32_t\(/g,'u32(').replace(/nullptr/g,'null');}
function context(){const c={touchContactActive:false,touchAwaitingRelease:false,touchRecoveryAt:0,touchBadCoordinates:0,touchLastX:0,touchLastY:0,touchLastGoodAt:0,touchCancelReported:false,touchCancelReportAt:0,events:[],consoleLines:[],t:0,pin:1,TP_INT:1,LOW:0,i2c_mutex:{},pdTRUE:1,lock:true,held:0,takes:0,gives:0,reads:[],points:1,x:100,y:60,screenWidth:368,screenHeight:448,waitReleases:0,touches:0,irqDuringRead:false,input:{},LV_INDEV_STATE_REL:0,LV_INDEV_STATE_PR:1,TOUCH_FINGER_NUMBER:'count',TOUCH_COORDINATE_X:'x',TOUCH_COORDINATE_Y:'y'};
 c.millis=()=>c.t;c.u32=n=>n>>>0;c.digitalRead=()=>c.pin;c.pdMS_TO_TICKS=x=>x;
 c.xSemaphoreTakeRecursive=()=>{c.takes++;if(!c.lock)return 0;c.held++;return 1;};
 c.xSemaphoreGiveRecursive=()=>{assert.equal(c.held,1);c.held--;c.gives++;};
 c.FT3168={IIC_Interrupt_Flag:false,IIC_Read_Device_Value:key=>{assert.equal(c.held,1);c.reads.push(key);if(c.irqDuringRead)c.FT3168.IIC_Interrupt_Flag=true;return key==='count'?c.points:c[key];}};
 c.logRetrievalTouch=()=>c.touches++;c.lv_indev_get_act=()=>c.input;c.lv_indev_wait_release=input=>{assert.equal(input,c.input);c.waitReleases++;};c.event=(...args)=>c.events.push(args);c.USBSerial={printf:(...args)=>c.consoleLines.push(args)};
 vm.createContext(c);vm.runInContext('function cancelTouchContact(reason){'+adapt(body(source,'static void cancelTouchContact('))+'}\nfunction read(driver,data){'+adapt(body(source,'void my_touchpad_read('))+'}',c);
 c.sample=()=>{const d={point:{x:0,y:0}};c.read(null,d);assert.equal(c.held,0);return d;};return c;}
let count=0;function test(n,f){f(context());count++;console.log('PASS '+n);}
test('idle performs no I2C, IRQ or low pin starts verified contact',c=>{
 assert.equal(c.sample().state,0);assert.equal(c.reads.length,0);c.pin=0;assert.equal(c.sample().state,1);assert.equal(c.touches,1);assert.equal(c.FT3168.IIC_Interrupt_Flag,false);
});
test('stationary contact survives a full second without any new interrupt; zero releases',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;assert.equal(c.sample().state,1);
 for(let t=10;t<=1200;t+=10){c.t=t;assert.equal(c.FT3168.IIC_Interrupt_Flag,false);assert.equal(c.sample().state,1);}
 c.points=0;assert.equal(c.sample().state,0);const reads=c.reads.length;c.sample();assert.equal(c.reads.length,reads);assert.equal(c.waitReleases,0);
});
test('coordinate movement and two-contact reports retain first-finger coordinates',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.points=2;c.sample();c.x=140;c.y=77;const d=c.sample();assert.equal(d.point.x,140);assert.equal(d.point.y,77);
});
test('zero-finger interrupt does not read coordinates or refresh activity',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.points=0;assert.equal(c.sample().state,0);assert.deepEqual(c.reads,['count']);assert.equal(c.touches,0);
});
test('new IRQ during I2C remains pending',c=>{c.FT3168.IIC_Interrupt_Flag=true;c.irqDuringRead=true;c.sample();assert.equal(c.FT3168.IIC_Interrupt_Flag,true);});
for(const failure of ['count','coordinate','mutex'])test('uncertain '+failure+' cancels without click and requires verified lift',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.sample();const touches=c.touches;
 if(failure==='count')c.points=-1;if(failure==='coordinate'){c.x=-1;c.touchBadCoordinates=5;}if(failure==='mutex')c.lock=false;
 assert.equal(c.sample().state,0);assert.equal(c.waitReleases,1);assert.equal(c.touches,touches);assert.ok(c.touchAwaitingRelease);
 c.points=1;c.x=100;c.lock=true;const attempts=c.takes;
 c.t=49;assert.equal(c.sample().state,0);assert.equal(c.takes,attempts);
 c.t=50;assert.equal(c.sample().state,0);assert.equal(c.touchAwaitingRelease,true);assert.equal(c.touches,touches);
 c.points=0;c.t=100;assert.equal(c.sample().state,0);assert.equal(c.touchAwaitingRelease,false);
 c.points=1;c.FT3168.IIC_Interrupt_Flag=true;assert.equal(c.sample().state,1);
});
test('bounds and invalid contact count never count as real touch',c=>{
 for(const [field,value] of [['x',368],['y',448],['y',-1],['points',3]]){c.touchAwaitingRelease=false;c.points=1;c.x=1;c.y=1;c[field]=value;c.FT3168.IIC_Interrupt_Flag=true;assert.equal(c.sample().state,0);assert.equal(c.touches,0);}
});
test('recovery throttles a stuck interrupt and handles tick wrap',c=>{
 c.t=0xfffffff0;c.pin=0;c.points=-1;c.sample();const takes=c.takes;
 c.t=20;c.sample();assert.equal(c.takes,takes);c.t=40;c.sample();assert.equal(c.takes,takes+1);assert.equal(c.touches,0);
});
test('valid read stream drives the actual UI hold then consumed release without navigating',c=>{
 const uiHarness=fs.readFileSync('tools/tests/retrieval_ui.test.cjs','utf8').split('\nlet count=0;')[0];
 const ui=new Function('require',uiHarness+'\nreturn context();')(require);ui.logRetrievalUiInit();
 let down=false;function pump(){const d=c.sample();ui.uiNow=c.t;if(d.state===1)ui.fire(down?'LV_EVENT_PRESSING':'LV_EVENT_PRESSED');else if(down){ui.fire('LV_EVENT_RELEASED');ui.fire('LV_EVENT_CLICKED');}down=d.state===1;}
 c.FT3168.IIC_Interrupt_Flag=true;pump();for(let t=10;t<=1100;t+=10){c.t=t;pump();}
 assert.equal(ui.active,ui.screen);assert.equal(ui.mode,1);assert.equal(ui.loads.length,1);
 c.points=0;c.t=1110;pump();assert.equal(ui.active,ui.screen);assert.equal(ui.events.filter(x=>x[0]==='UI_ACTION').length,0);
});
test('ordinary short tap still drives exactly one home navigation',c=>{
 const h=fs.readFileSync('tools/tests/retrieval_ui.test.cjs','utf8').split('\nlet count=0;')[0];const ui=new Function('require',h+'\nreturn context();')(require);ui.logRetrievalUiInit();
 c.FT3168.IIC_Interrupt_Flag=true;assert.equal(c.sample().state,1);ui.fire('LV_EVENT_PRESSED');c.t=100;c.points=0;assert.equal(c.sample().state,0);ui.uiNow=100;ui.fire('LV_EVENT_RELEASED');ui.fire('LV_EVENT_CLICKED');assert.equal(ui.active,ui.ui_Screen1);assert.equal(ui.loads.length,1);
});

test('one bad coordinate retains point then good sample resets the streak',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.sample();const touches=c.touches;c.x=-1;c.t=10;
 let d=c.sample();assert.equal(d.state,1);assert.equal(d.point.x,100);assert.equal(d.point.y,60);assert.equal(c.touches,touches);assert.equal(c.waitReleases,0);
 c.x=102;c.t=20;d=c.sample();assert.equal(d.point.x,102);assert.equal(c.touchBadCoordinates,0);
});
test('five bad coordinate samples bridge, sixth cancels',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.sample();c.x=-1;
 for(let n=1;n<=5;n++){c.t=n*10;assert.equal(c.sample().state,1);assert.equal(c.waitReleases,0);}
 c.t=60;assert.equal(c.sample().state,0);assert.equal(c.waitReleases,1);assert.ok(c.touchAwaitingRelease);
 assert.equal(c.events[0][2],'coordinates');
});
test('stale coordinate age cancels even before five samples',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.sample();c.y=-1;c.t=51;assert.equal(c.sample().state,0);assert.equal(c.waitReleases,1);
});
test('bad first coordinate is ignored without suppression; fresh valid point can start',c=>{
 c.x=-1;c.FT3168.IIC_Interrupt_Flag=true;assert.equal(c.sample().state,0);assert.equal(c.touchAwaitingRelease,false);assert.equal(c.waitReleases,0);
 c.x=20;c.FT3168.IIC_Interrupt_Flag=true;assert.equal(c.sample().state,1);
});
test('verified lift during coordinate grace remains an ordinary release',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.sample();c.x=-1;c.t=10;assert.equal(c.sample().state,1);
 c.points=0;c.t=20;assert.equal(c.sample().state,0);assert.equal(c.waitReleases,0);assert.equal(c.touchBadCoordinates,0);
});
test('count error bypasses coordinate grace and logs are limited to once per five seconds',c=>{
 c.FT3168.IIC_Interrupt_Flag=true;c.sample();c.x=-1;assert.equal(c.sample().state,1);c.points=-1;assert.equal(c.sample().state,0);assert.equal(c.waitReleases,1);
 assert.equal(c.events[0][2],'count');for(let t=50;t<5000;t+=50){c.t=t;c.sample();}assert.equal(c.events.length,1);assert.equal(c.consoleLines.length,1);
 c.t=5000;c.sample();assert.equal(c.events.length,2);assert.equal(c.consoleLines.length,2);
});
test('cancellation requests PRESS_LOST cleanup rather than resetting the input target',()=>{
 const cancel=body(source,'static void cancelTouchContact(');
 assert.match(cancel,/lv_indev_wait_release\(input\)/);assert.doesNotMatch(cancel,/lv_indev_reset\(/);
 assert.match(body(source,'void my_touchpad_read('),/data->state = LV_INDEV_STATE_REL/);
});
console.log(`${count} touch contact checks passed; source simulations only.`);
