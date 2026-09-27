'use strict';
// Execute the real shared updater and status selection with LVGL/network mocks.
const fs=require('fs'),vm=require('vm'),assert=require('node:assert/strict');
const ino=fs.readFileSync('companion.ino','utf8');
function body(sig){let i=ino.indexOf(sig);assert(i>=0,sig);i=ino.indexOf('{',i);const start=++i;let depth=1;while(depth){if(ino[i]==='{')depth++;if(ino[i]==='}')depth--;i++;}return ino.slice(start,i-1);}
function context(){
 const c={wifi:0,mqtt:false,port:9735,WL_CONNECTED:3,LV_PART_MAIN:0,writes:0,events:[],serial:[],prev_wifi_status:-1,prev_mqtt_status:false};
 for(const name of ['ui_labelConnectionStatus','ui_labelConnectionStatusGmeter','ui_labelConnectionStatusInclinometer','calibration','download'])c[name]={text:'',color:0};
 c.WiFi={status:()=>c.wifi};c.netIsMqttConnected=()=>c.mqtt;c.netGetActivePort=()=>c.port;
 c.lv_color_hex=x=>x;c.lv_label_set_text=(o,t)=>{o.text=t;c.writes++;};c.lv_obj_set_style_text_color=(o,color)=>o.color=color;
 c.diagnet={event:(...a)=>c.events.push(a)};c.USBSerial={printf:(...a)=>c.serial.push(a)};
 const helper=body('void setConnectionStatusLabels(').replace('lv_obj_t* labels[] = {','const labels = [').replace('    };','    ];').replace('for (lv_obj_t* label : labels)','for (const label of labels)');
 const update=body('void updateConnectionStatusUI()').replace(/static (?:int|bool) prev_\w+ = [^;]+;/g,'').replace(/\b(?:int|bool|uint16_t) /g,'let ').replace(/diagnet::/g,'diagnet.').replace('const let ','const ');
 vm.createContext(c);vm.runInContext(`function setConnectionStatusLabels(text,color){${helper}} function updateConnectionStatusUI(){${update}}`,c);return c;
}
function labels(c){return [c.ui_labelConnectionStatus,c.ui_labelConnectionStatusGmeter,c.ui_labelConnectionStatusInclinometer];}
function expect(c,text,color){for(const o of labels(c)){assert.equal(o.text,text);assert.equal(o.color,color);}assert.equal(c.calibration.text,'');assert.equal(c.download.text,'');}
let count=0;function test(name,fn){fn(context());count++;console.log('PASS '+name);}
test('offline, WiFi-only and secure MQTT transitions synchronize all three labels',c=>{
 c.updateConnectionStatusUI();expect(c,'Offline',0xff0000);
 c.wifi=3;c.updateConnectionStatusUI();expect(c,'WiFi Connected',0xffb700);
 c.mqtt=true;c.updateConnectionStatusUI();expect(c,'MQTT Remote',0x00ff00);
 c.mqtt=false;c.updateConnectionStatusUI();expect(c,'WiFi Connected',0xffb700);
 c.wifi=0;c.updateConnectionStatusUI();expect(c,'Offline',0xff0000);
 assert.equal(c.events.length,5);assert.equal(c.serial.length,5);
});
test('local and both secure ports preserve original names',c=>{
 for(const [port,text] of [[1883,'MQTT Local'],[8883,'MQTT Remote'],[9735,'MQTT Remote']]){
  c.port=port;c.mqtt=false;c.updateConnectionStatusUI();c.mqtt=true;c.updateConnectionStatusUI();expect(c,text,0x00ff00);
 }
});
test('unchanged state does not redraw or multiply connection records',c=>{
 c.wifi=3;c.mqtt=true;c.updateConnectionStatusUI();const n=c.writes;
 for(let i=0;i<100;i++)c.updateConnectionStatusUI();assert.equal(c.writes,n);assert.equal(c.events.length,1);assert.equal(c.serial.length,1);
 // Hidden screens already have the current text without another connection transition.
 expect(c,'MQTT Remote',0x00ff00);
});
test('startup, retry and power messages reach exactly the same labels',c=>{
 for(const [text,color] of [['Connecting...',0xffffff],['Retry 1/4...',0xffb700],['No WiFi',0xff0000],['No WiFi - Shutdown',0xff0000],['Sleeping...',0xffb700],['Shutdown...',0xff0000]]){c.setConnectionStatusLabels(text,color);expect(c,text,color);}
 assert.equal(c.events.length,0);
 assert(!/lv_label_set_text\(ui_labelConnectionStatus\s*,/.test(ino));
 for(const text of ['Connecting...','No WiFi','No WiFi - Shutdown','Sleeping...','Shutdown...'])assert(ino.includes(`setConnectionStatusLabels("${text}",`));
 assert(ino.includes('setConnectionStatusLabels(statusBuffer, 0xFFB700)'));
});
test('null labels are safe and each call reads current globals',c=>{
 c.ui_labelConnectionStatusGmeter=null;c.setConnectionStatusLabels('Offline',0xff0000);
 c.ui_labelConnectionStatusGmeter={};c.setConnectionStatusLabels('MQTT Remote',0x00ff00);expect(c,'MQTT Remote',0x00ff00);
});
test('SquareLine layout agrees on all three screens and eagerly creates them',()=>{
 for(const [file,name,parent] of [['ui_Screen1.c','ui_labelConnectionStatus','ui_Screen1'],['ui_Screen3.c','ui_labelConnectionStatusGmeter','ui_Screen3'],['ui_InclinometerScreen.c','ui_labelConnectionStatusInclinometer','ui_InclinometerScreen']]){
  const s=fs.readFileSync(file,'utf8');assert(s.includes(`${name} = lv_label_create(${parent})`));
  for(const [property,value] of [['x','8'],['y','-137'],['align','LV_ALIGN_CENTER'],['width','LV_SIZE_CONTENT'],['height','LV_SIZE_CONTENT']])assert(s.includes(`lv_obj_set_${property}(${name}, ${value})`));
  assert(s.includes(`lv_obj_set_style_text_font(${name}, &lv_font_montserrat_20`));
  assert(fs.readFileSync('ui.c','utf8').includes(`${parent}_screen_init();`));
 }
 assert(!fs.readFileSync('ui_calibrationScreen.c','utf8').includes('labelConnectionStatus'));
});
test('exported image assets retain LVGL 8 descriptors',()=>{
 for(const file of ['ui_img_1435680676.c','ui_img_2121104240.c','ui_img_button_back_png.c','ui_img_button_latst_png.c','ui_img_button_new_png.c']){
  const s=fs.readFileSync(file,'utf8');assert(s.includes('const lv_img_dsc_t '));assert(s.includes('LV_IMG_CF_TRUE_COLOR_ALPHA'));assert(!s.includes('LV_COLOR_FORMAT_RGB565A8'));
 }
});
console.log(`${count} connection status checks passed; no firmware compiled.`);
