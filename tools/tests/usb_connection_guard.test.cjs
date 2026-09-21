// Source-level simulation of the actual C++ guards, not a firmware build.
// Only declaration/cast syntax is adapted; mocked time/USB drive each branch.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const usb = fs.readFileSync(path.join(__dirname, '../../src/diagnostics/diagnostics_usb.cpp'), 'utf8');
const reader = fs.readFileSync(path.join(__dirname, '../../src/diagnostics/diagnostics_reader.cpp'), 'utf8');
const header = fs.readFileSync(path.join(__dirname, '../../src/diagnostics/diagnostics_reader.h'), 'utf8');
const source = usb + '\n' + reader + '\n' + header;
function body(signature) {
  const start = source.indexOf(signature);
  assert.ok(start >= 0, signature);
  const open = source.indexOf('{', start);
  let depth = 1, end = open + 1;
  while (depth && end < source.length) {
    if (source[end] === '{') ++depth;
    if (source[end] === '}') --depth;
    ++end;
  }
  return source.slice(open + 1, end - 1)
    .replace(/const (?:bool|uint64_t|size_t|int|auto) /g, 'const ')
    .replace(/^#(?:if DIAG_USB_TEST_FIXTURE|endif).*$/gm, '')
    .replace(/Phase::/g, 'Phase.')
    .replace(/diagreader::(\w+)/g, (_, n) => 'reader_' + n)
    .replace(/if \(const char\* reason = transportStop\(\)\) return reason;/g, 'const reason = transportStop(); if (reason) return reason;')
    .replace(/bool abort =/g, 'const abort =')
    .replace(/port(?:ENTER|EXIT)_CRITICAL\(&(?:usbMux|sessionMux)\);/g, '')
    .replace(/if \(const char\* reason = stopReason\(\)\) \{/g,
      'const reason = stopReason(); if (reason) {')
    .replace(/reinterpret_cast<const uint8_t\*>\(bytes\)/g, 'bytes')
    .replace(/\bint\(/g, 'Number(')
    .replace(/\bnullptr\b/g, 'null');
}
const adapted = `function reader_stopReason(transportStop){${body('const char* stopReason(const char* (*transportStop)()) {')}}
function transportStop(){${body('const char* transportStop() {')}}
function reader_release(generation){${body('bool release(uint64_t generation) {')}}
function testStartDownload(fileDownload){${body('void testStartDownload(bool fileDownload) {')}}
function testDataWaiting(){${body('bool testDataWaiting() {')}}
function release(){${body('void release() {')}}
function testCommand(command){${body('bool diagnosticsUsbCommand(const char* command) {').split('  if (!strcmp(command,"log status"))')[0]} return false;}
function transferConnected(){${body('bool transferConnected() {')}}
function stopReason(){${body('const char* stopReason() {')}}
function sendLine(bytes, transfer=false){${body('int sendLine(const char* bytes, bool transfer = false) {')}}`;
function context() {
  const c = {
    now:1000, connected:true, readings:[], writes:0, free:240, shortWrite:false,
    testSlowArmed:false, testSlowActive:false, testLastDataAt:0, dataLines:0,
    sessionGeneration:1, retainedGeneration:1, invalidated:false, reader:-1, paused:false, reserved:true, buffers:null, entries:null, pendingBytes:0, phase:'Data', Phase:{Idle:'Idle'},
    heap_caps_free:()=>{},
    connectionLost:false, connectionLostAt:0, longestLoss:0, connectionLosses:0,
    lastProgress:1000, startedAt:1000, isCurrent:false, abortRequested:false,
    state:{closing:false, ready:true, queued:0, capacity:16},
    lastLineBytes:0, lastTxFree:-1, lastWriteBytes:-1,
    lastSendCheck:'none', lastSendStop:'none',
    strlen:s=>s.length, strcmp:(a,b)=>a===b ? 0 : 1, errors:[], statusRequested:false,
  };
  for (const name of ['DISCONNECT_MS','STALL_MS','CURRENT_MS','WIRE','TEST_DATA_INTERVAL_MS']) {
    const match=source.match(new RegExp('\\b'+name+' = (\\d+)'));
    assert.ok(match, name); c[name]=Number(match[1]);
  }
  c.milliseconds=()=>c.now;
  c.reader_busy=()=>c.reserved; c.abortPending=()=>c.abortRequested;
  c.queueError=reason=>c.errors.push(reason);
  c.hooks={status:()=>c.state};
  c.USBSerial={
    isConnected:()=>c.readings.length ? c.readings.shift() : c.connected,
    availableForWrite:()=>c.free,
    write:(bytes,length)=>{ ++c.writes; return c.shortWrite ? length-1 : length; },
  };
  vm.createContext(c); vm.runInContext(adapted,c); return c;
}
let checks=0;
function check(name,fn){fn(context()); console.log('PASS',name); ++checks;}
check('healthy line sends once', c=>{
  assert.equal(c.sendLine('abc',true),1); assert.equal(c.writes,1);
});
check('brief loss pauses without writing then recovers', c=>{
  c.connected=false;
  assert.equal(c.stopReason(),null); assert.equal(c.sendLine('abc',true),0);
  c.now+=20; c.connected=true;
  assert.equal(c.sendLine('abc',true),1); assert.equal(c.writes,1);
  assert.equal(c.longestLoss,20); assert.equal(c.connectionLosses,1);
});
check('continuous loss expires at one second', c=>{
  c.connected=false; assert.equal(c.stopReason(),null);
  c.now+=999; assert.equal(c.stopReason(),null);
  c.now+=1; assert.equal(c.stopReason(),'disconnected'); assert.equal(c.writes,0);
});
check('false reading after space check does not send', c=>{
  c.readings=[true,false]; assert.equal(c.sendLine('abc',true),0);
  assert.equal(c.writes,0);
});
check('false reading in final stop check does not send', c=>{
  c.readings=[true,true,false]; assert.equal(c.sendLine('abc',true),0);
  assert.equal(c.writes,0); assert.equal(c.lastSendCheck,'connected_before_write');
});
check('flapping without progress still expires at five seconds', c=>{
  for(let i=0;i<10;++i){
    c.now=1000+i*400; c.connected=false; assert.equal(c.stopReason(),null);
    c.now+=200; c.connected=true; assert.equal(c.stopReason(),null);
  }
  c.now=6000; assert.equal(c.stopReason(),'stalled');
});
check('current deadline remains; archive has no overall deadline', c=>{
  c.now=121000; c.lastProgress=c.now; c.isCurrent=true;
  assert.equal(c.stopReason(),'timeout'); c.isCurrent=false;
  assert.equal(c.stopReason(),null);
});
check('abort and queue protection still apply during brief loss', c=>{
  c.connected=false; c.abortRequested=true; assert.equal(c.stopReason(),'aborted');
  c.abortRequested=false; c.state.queued=8; assert.equal(c.stopReason(),'logger_busy');
  c.state.closing=true; assert.equal(c.stopReason(),'shutdown');
});
check('short write remains a failure, never a whole-line retry', c=>{
  c.shortWrite=true; assert.equal(c.sendLine('abc',true),-1);
  assert.equal(c.writes,1); assert.equal(c.lastSendCheck,'short_write');
});
check('control reply retries transient loss without writing', c=>{
  c.connected=false; assert.equal(c.sendLine('abc'),0); assert.equal(c.writes,0);
});
check('serial on arms only while idle and off cancels both armed and active pacing', c=>{
  assert.equal(c.testCommand('log test slow on'),true);
  assert.deepEqual(c.errors,['busy']);assert.equal(c.testSlowArmed,false);
  c.reserved=false;c.testCommand('log test slow on');
  assert.equal(c.testSlowArmed,true);assert.equal(c.statusRequested,true);
  c.testSlowActive=true;c.reserved=true;c.testCommand('log test slow off');
  assert.equal(c.testSlowArmed,false);assert.equal(c.testSlowActive,false);
  assert.equal(c.testCommand('log test slow maybe'),false);
});
check('slow test defaults off and a list does not consume an armed test', c=>{
  assert.equal(Boolean(c.testDataWaiting()),false);
  c.testSlowArmed=true;c.testStartDownload(false);
  assert.equal(c.testSlowArmed,true);assert.equal(c.testSlowActive,false);
  c.testStartDownload(true);
  assert.equal(c.testSlowArmed,false);assert.equal(c.testSlowActive,true);
});
check('first data line is immediate; later lines wait 100 ms without resetting progress', c=>{
  c.testSlowArmed=true;c.testStartDownload(true);
  assert.equal(Boolean(c.testDataWaiting()),false);
  c.dataLines=1;c.testLastDataAt=c.lastProgress=c.now;
  c.now+=99;assert.equal(Boolean(c.testDataWaiting()),true);
  assert.equal(c.lastProgress,1000);
  c.now+=1;assert.equal(Boolean(c.testDataWaiting()),false);
  c.testSlowActive=false;c.now-=1;
  assert.equal(Boolean(c.testDataWaiting()),false);
});
check('continuously paced current reaches 120 seconds while archive keeps progressing', c=>{
  for (const current of [true,false]) {
    c.now=c.startedAt=c.lastProgress=1000;c.dataLines=0;
    c.isCurrent=current;c.testSlowArmed=true;c.testStartDownload(true);
    let reason=null;
    for (;c.now<=131000;++c.now) {
      reason=c.stopReason();if(reason)break;
      if (!c.testDataWaiting()) {
        ++c.dataLines;c.testLastDataAt=c.lastProgress=c.now;
      }
    }
    if(current) {
      assert.equal(reason,'timeout');assert.equal(c.now-c.startedAt,120000);
      assert.equal(c.now-c.lastProgress,100);assert.equal(c.dataLines,1200);
    } else {assert.equal(reason,null);assert.equal(c.dataLines,1301);}
  }
});
check('pacing never masks connection, stall, queue, abort or shutdown guards', c=>{
  c.testSlowArmed=true;c.testStartDownload(true);c.dataLines=1;c.testLastDataAt=c.now;
  assert.ok(c.testDataWaiting());
  c.state.queued=8;assert.equal(c.stopReason(),'logger_busy');c.state.queued=0;
  c.abortRequested=true;assert.equal(c.stopReason(),'aborted');c.abortRequested=false;
  c.state.closing=true;assert.equal(c.stopReason(),'shutdown');c.state.closing=false;
  c.connected=false;assert.equal(c.stopReason(),null);c.now+=1000;
  assert.equal(c.stopReason(),'disconnected');c.connected=true;c.now=6000;
  assert.equal(c.stopReason(),'stalled');
});
check('release disables active pacing; listing release preserves an armed test', c=>{
  c.testSlowActive=true;c.testSlowArmed=false;c.release();
  assert.equal(c.testSlowActive,false);assert.equal(c.reserved,false);
  c.testSlowArmed=true;c.release();assert.equal(c.testSlowArmed,true);
});
// Ensure the simulated wait is integrated after safety checks, never ahead of them.
const tick=source.slice(source.indexOf('void diagnosticsUsbTick() {'),source.indexOf('void diagnosticsUsbStop() {'));
assert.ok(tick.indexOf('stopReason()') < tick.indexOf('if (testDataWaiting()) break;'));
assert.ok(tick.indexOf('if (testDataWaiting()) break;') < tick.indexOf('diagreader::readChunk()'));
assert.ok(tick.indexOf('testLastDataAt = lastProgress;') > tick.indexOf('if (!sent) break;'));
assert.doesNotMatch(tick,/vTaskDelay|\bdelay\(/);
console.log(`${checks} connection/pacing checks passed (source simulation only; no firmware build).`);
