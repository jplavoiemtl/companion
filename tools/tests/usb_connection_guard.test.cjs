// Source-level simulation of the actual C++ guards, not a firmware build.
// Only declaration/cast syntax is adapted; mocked time/USB drive each branch.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync(path.join(__dirname, '../../src/diagnostics/diagnostics_usb.cpp'), 'utf8');
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
    .replace(/bool abort =/g, 'const abort =')
    .replace(/port(?:ENTER|EXIT)_CRITICAL\(&usbMux\);/g, '')
    .replace(/if \(const char\* reason = stopReason\(\)\) \{/g,
      'const reason = stopReason(); if (reason) {')
    .replace(/reinterpret_cast<const uint8_t\*>\(bytes\)/g, 'bytes')
    .replace(/\bint\(/g, 'Number(')
    .replace(/\bnullptr\b/g, 'null');
}
const adapted = `function transferConnected(){${body('bool transferConnected() {')}}
function stopReason(){${body('const char* stopReason() {')}}
function sendLine(bytes, transfer=false){${body('int sendLine(const char* bytes, bool transfer = false) {')}}`;
function context() {
  const c = {
    now:1000, connected:true, readings:[], writes:0, free:240, shortWrite:false,
    connectionLost:false, connectionLostAt:0, longestLoss:0, connectionLosses:0,
    lastProgress:1000, startedAt:1000, isCurrent:false, abortRequested:false,
    state:{closing:false, ready:true, queued:0, capacity:16},
    lastLineBytes:0, lastTxFree:-1, lastWriteBytes:-1,
    lastSendCheck:'none', lastSendStop:'none',
    strlen:s=>s.length,
  };
  for (const name of ['DISCONNECT_MS','STALL_MS','CURRENT_MS','WIRE']) {
    const match=source.match(new RegExp('\\b'+name+' = (\\d+)'));
    assert.ok(match, name); c[name]=Number(match[1]);
  }
  c.milliseconds=()=>c.now;
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
console.log(`${checks} connection guard checks passed (source simulation only).`);
