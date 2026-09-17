// Node-only tests of the actual inline page code. No board, serial port or build.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync(path.join(__dirname, '../sd_log_browser.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script); // Parse all UI code as well as the protocol core.
const core = script.split('// BEGIN PROTOCOL CORE')[1].split('// END PROTOCOL CORE')[0];
const coreContext = vm.createContext({Uint8Array, atob, btoa});
const Receiver = vm.runInContext(core + '\nLogFileReceiver', coreContext);
function crc32(bytes) {
  // Independent table implementation, checked against the standard test vector.
  let crc = 0xffffffff;
  for (const byte of bytes) {
    let entry = (crc ^ byte) & 255;
    for (let i=0; i<8; ++i) entry = (entry >>> 1) ^ ((entry & 1) ? 0xedb88320 : 0);
    crc = (crc >>> 8) ^ entry;
  }
  return ((crc ^ 0xffffffff) >>> 0).toString(16).padStart(8,'0').toUpperCase();
}
assert.equal(crc32(Buffer.from('123456789')), 'CBF43926');
function wire(bytes, name='current.log') {
  const lines=[`@@BEGIN version=1 name=${name} size=${bytes.length}`];
  for (let offset=0; offset<bytes.length; offset+=144)
    lines.push(`@@D ${lines.length} ${bytes.subarray(offset,offset+144).toString('base64')}`);
  lines.push(`@@END name=${name} bytes=${bytes.length} lines=${lines.length-1} crc32=${crc32(bytes)}`);
  assert.ok(lines.every(l=>l.length+2<=240));
  return lines;
}
let checks=0;
function check(name,fn) { fn(); ++checks; console.log('PASS',name); }
check('empty, standard CRC vector and multi-chunk binary round trips',()=>{
  for (const bytes of [Buffer.alloc(0),Buffer.from('123456789'),Buffer.from(Array.from({length:4097},(_,i)=>i%256))]) {
    const r=new Receiver('current.log'), lines=wire(bytes);
    r.begin(lines.shift()); const end=lines.pop(); lines.forEach(l=>r.data(l));
    assert.deepEqual(Buffer.from(r.end(end)),bytes);
  }
});
check('2 MiB synthetic archive crosses large sequence numbers and verifies every byte',()=>{
  const bytes=Buffer.alloc(2*1024*1024, '.');
  for (let offset=0; offset<bytes.length; offset+=64) {
    bytes.write('USB_TEST_FIXTURE line='+String(offset/64).padStart(8,'0')+' ',offset,'ascii');
    bytes[offset+63]=10;
  }
  assert.equal(crc32(bytes),'8D218D21');
  const name='archive-00000018.log', r=new Receiver(name), lines=wire(bytes,name);
  r.begin(lines.shift()); const end=lines.pop(); lines.forEach(l=>r.data(l));
  assert.ok(r.lines>10000);
  assert.deepEqual(Buffer.from(r.end(end)),bytes);
});
check('BEGIN rejects version, path, unexpected name and unbounded size',()=>{
  for (const line of ['@@BEGIN version=2 name=current.log size=0','@@BEGIN version=1 name=../current.log size=0',
    '@@BEGIN version=1 name=archive-00000014.log size=0','@@BEGIN version=1 name=current.log size=67108865',
    '@@BEGIN version=1 name=current.log size=-1']) assert.throws(()=>new Receiver('current.log').begin(line));
});
check('strict base64, padding, length and sequence checks',()=>{
  for (const data of ['@@D 2 QQ==','@@D 1 QR==','@@D 1 Q Q=','@@D 1 Q===','@@D 1 QQ','@@D 1 !Q==',
    '@@D 1 '+Buffer.alloc(145).toString('base64')]) {
    const r=new Receiver('current.log'); r.begin('@@BEGIN version=1 name=current.log size=144');
    assert.throws(()=>r.data(data));
  }
  const r=new Receiver(); r.begin('@@BEGIN version=1 name=current.log size=2');r.data('@@D 1 QQ==');
  assert.throws(()=>r.data('@@D 1 QQ=='));
});
check('END rejects CRC, filename, byte count, line count and truncation',()=>{
  const bytes=Buffer.from('123456789'), good=wire(bytes);
  for (const end of [good[2].replace('CBF43926','00000000'),good[2].replace('current.log','archive-00000001.log'),
    good[2].replace('bytes=9','bytes=8'),good[2].replace('lines=1','lines=2')]) {
    const r=new Receiver();r.begin(good[0]);r.data(good[1]);assert.throws(()=>r.end(end));
  }
  const r=new Receiver();r.begin(good[0]);assert.throws(()=>r.end(good[2]));
});

function page() {
  const nodes=new Map(), intervals=[], timers=new Map(), sent=[], saved=[];
  let now=0, nextTimer=1;
  function node(id) {
    if (nodes.has(id)) return nodes.get(id);
    const n={value:'',textContent:'',disabled:false,checked:false,children:[],handlers:{},scrollHeight:0,
      scrollTop:0,clientHeight:0,append(...items){this.children.push(...items)},replaceChildren(){this.children=[]},
      querySelectorAll(){return []},addEventListener(event,fn){this.handlers[event]=fn},click(){if(this.download)saved.push(this.download)}};
    nodes.set(id,n);return n;
  }
  node('mode').value='explicit';node('dtr').value='true';node('rts').value='false';
  const context=vm.createContext({Uint8Array,atob,btoa,TextDecoder,TextEncoder,Blob,Date,console,
    performance:{now:()=>now},URL:{createObjectURL:()=> 'blob:test',revokeObjectURL(){}},
    navigator:{serial:{addEventListener(){}},userAgent:'test'},
    window:{isSecureContext:true,addEventListener(){}},
    document:{getElementById:node,createElement:()=>node('new'+nextTimer++)},
    setTimeout:(fn,ms)=>{const id=nextTimer++;timers.set(id,{fn,ms});return id},clearTimeout:id=>timers.delete(id),
    setInterval:fn=>intervals.push(fn),queueMicrotask,
    mockWriter:{write:async data=>sent.push(new TextDecoder().decode(data)),abort:async()=>{},releaseLock(){}},
    mockPort:{close:async()=>{}},
  });
  vm.runInContext(script+`
    current={opened:true,closing:false,writer:mockWriter,reader:null,port:mockPort};
    globalThis.api={receive,downloadFile,cancelTransfer,refreshFiles,controls,consoleText,
      job:()=>download,files:()=>files,partial:()=>partial,disconnected:()=>current===null};`,context);
  return {api:context.api,node,sent,saved,advance:ms=>{now+=ms;intervals.forEach(f=>f())}};
}
async function main() {
  let p=page();
  await p.api.refreshFiles();
  p.api.receive('\n@@STATUS boot=38 logger=ready\n\n@@FILE name=current.log size=100\n\n@@FILE name=archive-00000014.log size=8059\n\n@@LIST_END count=2\n');
  assert.equal(p.api.files().size,2);assert.equal(p.node('refresh').disabled,false);++checks;
  console.log('PASS automatic status/list parsing and controls');

  const bytes=fs.readFileSync(path.join(__dirname,'../../docs/bench_data/sd_logs_2026-09-17_offline_rotation/logs/archive-00000014.log.txt'));
  const result=p.api.downloadFile('archive-00000014.log');
  const stream=wire(bytes,'archive-00000014.log').map(l=>'\n'+l+'\n').join('debug text\n');
  for (let i=0;i<stream.length;i+=7) p.api.receive(stream.slice(i,i+7));
  assert.equal(await result,true);assert.equal(p.saved[0],'38-archive-00000014.log');++checks;
  console.log('PASS actual SD archive through split reads and interleaved debug');

  p=page();p.node('damage').checked=true;
  const damaged=p.api.downloadFile('current.log');
  wire(Buffer.from('123456789')).forEach(l=>p.api.receive('\n'+l+'\n'));
  assert.ok(p.api.job().cancelling);assert.equal(p.saved.length,0);
  assert.equal(await p.api.downloadFile('current.log'),false);
  p.api.receive('\n@@ERR reason=aborted\n');assert.equal(await damaged,false);
  const retry=p.api.downloadFile('current.log');wire(Buffer.from('123456789')).forEach(l=>p.api.receive('\n'+l+'\n'));
  assert.equal(await retry,true);++checks;
  console.log('PASS damaged-line rejection, abort barrier and successful retry');

  p=page();const busy=p.api.downloadFile('current.log');
  p.api.receive('\n@@D 99 QQ==\n');assert.equal(p.api.job().cancelling,false);
  p.api.receive('\n@@BEGIN version=1 name=current.log size=0\n\n@@ERR reason=busy\n');
  assert.equal(p.api.job().cancelling,false);
  p.api.receive('\n@@END name=current.log bytes=0 lines=0 crc32=00000000\n');
  assert.equal(await busy,true);++checks;
  console.log('PASS stray pre-BEGIN data and rejected extra command preserve active transfer');

  p=page();const missing=p.api.downloadFile('current.log');
  p.api.receive('\n@@BEGIN version=1 name=current.log size=9\n');p.advance(16000);
  assert.ok(p.api.job().cancelling);p.api.receive('\n@@ERR reason=aborted\n');
  assert.equal(await missing,false);assert.equal(p.saved.length,0);++checks;
  console.log('PASS missing END times out without saving');

  p=page();const clearing=p.api.downloadFile('current.log');
  p.api.receive('\n@@BEGIN version=1 name=current.log si');
  p.node('clear').handlers.click();assert.ok(p.api.partial().includes('name=current.log si'));
  p.api.receive('ze=0\n\n@@END name=current.log bytes=0 lines=0 crc32=00000000\n');
  assert.equal(await clearing,true);++checks;
  console.log('PASS console clear preserves protocol fragments');

  p=page();const long=p.api.downloadFile('current.log');
  p.api.receive('\n@@'+ 'X'.repeat(5000)+'\n');assert.ok(p.api.job().cancelling);
  p.api.receive('\n@@ERR reason=aborted\n');assert.equal(await long,false);++checks;
  console.log('PASS oversized protocol line fails safely');
  assert.match(p.node('progress').textContent,/abort confirmed/);
  p=page();const nonAscii=p.api.downloadFile('current.log');
  p.api.receive('\n@@BEGIN version=1 name=current.log size=0\u0000\n');
  assert.ok(p.api.job().cancelling);
  assert.match(p.api.consoleText(),/first_non_ascii=\d+:U\+0000/);
  assert.ok(p.api.consoleText().includes('\\u0000'));
  p.api.receive('\n@@ERR reason=aborted\n');
  assert.equal(await nonAscii,false);++checks;
  console.log('PASS non-ASCII protocol line still aborts safely with diagnostic evidence');
  p=page();const stalled=p.api.downloadFile('current.log');
  const partial=wire(Buffer.alloc(1440));
  partial.slice(0,9).forEach(l=>p.api.receive('\n'+l+'\n'));
  assert.equal(p.api.job().receiver.offset,1152);
  p.api.receive('\n@@ERR reason=stalled phase=data bytes=1152 line_bytes=200 tx_free=128 write_bytes=-1\n');
  assert.equal(await stalled,false);
  assert.equal(p.saved.length,0);
  assert.match(p.api.consoleText(),/tx_free=128 write_bytes=-1/);
  ++checks;console.log('PASS partial transfer retains stall diagnostics and saves no file');
  console.log(`${checks} checks passed; no hardware accessed.`);
}
const deadline = setTimeout(()=>{console.error('Test promise did not settle');process.exit(1)},5000);
main().catch(e=>{console.error(e);process.exitCode=1}).finally(()=>clearTimeout(deadline));
