// Actual source-body simulations with mocked SD and queue. No firmware build.
const fs=require('node:fs'), vm=require('node:vm'), assert=require('node:assert/strict');
const path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../../src/diagnostics/sd_diagnostics.cpp'),'utf8');
const usb=fs.readFileSync(path.join(__dirname,'../../src/diagnostics/diagnostics_usb.cpp'),'utf8');
const reader=fs.readFileSync(path.join(__dirname,'../../src/diagnostics/diagnostics_reader.cpp'),'utf8');
function body(text,sig){const s=text.indexOf(sig);assert.ok(s>=0,sig);const o=text.indexOf('{',s);let d=1,e=o+1;while(d){if(text[e]==='{')++d;if(text[e]==='}')--d;++e;}return text.slice(o+1,e-1);}
function adapt(t){return t
.replace(/^#(?:if DIAG_USB_TEST_FIXTURE|endif).*$/gm,'')
.replace(/port(?:ENTER|EXIT)_CRITICAL\(&(?:mux|usbMux)\);/g,'')
.replace(/const UsbGateStatus g = usbGate/g,'const g = {...usbGate}')
.replace(/UsbGateStatus\{\}/g,'freshGate()').replace(/Event\{\}/g,'{}')
.replace(/UsbGate::|State::|diag::/g,m=>m.replace('::','.'))
.replace(/bool current; uint32_t number; uint64_t bytes;/g,'let {current,number,bytes} = progress;')
.replace(/diagnosticsUsbTestProgress\(current,number,bytes\)/g,'progress.valid')
.replace(/archiveNumber\(name,number\)/g,'(number=archiveNumber(name)) !== null')
.replace(/decimal\(command\+15,MAX_GENERATION,number\)/g,'((number=decimal(command.slice(15))) !== null)')
.replace(/command\+15/g,'command.slice(15)')
.replace(/const char\* result/g,'let result')
.replace(/const (?:bool|uint64_t|auto) /g,'const ')
.replace(/diagtransfer::busy\(\)/g,'false')
.replace(/diagtransfer::beforePrune\(number\)/g,'httpBeforePrune(number)')
.replace(/diagreader::(\w+)/g,(_,n)=>'reader_'+n)
.replace(/diaginventory::writerChanged\(\)/g,'inventoryChanged()')
.replace(/if \(const char\* reason = transportStop\(\)\) return reason;/g,'const reason=transportStop(); if(reason)return reason;')
.replace(/constexpr uint32_t target = \(QUEUE_COUNT \+ 1\) \/ 2;/g,'const target = Math.floor((QUEUE_COUNT+1)/2);')
.replace(/(?:uint32_t|uint64_t|bool) (\w+) =/g,'let $1 =')
.replace(/static_assert\([^;]+;/g,'')
.replace(/char path\[80\]/g,'const path = {}').replace(/sizeof\(path\)/g,'80')
.replace(/struct stat info\{\}/g,'const info = {}').replace(/stat\(path,&info\)/g,'stat(path,info)')
.replace(/Inventory files;/g,'const files = {};')
.replace(/strcpy\(writerEvent\.(\w+),([^;]+)\);/g,'writerEvent.$1 = $2;')
.replace(/= writerEvent;/g,'= {...writerEvent};')
.replace(/uint32_t\(number\)/g,'Number(number)').replace(/nullptr/g,'null');}
const sigs=[['usbGateBegin','name','void usbGateBegin(const char* name)'],['usbGateEnd','outcome','void usbGateEnd(const char* outcome)'],['usbGateTick','','void usbGateTick()'],['pruneArchive','number','bool pruneArchive(uint32_t number)']];
const code=sigs.map(([n,a,s])=>`function ${n}(${a}){${adapt(body(source,s))}}`).join('\n')+
`\nfunction command(command){${adapt(body(source,'bool diagnosticsCommand(const char* command)').split('  const bool createFixture')[0])}return false;}
function reader_stopReason(transportStop){${adapt(body(reader,'const char* stopReason(const char* (*transportStop)()) {'))}}
function transportStop(){${adapt(body(usb,'const char* transportStop() {'))}}
function reader_beforePrune(number){${adapt(body(reader,'bool beforePrune(uint32_t number) {'))}}
function stopReason(){${adapt(body(usb,'const char* stopReason() {'))}}
function diagnosticsUsbBeforePrune(number){${adapt(body(usb,'void diagnosticsUsbBeforePrune(uint32_t number)'))}}`;
function context(){const c={logRetrievalCommand:()=>false,UsbGate:{None:0,Queue:1,Prune:2},State:{Ready:1},QUEUE_COUNT:16,FILE_LIMIT:2097152,
queue:Array(16).fill(null),head:14,count:0,accepting:true,writerEvent:{},snapshot:{state:1,highWater:0,pruned:0},closeRequested:false,
fixture:{busy:false,number:19,bytes:2097152,result:'ok'},fixtureFd:-1,fixtureDeleting:false,fixtureNumber:0,errno:5,
progress:{valid:true,current:true,number:0,bytes:1440},paused:true,busy:false,reader:7,isCurrent:true,fileNumber:0,
order:[],removed:[],errors:[],records:[],fileExists:true,fileSize:2097152,regular:true,unlinkError:0,
connectionLostAt:0,abortRequested:false,startedAt:100,lastProgress:100,DISCONNECT_MS:1000,CURRENT_MS:120000,STALL_MS:5000,
freshGate:()=>({armed:0,active:0,target:0,added:0,queued:0,fired:false,result:'none',outcome:'none'}),
strcmp:(a,b)=>a===b?0:1,strncmp:(a,b,n)=>a.slice(0,n)===b.slice(0,n)?0:1,strlen:s=>s.length,
decimal:s=>/^[0-9]+$/.test(s)&&Number(s)<=99999999?Number(s):null,
archiveNumber:s=>/^archive-[0-9]{8}\.log$/.test(s)?Number(s.slice(8,16)):null,
max:Math.max,printUsbGate:()=>{},vTaskDelay:()=>{},USBSerial:{println:()=>{}},milliseconds:()=>100,transferConnected:()=>true};
c.abortPending=()=>c.abortRequested;
c.usbGate=c.freshGate();c.diagnosticsUsbPaused=()=>c.paused;c.diagnosticsUsbBusy=()=>c.busy;
c.hooks={status:()=>({ready:true,closing:false,queued:c.count,capacity:16})};c.diag={stamp:()=>123,record:(...a)=>c.records.push(a)};
c.archivePath=(n,p)=>p.number=n;c.stat=(p,i)=>{i.st_size=c.fileSize;i.st_mode=c.regular;return c.fileExists?0:-1;};c.S_ISREG=x=>x;
c.unlink=p=>{c.order.push('unlink');assert.equal(c.reader,-1);if(!c.unlinkError)c.removed.push(p.number);return c.unlinkError;};
c.inventory=()=>true;c.spaceAvailable=()=>true;c.disable=(...a)=>c.errors.push(a);
c.finishError=r=>{c.order.push('close');c.reader=-1;c.usbGateEnd(r);};c.inventoryChanged=()=>{c.inventoryInvalidations=(c.inventoryInvalidations||0)+1;};vm.createContext(c);vm.runInContext(code,c);return c;}
let checks=0;function check(n,f){f(context());console.log('PASS',n);++checks;}
function queue(c){c.command('log test queue');c.usbGateBegin('current.log');}
function prune(c){c.command('log test prune 19');c.usbGateBegin('archive-00000019.log');c.progress.current=false;c.progress.number=19;c.isCurrent=false;c.fileNumber=19;}
check('actual queue reaches half; preserves real entries and triggers production guard',c=>{c.count=2;c.queue[14]={real:1};c.queue[15]={real:2};queue(c);c.usbGateTick();assert.equal(c.count,8);assert.equal(c.usbGate.added,6);assert.equal(c.snapshot.highWater,8);assert.equal(c.queue[14].real,1);assert.equal(c.queue[15].real,2);for(let i=0;i<6;++i)assert.equal(c.queue[i].event,'USB_QUEUE_TEST');assert.equal(c.stopReason(),'logger_busy');});
check('wait for data and current pause',c=>{queue(c);c.progress.bytes=1439;c.usbGateTick();assert.equal(c.count,0);c.progress.bytes=1440;c.paused=false;c.usbGateTick();assert.equal(c.count,0);c.paused=true;c.usbGateTick();assert.equal(c.count,8);});
check('one shot never refills or repeats on retry',c=>{queue(c);c.usbGateTick();c.count=0;c.usbGateTick();assert.equal(c.count,0);c.usbGateEnd('logger_busy');c.usbGateBegin('current.log');c.usbGateTick();assert.equal(c.count,0);});
check('wrong file consumes arm safely',c=>{c.command('log test queue');c.usbGateBegin('archive-00000019.log');c.usbGateTick();assert.equal(c.usbGate.result,'wrong_file');assert.equal(c.count,0);});
check('disarm idle only',c=>{c.command('log test queue');c.command('log test usb off');assert.equal(c.usbGate.armed,0);queue(c);c.busy=true;c.command('log test usb off');assert.equal(c.usbGate.active,1);});
check('arming refuses busy, shutdown, unavailable and fixture activity',c=>{for(const [o,k]of[[c,'busy'],[c,'closeRequested'],[c.fixture,'busy']]){o[k]=true;c.command('log test queue');assert.equal(c.usbGate.armed,0);o[k]=false;}c.snapshot.state=0;c.command('log test queue');assert.equal(c.usbGate.armed,0);});
check('prune rejects real archives, old ownership and malformed numbers',c=>{for(const n of ['14','0','-1','../19','19x','100000000','']){c.command('log test prune '+n);assert.equal(c.usbGate.armed,0);}c.fixture.result='none';c.command('log test prune 19');assert.equal(c.usbGate.armed,0);});
check('fresh fixture reader closes before unlink, only once',c=>{prune(c);c.usbGateTick();assert.deepEqual(c.order,['close','unlink']);assert.deepEqual(c.removed,[19]);assert.equal(c.snapshot.pruned,1);assert.equal(c.usbGate.result,'pruned');assert.equal(c.usbGate.outcome,'pruned');assert.equal(c.fixture.result,'pruned_by_test');c.usbGateTick();assert.equal(c.removed.length,1);});
check('ownership is rechecked at firing',c=>{prune(c);c.fixture.number=20;c.usbGateTick();assert.equal(c.usbGate.result,'fixture_not_owned');assert.equal(c.removed.length,0);});
check('changed, missing or nonregular fixtures preserved',()=>{for(const [k,v]of[['fileSize',1],['fileExists',false],['regular',false]]){const c=context();prune(c);c[k]=v;c.usbGateTick();assert.equal(c.usbGate.result,'fixture_changed');assert.equal(c.removed.length,0);}});
check('unlink failure not reported as pass',c=>{prune(c);c.unlinkError=-1;c.usbGateTick();assert.equal(c.usbGate.result,'prune_failed');assert.equal(c.snapshot.pruned,0);assert.equal(c.errors[0][0],'archive_delete');});
check('retention uses same removal helper; normal default remains off',()=>{assert.match(body(source,'bool prune(uint64_t incoming'),/pruneArchive\(files.oldest\)/);assert.match(fs.readFileSync(path.join(__dirname,'../../src/diagnostics/diagnostics_config.h'),'utf8'),/#define DIAG_USB_TEST_FIXTURE 0/);});
console.log(`${checks} logger gate source simulations passed`);
