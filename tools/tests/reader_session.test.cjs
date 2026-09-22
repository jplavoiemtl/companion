// Executes the actual shared-service C++ bodies with syntax-only JS adaptation.
// No C++/firmware build: mocks replace allocator, clock, critical sections and SD.
const fs=require('node:fs'), path=require('node:path'), vm=require('node:vm'), assert=require('node:assert/strict');
const base=path.join(__dirname,'../../src/diagnostics');
const source=fs.readFileSync(path.join(base,'diagnostics_reader.cpp'),'utf8');
const header=fs.readFileSync(path.join(base,'diagnostics_reader.h'),'utf8');
const usb=fs.readFileSync(path.join(base,'diagnostics_usb.cpp'),'utf8');
function body(sig){const i=source.indexOf(sig);assert.ok(i>=0,sig);const a=source.indexOf('{',i);let n=a+1,d=1;while(d){if(source[n]==='{')++d;if(source[n]==='}')--d;++n;}return source.slice(a+1,n-1);}
function adapt(s){return s
 .replace(/port(?:ENTER|EXIT)_CRITICAL\(&sessionMux\);/g,'')
 .replace(/const (?:bool|size_t|auto) /g,'const ')
 .replace(/\bbool ok =/g,'let ok =')
 .replace(/Accepted value = pending;/g,'const value = {...pending};')
 .replace(/const Published value = snapshot;/g,'const value = {...snapshot};')
 .replace(/pending = \{request, number, at, retainedGeneration, false\};/g,'pending = {request,number,at,generation:retainedGeneration,abort:false};')
 .replace(/pending = \{\};/g,'pending = {request:Request.None,number:0,at:0,generation:0,abort:false};')
 .replace(/const char\* reason = stopReason\(transportStop\)/g,'__unused') // handled below, before actual execution
 .replace(/if \(__unused\) return reason;/g,'const reason=stopReason(transportStop); if(reason)return reason;')
 .replace(/if \(const char\* reason = transportStop\(\)\) return reason;/g,'const reason=transportStop(); if(reason)return reason;')
 .replace(/const uint64_t /g,'const ').replace(/buffers->raw\+bytes/g,'buffers.raw.subarray(bytes)')
 .replace(/Request::/g,'Request.').replace(/diagtransfer::/g,'diagtransfer.')
 .replace(/static_cast<Buffers\*>\(heap_caps_calloc\(1, sizeof\(Buffers\), MALLOC_CAP_SPIRAM \| MALLOC_CAP_8BIT\)\)/g,'allocate()')
 .replace(/nameFor\(\{0,accepted.number,isCurrent\}, filename, sizeof\(filename\)\);/g,'filename=nameFor(accepted.number,isCurrent);')
 .replace(/char path\[64\]; snprintf\(path, sizeof\(path\), "%s%s", ROOT, filename\);/g,'const filePath=ROOT+filename;')
 .replace(/struct stat st\{\};/g,'const st={};')
 .replace(/stat\(path, &st\)/g,'stat(filePath, st)')
 .replace(/open\(path, O_RDONLY\)/g,'open(filePath, O_RDONLY)')
 .replace(/fstat\(reader, &st\)/g,'fstat(reader, st)')
 .replace(/::close/g,'close')
 .replace(/const ssize_t /g,'const ')
 .replace(/size_t\(fileSize-sentBytes\)/g,'Number(fileSize-sentBytes)')
 .replace(/size_t\(got\)/g,'Number(got)')
 .replace(/buffers->/g,'buffers.')
 .replace(/for \(size_t /g,'for (let ')
 .replace(/for \(unsigned /g,'for (let ')
 .replace(/0u/g,'0').replace(/1u/g,'1')
 .replace(/value >> 1/g,'value >>> 1') // C++ value is uint32_t
 .replace(/return value;\s*$/g,'return value;')
 .replace(/\bnullptr\b/g,'null');}
const specs=[['reserve','request,number,at','bool reserve('],['busy','','bool busy()'],['requestAbort','','void requestAbort()'],['abortPending','','bool abortPending()'],['takeAbort','','bool takeAbort()'],['takeRequest','','Accepted takeRequest()'],['invalidate','generation','void invalidate('],['release','generation','bool release('],['publish','','void publish()'],['published','','Published published()'],['start','accepted,transportStop','const char* start('],['stopReason','transportStop','const char* stopReason('],['readChunk','','const char* readChunk()'],['progress','generation,body','bool progress('],['progressBytes','generation,bytes,at','bool progressBytes('],['terminalClock','','void terminalClock()'],['closeReaderAndResume','recordEnd,result,keepEvent=false','bool closeReaderAndResume('],['beforePrune','number','bool beforePrune('],['updateCrc','value,data,length','uint32_t updateCrc(']];
const code=specs.map(([n,a,s])=>`function ${n}(${a}){${adapt(body(s))}}`).join('\n');
function context(){
 const c={nextGeneration:0,retainedGeneration:0,reserved:false,invalidated:false,abortRequested:false,
 pending:{request:0,number:0,at:0,generation:0,abort:false},Request:{None:0,List:1,Current:2,Archive:3,HttpArchive:4},diagtransfer:{busy:()=>false},UINT64_MAX:Number.MAX_SAFE_INTEGER,
 now:1000,buffers:null,entries:null,pendingBytes:0,entryCount:0,reader:-1,paused:false,begun:false,isCurrent:false,
 fileNumber:0,fileSize:0,sentBytes:0,crc:0xffffffff,startedAt:0,lastProgress:0,filename:'',snapshot:{bytes:0,paused:false,result:'none'},
 ROOT:'/sdcard/logs/',O_RDONLY:0,ENOENT:2,errno:0,order:[],frees:[],content:Buffer.from('123456789'),position:0,
 allocOk:true,beginOk:true,pauseOk:true,resumeOk:true,closeOk:true,statOk:true,openOk:true,fstatOk:true,regular:true,
 readFailure:false,transportFailure:null,state:{ready:true,closing:false,queued:0,capacity:16},ends:[]};
 for(const k of ['CHUNK','STALL_MS','CURRENT_MS']){const m=header.match(new RegExp('\\b'+k+' = (\\d+)'));assert.ok(m,k);c[k]=Number(m[1]);}
 c.milliseconds=()=>c.now;c.nameFor=(n,current)=>current?'current.log':`archive-${String(n).padStart(8,'0')}.log`;
 c.allocate=()=>{c.order.push('allocate');return c.allocOk?{raw:Buffer.alloc(c.CHUNK),wire:{}}:null;};
 c.heap_caps_free=x=>c.frees.push(x);c.memmove=(dst,src,n)=>src.copy(dst,0,0,n);
 c.stat=(p,s)=>{c.order.push('stat');s.st_mode=c.regular;s.st_size=c.content.length;return c.statOk?0:-1;};c.S_ISREG=x=>x;
 c.open=()=>{c.order.push('open');c.position=0;return c.openOk?7:-1;};
 c.fstat=(fd,s)=>{c.order.push('fstat');s.st_size=c.content.length;return c.fstatOk?0:-1;};
 c.close=()=>{c.order.push('close');return c.closeOk?0:-1;};
 c.read=(fd,out,n)=>{c.order.push('read');if(c.readFailure)return -1;const k=Math.min(n,c.content.length-c.position);c.content.copy(out,0,c.position,c.position+k);c.position+=k;return k;};
 c.transportStop=()=>c.transportFailure;
 c.captureList=()=>{c.order.push('list');return null;}; // Inventory loop separately inspected; lifecycle cases here use files.
 c.hooks={status:()=>c.state,begin:()=>{c.order.push('begin');return c.beginOk;},pause:()=>{c.order.push('pause');return c.pauseOk;},resume:()=>{c.order.push('resume');return c.resumeOk;},end:(...x)=>{c.order.push('end');c.ends.push(x);}};
 vm.createContext(c);vm.runInContext(code,c);return c;
}
function begin(c,current=true){assert.equal(c.reserve(current?2:3,19,c.now),true);const a=c.takeRequest();assert.equal(c.start(a,c.transportStop),null);return a.generation;}
let checks=0;function check(name,f){f(context());console.log('PASS',name);++checks;}
check('accept reserves once; competing request cannot overwrite metadata or token',c=>{assert.ok(c.reserve(2,0,40));assert.equal(c.reserve(3,19,50),false);const a=c.takeRequest();assert.equal(a.at,40);assert.equal(a.request,2);assert.equal(a.generation,1);assert.equal(c.takeRequest().request,0);assert.ok(c.busy());});
check('queued cancellation before start releases matching reservation without SD work',c=>{c.reserve(2,0,100);c.requestAbort();const a=c.takeRequest();assert.ok(a.abort);c.invalidate(a.generation);assert.equal(c.progress(a.generation,true),false);assert.ok(c.release(a.generation));assert.deepEqual(c.order,[]);assert.equal(c.busy(),false);});
check('invalidated generation retains bytes until matching release and refuses new work',c=>{const g=begin(c);c.readChunk();const held=c.buffers;c.invalidate(g);assert.equal(c.progress(g,true),false);assert.equal(c.sentBytes,0);assert.equal(c.pendingBytes,9);assert.equal(c.buffers,held);assert.equal(c.reserve(3,1,c.now),false);c.closeReaderAndResume(true,'aborted');assert.equal(c.buffers,held);assert.equal(c.frees.length,0);assert.ok(c.release(g));assert.deepEqual(c.frees,[held]);assert.equal(c.release(g),false);});
check('old progress/release/cancel cannot alter a newer reservation',c=>{const old=begin(c);c.invalidate(old);c.closeReaderAndResume(false,'shutdown');c.release(old);const fresh=begin(c);assert.notEqual(old,fresh);c.readChunk();const held=c.buffers;c.invalidate(old);assert.equal(c.progress(old,true),false);assert.equal(c.release(old),false);assert.equal(c.buffers,held);assert.equal(c.invalidated,false);assert.ok(c.progress(fresh,true));assert.equal(c.sentBytes,9);});
check('generation exhaustion refuses rather than wrapping into old identity',c=>{c.nextGeneration=c.UINT64_MAX;assert.equal(c.reserve(2,0,0),false);assert.equal(c.busy(),false);});
check('current snapshot begin/pause/read ordering and frozen size',c=>{begin(c);assert.deepEqual(c.order,['allocate','stat','begin','pause','open','fstat']);assert.equal(c.fileSize,9);assert.equal(c.published().paused,true);c.content=Buffer.from('123456789appended');c.readChunk();assert.equal(c.pendingBytes,9);});
check('USB BEGIN/control metadata progress and CRC body accounting remain distinct',c=>{const g=begin(c);c.now+=10;c.progress(g,false);assert.equal(c.lastProgress,1010);assert.equal(c.sentBytes,0);assert.equal(c.crc,0xffffffff);c.readChunk();c.now+=10;c.progress(g,true);assert.equal(c.sentBytes,9);assert.equal((c.crc^0xffffffff)>>>0,0xcbf43926);assert.equal(c.pendingBytes,0);c.now+=10;c.progress(g,false);assert.equal(c.lastProgress,1030);assert.equal(c.sentBytes,9);});
check('unsent chunk is reused; no read/prefetch progress while backpressured',c=>{begin(c);c.readChunk();c.now+=4999;c.readChunk();assert.equal(c.order.filter(x=>x==='read').length,1);assert.equal(c.lastProgress,1000);assert.equal(c.sentBytes,0);assert.equal(c.stopReason(c.transportStop),null);c.now++;assert.equal(c.stopReason(c.transportStop),'stalled');});
check('multi-chunk binary accounting matches independent CRC reference',c=>{c.content=Buffer.from(Array.from({length:4097},(_,i)=>(i*37)&255));const g=begin(c,false);let crc=0xffffffff;for(const b of c.content){crc^=b;for(let i=0;i<8;i++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}while(c.sentBytes<c.fileSize){assert.equal(c.readChunk(),null);assert.ok(c.pendingBytes<=144);assert.ok(c.progress(g,true));}assert.equal(c.sentBytes,4097);assert.equal(c.crc>>>0,crc>>>0);});
check('normal EOF closes and resumes before END while preserving its event',c=>{const g=begin(c);c.readChunk();c.progress(g,true);assert.ok(c.closeReaderAndResume(false,'ok',true));assert.deepEqual(c.order.slice(-2),['close','resume']);assert.equal(c.begun,true);assert.equal(c.paused,false);assert.equal(c.ends.length,0);c.progress(g,false);c.closeReaderAndResume(true,'ok');assert.equal(c.ends.length,1);assert.equal(c.order.at(-1),'end');c.release(g);});
check('failed read-open still resumes appending; cleanup is idempotent',c=>{c.openOk=false;c.reserve(2,0,0);const a=c.takeRequest();assert.equal(c.start(a,c.transportStop),'read_failed');assert.equal(c.paused,true);c.invalidate(a.generation);assert.ok(c.closeReaderAndResume(true,'read_failed'));assert.equal(c.order.at(-2),'resume');assert.equal(c.order.at(-1),'end');c.closeReaderAndResume(true,'read_failed');assert.equal(c.ends.length,1);c.release(a.generation);});
check('failed pause also takes resume cleanup path',c=>{c.pauseOk=false;c.reserve(2,0,0);const a=c.takeRequest();assert.equal(c.start(a,c.transportStop),'logger_failed');assert.equal(c.paused,true);c.closeReaderAndResume(true,'logger_failed');assert.ok(c.order.includes('resume'));assert.equal(c.order.includes('open'),false);});
check('read error or premature EOF produces read_failed without crediting bytes',c=>{begin(c);c.readFailure=true;assert.equal(c.readChunk(),'read_failed');assert.equal(c.sentBytes,0);c.readFailure=false;c.content=Buffer.alloc(0);assert.equal(c.readChunk(),'read_failed');});
check('reader close failure still attempts resume; reopen failure is not hidden',c=>{begin(c);c.closeOk=false;assert.equal(c.closeReaderAndResume(true,'aborted'),false);assert.ok(c.order.includes('resume'));const d=context();begin(d);d.resumeOk=false;assert.equal(d.closeReaderAndResume(true,'aborted'),false);assert.equal(d.paused,false);});
check('shutdown cleanup produces no transfer END and does not await transport',c=>{const g=begin(c);c.readChunk();c.invalidate(g);assert.ok(c.closeReaderAndResume(false,'shutdown'));assert.equal(c.reader,-1);assert.equal(c.paused,false);assert.equal(c.ends.length,0);assert.ok(c.busy());assert.ok(c.buffers);assert.ok(c.release(g));});
check('archive cleanup never pauses appends; prune matches only the open generation',c=>{begin(c,false);assert.equal(c.paused,false);assert.equal(c.beforePrune(18),false);assert.equal(c.beforePrune(19),true);c.closeReaderAndResume(true,'pruned');assert.equal(c.beforePrune(19),false);assert.equal(c.order.includes('resume'),false);});
check('acceptance time remains deadline origin; guard precedence is preserved',c=>{c.reserve(2,0,1);const a=c.takeRequest();c.now=120001;assert.equal(c.start(a,c.transportStop),'timeout');c.state.queued=8;assert.equal(c.stopReason(c.transportStop),'logger_busy');c.requestAbort();assert.equal(c.stopReason(c.transportStop),'aborted');c.transportFailure='disconnected';assert.equal(c.stopReason(c.transportStop),'disconnected');c.state.ready=false;assert.equal(c.stopReason(c.transportStop),'logger_failed');c.state.closing=true;assert.equal(c.stopReason(c.transportStop),'shutdown');});
check('allocation failure leaves no open reader or pause and can release for retry',c=>{c.allocOk=false;c.reserve(2,0,0);const a=c.takeRequest();assert.equal(c.start(a,c.transportStop),'memory');c.invalidate(a.generation);c.closeReaderAndResume(true,'memory');assert.ok(c.release(a.generation));assert.equal(c.reader,-1);assert.equal(c.paused,false);c.allocOk=true;begin(c);});
check('invalidated start/read cannot produce more data; release requires SD cleanup',c=>{c.reserve(2,0,0);const a=c.takeRequest();c.invalidate(a.generation);assert.equal(c.start(a,c.transportStop),'aborted');assert.equal(c.readChunk(),'aborted');assert.deepEqual(c.order,[]);c.release(a.generation);const g=begin(c);assert.equal(c.release(g),false);assert.ok(c.buffers);c.invalidate(g);assert.equal(c.readChunk(),'aborted');c.closeReaderAndResume(true,'aborted');assert.ok(c.release(g));});
// Run the real adapter tick against the real service bodies for the normal wire path.
check('adapter emits unchanged BEGIN/D/END with resume before END and append readiness',c=>{
 function usbBody(sig){const i=usb.indexOf(sig);assert.ok(i>=0);const a=usb.indexOf('{',i);let n=a+1,d=1;while(d){if(usb[n]==='{')d++;if(usb[n]==='}')d--;n++;}return usb.slice(a+1,n-1);}
 let tick=usbBody('void diagnosticsUsbTick() {')
  .replace(/#if DIAG_USB_TEST_FIXTURE[\s\S]*?#endif/g,'')
  .replace(/const (?:auto|bool|Request|int) /g,'const ')
  .replace(/for \(unsigned /g,'for (let ')
  .replace(/char\* wire =/g,'let wire =').replace(/char wire\[WIRE\+1\];/g,'let wire = {};')
  .replace(/char name\[24\];/g,'let name = {};')
  .replace(/if \(const char\* reason = ([^;\n]+?)\) (?=\{|finishError)/g,'if ((reason = $1)) ')
  .replace(/diagreader::/g,'service.').replace(/Phase::/g,'Phase.').replace(/Request::/g,'Request.').replace(/diagtransfer::/g,'diagtransfer.')
  .replace(/buffers->/g,'buffers.').replace(/sizeof\((wire|name)\)/g,'241')
  .replace(/\(unsigned long long\)/g,'').replace(/\(unsigned long\)/g,'').replace(/unsigned\(/g,'Number(')
  .replace(/const char\* reason =/g,'const reason =').replace(/\bnullptr\b/g,'null');
 // The one-line guard needs no extra scope because it has no declaration body.
 tick=tick.replace('start(accepted)','adapterStart(accepted)');
 c.service={takeRequest:c.takeRequest,readChunk:c.readChunk,progress:c.progress};
 c.phase='Idle';c.Phase={Idle:'Idle',Begin:'Begin',Data:'Data',End:'End',List:'List',Terminal:'Terminal'};
 c.sessionGeneration=0;c.dataLines=0;c.entryIndex=0;c.WIRE=240;c.controlTick=()=>{};
 c.strcmp=(a,b)=>a===b?0:1;c.wireLines=[];
 const readerStart=c.start;
 c.adapterStart=a=>{const err=readerStart(a,c.transportStop);assert.equal(err,null);c.phase='Begin';};
 c.stopReason=()=>null; // Guard behavior exercised by existing USB/reader checks.
 c.finishError=r=>assert.fail('unexpected adapter error '+r);
 c.diagnosticsUsbStop=()=>assert.fail('unexpected shutdown');c.queueError=r=>assert.fail(r);
  // Capture original service release from a fresh context's identical function body in this VM.
 vm.runInContext(`function serviceRelease(generation){${adapt(body('bool release('))}}`,c);
 c.release=()=>{assert.ok(c.serviceRelease(c.sessionGeneration));c.phase='Idle';};
 c.snprintf=(out,cap,fmt,...args)=>{let i=0;out.text=fmt.replace(/%(?:0\d+)?(?:llu|lu|lX|s|u|d)/g,x=>{const a=args[i++];return x.endsWith('lX')?(Number(a)>>>0).toString(16).toUpperCase().padStart(8,'0'):String(a);});return out.text.length;};
 // Adapt pointer arithmetic used to append base64 into the wire scratch.
 tick=tick.replace('wire+prefix','wire');
 c.encode64=(data,len,out)=>{out.text+=Buffer.from(data.subarray(0,len)).toString('base64');};
 c.strcat=(out,s)=>out.text+=s;
 c.sendLine=out=>{assert.ok(Buffer.byteLength(out.text)<=240);if(out.text.startsWith('\n@@END')){assert.equal(c.paused,false);assert.ok(c.order.includes('resume'));}c.wireLines.push(out.text);return 1;};
 c.content=Buffer.from(Array.from({length:301},(_,i)=>i&255));
 vm.runInContext(`function adapterTick(){let reason;${tick}}`,c);
 c.reserve(2,0,c.now);
 for(let i=0;i<10&&c.busy();++i)c.adapterTick();
 assert.equal(c.busy(),false);assert.equal(c.phase,'Idle');assert.equal(c.ends.length,1);
 assert.equal(c.wireLines[0],'\n@@BEGIN version=1 name=current.log size=301\n');
 const data=c.wireLines.filter(x=>x.startsWith('\n@@D '));assert.equal(data.length,3);
 assert.deepEqual(Buffer.concat(data.map((x,i)=>{assert.ok(x.startsWith(`\n@@D ${i+1} `));return Buffer.from(x.trim().split(' ')[2],'base64');})),c.content);
 assert.match(c.wireLines.at(-1),/^\n@@END name=current\.log bytes=301 lines=3 crc32=[0-9A-F]{8}\n$/);
 assert.equal(c.reader,-1);assert.equal(c.paused,false);assert.equal(c.snapshot.result,'ok');
});
// Structural ownership checks complement, not replace, the behavioral cases.
assert.doesNotMatch(usb,/\b(?:opendir|readdir|closedir|fstat|stat|open|read|close)\s*\(/);
assert.doesNotMatch(source,/USBSerial|HWCDC|encode64|@@BEGIN|@@D |@@END|esp_http|\bsend\(/);
assert.match(header,/CHUNK = 144, SCRATCH = 241/);
assert.match(usb,/diagreader::progress\(sessionGeneration,phase == Phase::Data\)/);
assert.ok(usb.indexOf('if (!sent) break;')<usb.indexOf('diagreader::progress(sessionGeneration'));


check('HTTP partial progress credits each accepted prefix once and keeps unsent tail',c=>{
 c.reserve(c.Request.HttpArchive,7,c.now);const a=c.takeRequest();assert.equal(c.start(a,c.transportStop),null);
 assert.equal(c.begun,false);assert(!c.order.includes('begin'));assert(!c.paused);
 c.readChunk();assert(c.progressBytes(a.generation,3,1010));assert.equal(c.sentBytes,3);assert.equal(c.pendingBytes,6);
 assert.equal(c.buffers.raw.subarray(0,6).toString(),'456789');assert.equal(c.lastProgress,1010);
 assert(!c.progressBytes(a.generation,7,1015));assert(c.progressBytes(a.generation,6,1020));
 assert.equal((c.crc^0xffffffff)>>>0,0xcbf43926);assert.equal(c.sentBytes,9);
 c.invalidate(a.generation);assert(!c.progressBytes(a.generation,1,1030));
 c.closeReaderAndResume(false,'ok');assert(c.release(a.generation));
});
check('offline abort consumption preserves the queued request and reservation',c=>{c.reserve(2,0,42);c.requestAbort();assert.equal(c.takeAbort(),true);assert.equal(c.takeAbort(),false);assert.ok(c.busy());const a=c.takeRequest();assert.equal(a.request,2);assert.equal(a.at,42);assert.equal(a.abort,false);});
console.log(`${checks} reader/session checks passed; source simulations only, no firmware build.`);
