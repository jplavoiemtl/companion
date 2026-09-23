// Execute actual mailbox/writer bodies with platform and reader mocks. No firmware build.
const fs=require('fs'), vm=require('vm'), assert=require('assert/strict');
const source=fs.readFileSync('src/diagnostics/diagnostics_http_transfer.cpp','utf8');
const http=fs.readFileSync('src/diagnostics/diagnostics_http.cpp','utf8');
const usb=fs.readFileSync('src/diagnostics/diagnostics_usb.cpp','utf8');
const sd=fs.readFileSync('src/diagnostics/sd_diagnostics.cpp','utf8');
function body(s,sig){let i=s.indexOf(sig);assert(i>=0,sig);i=s.indexOf('{',i);let j=i+1,d=1;while(d){if(s[j]==='{')d++;if(s[j]==='}')d--;j++;}return s.slice(i+1,j-1);}
function adapt(s){return s.replace(/port(?:ENTER|EXIT)_CRITICAL\(&mux\);/g,'')
 .replace(/const (?:bool|uint64_t|char\*|View|Result|auto&)\s*/g,'const ')
 .replace(/uint64_t bytes,at;/g,'let bytes,at;')
 .replace(/diagreader::Request::/g,'Request.').replace(/diagreader::/g,'reader.').replace(/diaginventory::/g,'inventory.')
 .replace(/View\{\}/g,'fresh()').replace(/View s=box/g,'const s=structuredClone(box)').replace(/Result s=lastResult/g,'const s=({...lastResult})')
 .replace(/lastResult=result/g,'lastResult={...result}')
 .replace(/if \(const reason=([^;]+)\) \{/g,'{ const reason=$1; if(reason) {') // extra close handled below
 .replace(/\{ const reason=([^;]+); if\(reason\) \{ close\(reason\); return; \}/g,'{ const reason=$1; if(reason) { close(reason); return; } }')
 .replace('box.writerCrc=writerState.crc ^ 0xffffffff','box.writerCrc=(writerState.crc ^ 0xffffffff)>>>0')
 .replace(/size_t\(/g,'Number(').replace(/r.buffers->raw/g,'r.buffers.raw').replace(/nullptr/g,'null');}
const defs=[['compareWriter','result,closed','void compareWriter('],['view','','View view()'],['busy','','bool busy()'],['request','number,now,current=false','uint64_t request('],['progress','id,bytes,at','bool progress('],['cancel','id,reason','void cancel('],['cancelAll','reason','void cancelAll('],['release','id,result','void release('],['last','','Result last()'],['transportStop','','const char* transportStop('],['close','reason','void close('],['finishRelease','','void finishRelease()'],['writerOnline','','void writerOnline()'],['writerOffline','','void writerOffline()'],['failure','','const char* failure()'],['accept','request','void accept('],['tick','','void tick()'],['stop','pending','void stop('],['beforePrune','number','void beforePrune('],['offlineTick','','void offlineTick()']];
function context(){
 const fresh=()=>({id:0,generation:0,size:0,offset:0,started:0,closedAt:0,cancelledAt:0,readerAt:0,writerBytes:0,writerCrc:0,pausedAt:0,readerClosedAt:0,resumedAt:0,length:0,reserved:false,metadata:false,closed:false,released:false,result:'none',data:new Uint8Array(144)});
 const c={fresh,structuredClone,box:fresh(),lastResult:{},nextId:0,acknowledged:0,progressAt:0,cancellation:null,readerGeneration:0,started:false,online:true,t:1000,UINT64_MAX:Number.MAX_SAFE_INTEGER,Request:{None:0,HttpArchive:4,HttpCurrent:5},calls:[],storageReady:true,releaseOk:true,startError:null,readError:null,stopError:null,g:0,held:false,invalid:false,content:Buffer.from('123456789'),position:0,r:{pausedAt:0,readerClosedAt:0,resumedAt:0,fileSize:0,sentBytes:0,crc:0xffffffff,pendingBytes:0,buffers:{raw:Buffer.alloc(144)}}};
 c.nowMs=()=>c.t;c.strcmp=(a,b)=>a===b?0:1;c.memcpy=(dst,src,n)=>dst.set(src.subarray(0,n));
 c.inventory={writerPreempt:()=>c.calls.push('preempt')};
 c.reader={reserve:(request,number,at)=>{if(c.held)return false;c.held=true;c.g++;c.pending={request,number,at,generation:c.g,abort:false};return true;},
 view:()=>c.r,start:(a,stop)=>{c.calls.push('start');c.invalid=false;c.position=0;c.r.sentBytes=0;c.r.pendingBytes=0;c.r.fileSize=c.content.length;return stop()||c.startError;},
 invalidate:()=>{c.invalid=true;c.calls.push('invalidate');},closeReaderAndResume:()=>{c.calls.push('close');return true;},
 release:g=>{c.calls.push('release');if(!c.releaseOk)return false;c.held=false;return true;},
 progressBytes:(g,n,at)=>{assert(!c.invalid);assert(n<=c.r.pendingBytes);c.calls.push(['progress',n,at]);c.r.sentBytes+=n;c.r.pendingBytes-=n;c.r.buffers.raw.copyWithin(0,n,n+c.r.pendingBytes);return true;},
 stopReason:stop=>c.stopError||stop(),readChunk:()=>{c.calls.push('read');if(c.readError)return c.readError;const n=Math.min(144,c.content.length-c.position);c.content.copy(c.r.buffers.raw,0,c.position,c.position+n);c.position+=n;c.r.pendingBytes=n;return null;},beforePrune:n=>n===7};
 vm.createContext(c);for(const [name,args,sig] of defs)vm.runInContext(`function ${name}(${args}){${adapt(body(source,sig))}}`,c);return c;
}
let count=0;function test(name,fn){fn();count++;console.log('PASS '+name);}
function begin(c){const id=c.request(7,c.t);assert(id);c.accept(c.pending);c.tick();return id;}
test('USB reservation excludes HTTP without changing mailbox',()=>{const c=context();c.held=true;assert.equal(c.request(7,1000),0);assert.equal(c.box.id,0);});
test('writer offline admission closes shutdown race before reservation',()=>{const c=context();c.writerOffline();assert.equal(c.request(7,1000),0);assert(!c.held);});
test('identity exhaustion refuses safely',()=>{const c=context();c.nextId=c.UINT64_MAX;assert.equal(c.request(7,1000),0);});
test('partial progress holds immutable published bytes until full acknowledgement',()=>{const c=context();const id=begin(c);const data=Array.from(c.box.data);assert(c.progress(id,3,1010));c.tick();assert.equal(c.r.sentBytes,3);assert.equal(c.r.pendingBytes,6);assert.deepEqual(Array.from(c.box.data),data);assert.equal(c.calls.filter(x=>x==='read').length,1);assert(!c.progress(id,3,1015));assert(!c.progress(id,10,1015));assert(c.progress(id,9,1020));c.tick();assert(c.box.closed);assert.equal(c.box.result,'ok');assert(c.held);});
test('writer cancellation closes before transport release without reclaiming reservation',()=>{const c=context();const id=begin(c);c.cancel(id,'mode_exit');c.tick();assert(c.box.closed);assert(c.held);assert.equal(c.request(8,1100),0);assert(!c.progress(id,3,1100));assert(!c.calls.includes('release'));c.release(id,{id,result:'mode_exit',releasedAt:1100});c.tick();assert(!c.held);assert(!c.busy());});
test('stale release/progress/cancellation cannot change a reused generation',()=>{const c=context();const old=begin(c);c.cancel(old,'aborted');c.tick();c.release(old,{id:old,result:'aborted',releasedAt:1000});c.tick();const id=begin(c);assert(id>old);c.cancel(old,'stale');c.release(old,{id:old,result:'bad'});assert(!c.progress(old,1,1100));assert(!c.box.released);assert.equal(c.cancellation,null);});
test('positive late prefix may be reported after cancellation while progress stays invalid',()=>{const c=context();const id=begin(c);c.cancel(id,'pruned');c.tick();assert(!c.progress(id,3,1100));c.release(id,{id,bytes:3,crc:123,result:'cancelled',releasedAt:1100});assert.equal(c.lastResult.bytes,3);assert.equal(c.lastResult.result,'pruned');c.tick();assert(!c.held);});
test('queued shutdown initializes cleanup without any file read and accepts late release offline',()=>{const c=context();const id=c.request(7,1000);c.writerOffline();c.stop(c.pending);assert(c.box.closed);assert(!c.calls.includes('read'));assert(c.held);c.release(id,{id,result:'shutdown',releasedAt:1100});c.offlineTick();assert(!c.held);});
test('prune closes reader before returning while keeping network reservation',()=>{const c=context();begin(c);c.beforePrune(7);assert(c.box.closed);assert(c.calls.includes('close'));assert(c.held);});
test('start/read/queue errors close and preserve reason',()=>{for(const field of ['startError','readError','stopError']){const c=context();c[field]='not_found';const id=begin(c);assert(c.box.closed);assert.equal(c.box.result,'not_found');c.release(id,{id,result:'not_found',releasedAt:1100});c.tick();assert(!c.held);}});
test('missing or refused release remains retained and escalates after interval',()=>{const c=context();const id=begin(c);c.cancel(id,'aborted');c.tick();c.releaseOk=false;c.release(id,{id,result:'aborted',releasedAt:1000});c.tick();c.t=10999;assert.equal(c.failure(),null);c.t=11000;assert.equal(c.failure(),'reader_release');assert(c.held);c.releaseOk=true;c.tick();assert(!c.held);});
test('empty file still supplies metadata and error in sending its header is never relabelled success',()=>{const c=context();c.content=Buffer.alloc(0);const id=begin(c);assert(c.box.metadata);assert(c.box.closed);assert.equal(c.box.size,0);c.release(id,{id,result:'header_send_failed',releasedAt:1000});assert.equal(c.lastResult.result,'header_send_failed');});
test('ordinary shutdown adds no network wait and terminal fallback runs only after writer stops',()=>{assert.doesNotMatch(body(source,'void stop('),/nap|Delay|while|httpd|send\(/);assert.match(body(usb,'void diagnosticsUsbOfflineTick()'),/diagtransfer::offlineTick/);assert.match(sd,/if \(!writerRunning\(readSnapshot\(\)\.writerLifecycle\)\) diagnosticsUsbOfflineTick/);assert(sd.indexOf('diagtransfer::writerOffline();')<sd.indexOf('diagnosticsUsbStop(); // Close reader'));});
test('HTTP response has fixed length attachment and never chunks or appends error after headers',()=>{const b=body(http,'esp_err_t download(httpd_req_t* req, SessionIo* io, uint32_t number, bool current) {');assert.match(b,/Content-Type: application\/octet-stream/);assert.match(b,/Content-Disposition: attachment/);assert.match(b,/Content-Length: %llu/);assert.doesNotMatch(b,/send_chunk|Transfer-Encoding/);assert.match(b,/if\(!headersSent && !cancelled/);assert.match(b,/updateCrc\(crc,state.data\+offset,size_t\(n\)\)/);assert.match(b,/result.bytes\+=size_t\(n\)/);assert.match(b,/diagtransfer::progress\(id,result.bytes,progressAt\)/);});
test('canonical managed archive route and incidental methods never enter download',()=>{const b=body(http,'esp_err_t handle(');assert(b.indexOf('req->method != HTTP_GET')<b.indexOf('return download'));assert.match(b,/strlen\(req->uri\+3\)==8/);assert.match(b,/parseNumber\(req->uri\+3,number\)/);assert.doesNotMatch(b,/Request::Current|opendir|open\(/);});
test('HTTP task cannot access reader-owned buffers or lifecycle',()=>{assert.doesNotMatch(http,/diagreader::(?:view|start|readChunk|progressBytes|release|invalidate|closeReaderAndResume)\(/);assert.match(body(http,'void worker('),/snapshot\(\)\.handlers \|\| diagtransfer::busy\(\)/);});

function downloadContext(content=Buffer.from('123456789')) {
 const c={HTTPD_RESP_USE_STRLEN:-1,MALLOC_CAP_INTERNAL:1,MALLOC_CAP_8BIT:2,PAGE_CAP:32768,IO_MS:5000,ESP_FAIL:-1,t:1000,page:{value:''},req:{},io:{generation:1,bodyOutput:false},
 content,accepted:0,closed:false,reason:'ok',cancelledMode:false,records:[],raw:'',body:[],statusCode:null,reply:null,reservationAttempts:0,
 released:null,ready:true,busy:false,metadata:true,sendSteps:[],range:false,ifRange:false,serverFailure:null,requestOk:true};
 c.nowMs=()=>c.t;c.nap=()=>c.t+=20;c.vTaskDelay=()=>{c.t++;if(c.reason!=='ok')c.closed=true;};
 c.strcmp=(a,b)=>a===b?0:1;c.strlen=x=>x.length;c.cancelled=()=>c.cancelledMode;
 c.shared={cancel:false,generation:1,userActivity:0};
 c.headerPresent=(r,n)=>n==='Range'?c.range:c.ifRange;
 c.reader={status:()=>({ready:c.ready,closing:false,boot:108}),busy:()=>c.busy,updateCrc:(crc,data,n)=>{for(const b of data.subarray(0,n)){crc^=b;for(let j=0;j<8;j++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}return crc>>>0;}};
 c.transfer={request:()=>{c.reservationAttempts++;return c.requestOk?1:0;},
 view:()=>({id:1,metadata:c.metadata,size:c.content.length,closed:c.closed,result:c.reason,closedAt:c.closed?c.t:0,cancelledAt:c.reason==='ok'?0:c.t,readerAt:1001,pausedAt:c.pausedAt||0,readerClosedAt:c.closed?c.t:0,resumedAt:c.resumedAt||0,writerBytes:c.writerBytesOverride ?? c.accepted,writerCrc:c.writerCrcOverride ?? ((c.reader.updateCrc(0xffffffff,c.content,c.accepted)^0xffffffff)>>>0),offset:0,length:c.content.length,data:c.content}),
 progress:(id,bytes,at)=>{assert(bytes>c.accepted);c.accepted=bytes;if(bytes===c.content.length)c.closed=true;return true;},
 cancel:(id,r)=>{c.reason=r;c.closed=true;},release:(id,r)=>{c.released={...r};}};
 c.transfer.compareWriter=(result,closed)=>c.compareWriter(result,closed);
 c.diag={record:(name,page)=>{c.records.push({name,text:page.value});return true;}};
 c.snprintf=(page,cap,fmt,...args)=>{let i=0;page.value=fmt.replace(/%08lu|%08lX|%llu|%u|%s/g,m=>{const a=args[i++];if(m==='%08lu')return String(a).padStart(8,'0');if(m==='%08lX')return (a>>>0).toString(16).toUpperCase().padStart(8,'0');return String(a);});assert(page.value.length<cap);return page.value.length;};
 c.httpd_resp_set_status=(r,s)=>c.statusCode=s;c.httpd_resp_send=(r,data,n)=>{c.reply=data?.slice(0,n);return 0;};
 c.httpd_send=(r,data,len)=>{c.t+=5;if(!c.io.bodyOutput){c.raw+=data.slice(0,len);return len;}
 const n=c.sendSteps.length?c.sendSteps.shift():len;if(n<=0)return n;c.body.push(...data.subarray(0,n));return n;};
 c.setFailure=r=>c.serverFailure=r;c.sampleHttp=()=>{};c.heap_caps_get_free_size=()=>60000;c.heap_caps_get_largest_free_block=()=>40000;c.uxTaskGetStackHighWaterMark=()=>1500;
 let b=body(http,'esp_err_t download(httpd_req_t* req, SessionIo* io, uint32_t number, bool current) {')
 .replace(/port(?:ENTER|EXIT)_CRITICAL\(&mux\);/g,'').replace(/diagreader::/g,'reader.').replace(/diagtransfer::/g,'transfer.').replace(/diag::/g,'diag.')
 .replace(/diagtransfer::Result result;/g,'let result={};').replace(/transfer.Result result;/g,'let result={};')
 .replace(/const (?:auto|bool|uint64_t|size_t|int) /g,'const ').replace(/\b(?:uint32_t|bool|size_t) (\w+)=/g,'let $1=')
 .replace(/\(unsigned long(?: long)?\)/g,'').replace(/unsigned\(/g,'Number(').replace(/size_t\(/g,'Number(')
 .replace(/io->/g,'io.').replace(/reinterpret_cast<const char\*>\(state.data\+offset\)/g,'state.data.subarray(offset)')
 .replace(/state.data\+offset/g,'state.data.subarray(offset)').replace(/page\+offset/g,'page.value.slice(offset)')
 .replace(/nullptr/g,'null').replace(/"\s*\n\s*"/g,'"+"');
 b=b.replace('char dated[32]; diagtime::prefix(result.opened,dated,sizeof(dated));', "const dated='start-unknown';");
 b=b.replace('char name[24]; reader.nameFor({0,number,current},name,sizeof(name));', "let name=current?'current.log':`archive-${String(number).padStart(8,'0')}.log`;")
 .replace('name[strlen(name)-4]=0;', 'name=name.slice(0,-4);');
 // C++ default member initializers.
 b=b.replace('result.crc=crc^0xffffffff','result.crc=(crc^0xffffffff)>>>0');
 b=b.replace('let result={};','let result={bytes:0,expected:0,firstBody:0,lastBody:0,maxGap:0,cancelledAt:0,closedAt:0,releasedAt:0,writerBytes:0,writerCrc:0,crcCheck:"unavailable"};');
 vm.createContext(c);vm.runInContext(`function compareWriter(result,closed){${adapt(body(source,'void compareWriter('))}}`,c);vm.runInContext(`function download(req,io,number,current=false){${b}}`,c);return c;
}
test('actual HTTP handler sends full fixed-length response and exact partial-send CRC',()=>{
 const c=downloadContext();c.sendSteps=[3,2,4];assert.equal(c.download(c.req,c.io,7),-1);
 assert.match(c.raw,/HTTP\/1.1 200 OK\r\n/);assert.match(c.raw,/Content-Length: 9\r\n/);
 assert.match(c.raw,/filename="start-unknown_108-1-archive-00000007-9.log"/);
 assert.equal(Buffer.from(c.body).toString(),'123456789');assert.equal(c.released.bytes,9);assert.equal(c.released.crc>>>0,0xcbf43926);assert.equal(c.released.result,'ok');
 assert.deepEqual(c.records.map(x=>x.name),['HTTP_GET_BEGIN','HTTP_GET_META','HTTP_GET_END','HTTP_GET_CLOSE','HTTP_GET_MEM']);
 assert.equal(c.shared.userActivity,1000);
});
test('actual handler partial-body failure records only accepted prefix and never appends error',()=>{
 const c=downloadContext();c.sendSteps=[3,-1];c.download(c.req,c.io,7);
 assert.equal(Buffer.from(c.body).toString(),'123');assert.equal(c.released.bytes,3);
 assert.equal(c.released.crc>>>0,0x884863d2);assert.equal(c.reply,null);assert.equal(c.released.result,'body_send_failed');
 assert.match(c.records.find(x=>x.name==='HTTP_GET_END').text,/bytes=3/);
});
test('actual handler Range and If-Range get full 200 with arrival recorded',()=>{
 const c=downloadContext();c.range=c.ifRange=true;c.download(c.req,c.io,7);
 assert.match(c.records[0].text,/range=1 if_range=1/);assert.match(c.raw,/200 OK/);assert.doesNotMatch(c.raw,/206|Content-Range/);assert.equal(c.body.length,9);
});
test('actual handler absent managed archive returns 404 without body metadata',()=>{
 const c=downloadContext();c.metadata=false;c.closed=true;c.reason='not_found';c.download(c.req,c.io,7);
 assert.equal(c.statusCode,'404 Not Found');assert.equal(c.body.length,0);assert.equal(c.released.result,'not_found');assert(!c.records.some(x=>x.name==='HTTP_GET_META'));
});
test('actual handler busy refusal does not reserve or overwrite last result',()=>{
 const c=downloadContext();c.busy=true;c.download(c.req,c.io,7);assert.equal(c.reservationAttempts,0);assert.equal(c.released,null);assert.equal(c.statusCode,'503 Service Unavailable');assert.equal(c.records.length,0);
});
test('actual handler empty archive still sends 200 and Content-Length zero',()=>{
 const c=downloadContext(Buffer.alloc(0));c.closed=true;c.download(c.req,c.io,7);assert.match(c.raw,/200 OK/);assert.match(c.raw,/Content-Length: 0/);assert.equal(c.released.result,'ok');assert.equal(c.released.crc,0);
});

test('equal-length writer CRC divergence is explicit in END and changes successful result',()=>{
 const c=downloadContext();c.writerCrcOverride=0x12345678;c.download(c.req,c.io,7);
 assert.equal(c.released.crc>>>0,0xcbf43926);assert.equal(c.released.writerCrc,0x12345678);
 assert.equal(c.released.result,'crc_mismatch');assert.equal(c.released.crcCheck,'mismatch');
 const end=c.records.find(x=>x.name==='HTTP_GET_END').text;
 assert.match(end,/crc32=CBF43926/);assert.match(end,/writer_bytes=9 writer_crc32=12345678 crc_check=mismatch/);
});
test('matching writer CRC is carried independently and preserves accepted-prefix CRC',()=>{
 const c=downloadContext();c.download(c.req,c.io,7);assert.equal(c.released.crcCheck,'match');assert.equal(c.released.result,'ok');
 assert.match(c.records.find(x=>x.name==='HTTP_GET_END').text,/writer_bytes=9 writer_crc32=CBF43926 crc_check=match/);
});
test('cancellation with unequal prefix coverage never reports corruption',()=>{
 const c=downloadContext();c.sendSteps=[3,-1];c.writerBytesOverride=0;c.writerCrcOverride=0;c.download(c.req,c.io,7);
 assert.equal(c.released.bytes,3);assert.equal(c.released.crc>>>0,0x884863d2);assert.equal(c.released.crcCheck,'prefix_diff');assert.equal(c.released.result,'body_send_failed');
});
test('late writer close updates retained comparison without overwriting HTTP CRC',()=>{
 const c=context();const id=begin(c);c.release(id,{id,bytes:3,crc:0x12345678,result:'writer_close_pending',releasedAt:1000});
 c.r.sentBytes=3;c.r.crc=0xabcdef01;c.close('aborted');assert.equal(c.lastResult.crc,0x12345678);
 assert.equal(c.lastResult.writerBytes,3);assert.equal(c.lastResult.crcCheck,'mismatch');assert.equal(c.lastResult.result,'aborted');
});
test('unclosed or different-generation writer snapshot cannot claim comparison',()=>{
 const c=context();const r={id:2,bytes:0,crc:0,result:'writer_close_pending'};
 c.compareWriter(r,{id:2,closed:false});assert.equal(r.crcCheck,'unavailable');
 c.compareWriter(r,{id:1,closed:true,writerBytes:0,writerCrc:0});assert.equal(r.crcCheck,'unavailable');
});
test('dual-CRC END fits the fixed log field capacity at maximum numeric widths',()=>{
 const capacity=Number(sd.split('char fields[')[1].split(']')[0]);
 const format='id=%llu expected=%llu bytes='+http.split('snprintf(page,PAGE_CAP,"id=%llu expected=%llu bytes=')[1].split('"')[0];
 const worst=format.replace(/%llu/g,'18446744073709551615').replace(/%08lX/g,'FFFFFFFF').replace(/%s/, 'writer_close_pending').replace(/%s/,'unavailable');
 assert(worst.length<capacity,`${worst.length} >= ${capacity}`);
});

test('current reservation dispatches distinctly including queued shutdown',()=>{
 const c=context();const id=c.request(0,1000,true);assert.equal(c.pending.request,c.Request.HttpCurrent);
 c.writerOffline();c.stop(c.pending);assert(c.box.closed);assert.equal(c.box.result,'shutdown');
 c.release(id,{id,result:'shutdown',releasedAt:1100});c.offlineTick();assert(!c.busy());
 assert.match(usb,/accepted.request == Request::HttpCurrent/);
});
test('current cleanup timing survives release-first and late writer completion',()=>{
 const c=context();const id=c.request(0,1000,true);c.accept(c.pending);
 c.release(id,{id,bytes:0,crc:0,result:'writer_close_pending',releasedAt:1050});
 c.r.pausedAt=1001;c.r.readerClosedAt=1100;c.r.resumedAt=1101;c.close('aborted');
 assert.equal(c.lastResult.pausedAt,1001);assert.equal(c.lastResult.resumedAt,1101);
 assert.equal(c.lastResult.readerClosedAt,1100);assert.equal(c.lastResult.appends,'resumed');
});
test('actual current response uses frozen size and reports measured cleanup separately',()=>{
 const c=downloadContext();c.pausedAt=1001;c.resumedAt=1050;
 c.download(c.req,c.io,0,true);assert.match(c.raw,/filename="start-unknown_108-1-current-9.log"/);
 assert.match(c.raw,/Content-Length: 9/);assert.match(c.records[0].text,/file=current.log/);
 const close=c.records.find(x=>x.name==='HTTP_GET_CLOSE').text;
 assert.match(close,/pause_ms=1001/);assert.match(close,/resume_ms=1050 paused_ms=49 appends=resumed/);
 assert.equal(c.released.crcCheck,'match');
});
test('resume failure and unavailable close never claim successful append resumption',()=>{
 const c=context();const r={id:1,bytes:0,crc:0,result:'logger_failed',appends:'unknown'};
 c.compareWriter(r,{id:1,closed:false});assert.equal(r.appends,'unknown');
 c.compareWriter(r,{id:1,closed:true,pausedAt:1000,resumedAt:0,readerClosedAt:1100,writerBytes:0,writerCrc:0});
 assert.equal(r.appends,'resume_failed');assert.equal(r.result,'logger_failed');
});
test('current route is exact and methods are refused before reserving',()=>{
 const b=body(http,'esp_err_t handle(');
 assert.match(b,/!strcmp\(req->uri,"\/f\/current"\)\) return download\(req,io,0,true\)/);
 assert(b.indexOf('req->method != HTTP_GET')<b.indexOf('"/f/current"'));
 assert.match(body(http,'bool formatPage('),/href='\/f\/current'/);
});
test('cleanup record fits field capacity with maximum timestamps',()=>{
 const capacity=Number(sd.split('char fields[')[1].split(']')[0]);
 const format='id=%llu pause_ms='+http.split('snprintf(page,PAGE_CAP,"id=%llu pause_ms=')[1].split('"')[0];
 const worst=format.replace(/%llu/g,'18446744073709551615').replace('%s','resume_failed');
 assert(worst.length<capacity);
});
test('measured lifecycle configuration remains pinned',()=>{
 assert.match(http,/config.stack_size = 6144;/);assert.match(http,/config.task_caps = MALLOC_CAP_INTERNAL \| MALLOC_CAP_8BIT;/);
 assert.match(http,/config.max_open_sockets = CLIENTS;/);assert.match(http,/CLIENTS = 3/);
 assert.match(http,/config.lru_purge_enable = false;/);
});
console.log(`${count} HTTP transfer checks passed; source simulations only.`);
