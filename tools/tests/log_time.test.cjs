// Actual pure C++ parser/reader bodies executed via syntax adaptation and mocked SD.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict'),cp=require('child_process');
const source=fs.readFileSync('src/diagnostics/diagnostics_log_time.cpp','utf8').replace(/\r\n/g,'\n');
const header=fs.readFileSync('src/diagnostics/diagnostics_log_time.h','utf8').replace(/\r\n/g,'\n');
const sd=fs.readFileSync('src/diagnostics/sd_diagnostics.cpp','utf8').replace(/\r\n/g,'\n');
const inv=fs.readFileSync('src/diagnostics/diagnostics_inventory.cpp','utf8').replace(/\r\n/g,'\n');
function body(s,sig){let a=s.indexOf(sig);assert(a>=0,sig);a=s.indexOf('{',a);let j=a+1,d=1;while(d){if(s[j]==='{')d++;if(s[j]==='}')d--;j++;}return s.slice(a+1,j-1);}
const Quality=Object.fromEntries(['Pending','Unknown','Approx','Synced','Test','Malformed','Inconsistent','Io'].map((s,i)=>[s,i]));
const fresh=()=>({year:0,ms:0,offset:0,month:0,day:0,hour:0,minute:0,second:0,quality:Quality.Pending});
function common(s){return s.replace(/Quality::/g,'Quality.').replace(/\bconst (?:size_t|int|bool|ssize_t|uint64_t|Record) /g,'const ').replace(/\b(?:size_t|int) (\w+)=/g,'let $1=').replace(/\bnullptr\b/g,'null').replace(/\bsize_t\(/g,'Number(').replace(/ssize_t\(/g,'Number(').replace(/off_t\(/g,'Number(').replace(/uint64_t\(/g,'Number(').replace(/unsigned\(/g,'Number(');}
function parser(){
 const c={Quality,fresh,UINT64_MAX:(1n<<64n)-1n,strlen:s=>s.length,strcmp:(a,b)=>a===b?0:1,
 memcmp:(a,b,n)=>a.slice(0,n)===b.slice(0,n)?0:1};vm.createContext(c);
 let b=common(body(source,'bool validDate(')).replace('static const uint8_t days[]={31,28,31,30,31,30,31,31,30,31,30,31};','const days=[31,28,31,30,31,30,31,31,30,31,30,31];');
 vm.runInContext(`function validDate(t){${b}}`,c);
 b=common(body(source,'int digits(')).replace("value=value*10+p[i]-'0'",'value=value*10+Number(p[i])');vm.runInContext(`function digits(p,n){${b}}`,c);
 b=body(source,'bool decimal(').replace('value=0;','out.value=0n;').replace(/\bvalue\b/g,'VALUE'); // output references mapped below
 b=b.replace('out.VALUE','out.value').replace(/\bVALUE\b/g,'out.value').replace('if(!*p)','if(!p.length)').replace('for(;*p;++p)','for(let i=0;i<p.length;++i)').replace(/\*p/g,'p[i]').replace("const unsigned d=p[i]-'0'",'const d=BigInt(p[i])').replace('/10','/10n').replace('value*10+d','value*10n+d');
 vm.runInContext(`function decimal(p,out){${b}}`,c);
 // Execute token's pointer algorithm with explicit string cursor and output cell.
 b=common(body(source,'bool token(')).replace('end-p','end-p.at').replace('strncmp(p,key,n)','strncmp(p.text.slice(p.at),key,n)').replace('p+=n','p.at+=n').replace('p<end','p.at<end').replace(/\*p\+\+/g,'p.text[p.at++]').replace(/\*p/g,'p.text[p.at]').replace(/\+\+p/g,'++p.at').replace('p==end','p.at==end').replace('value[used++]=p.text[p.at++]','value.text+=p.text[p.at++]; ++used').replace('value[used]=0;','');
 c.strncmp=(a,b,n)=>a.slice(0,n)===b.slice(0,n)?0:1;
 vm.runInContext(`function rawToken(p,end,key,value,cap){${b}}`,c);
 c.token=(p,end,key,value,cap)=>{value.text='';return c.rawToken(p,end,key,value,cap);};
 c.readDecimal=(text,obj,key)=>{const out={value:0n};const ok=c.decimal(text,out);obj[key]=out.value;return ok;};
 c.diagnosticsHeaderValid=(line)=>{ // Actual unchanged header body installed next.
  const out={value:0n};return c.recovery(line,out);
 };
 // Recovery acceptance body is pinned byte-for-byte, then run with real token/decimal helpers.
 const baseline=cp.execFileSync('git',['show','229a344:src/diagnostics/sd_diagnostics.cpp'],{encoding:'utf8'});
 assert.equal(body(sd,'bool headerGeneration('),body(baseline,'bool headerGeneration('));
 b=body(sd,'bool headerGeneration(').replace('const char* cursor = header;','const cursor={text:header,at:0};').replace('char token[64];','const token={text:""};').replace('uint64_t number;','const number={value:0n};')
 .replace(/headerToken\(cursor, /g,'tokenRead(cursor, ').replace(/sizeof\(token\)/g,'64').replace(/\b(?:strcmp|strlen)\(token/g,m=>m.replace('token','token.text')).replace(/token\[/g,'token.text[').replace(/decimal\(token,([^,]+),number\)/g,'boundedDecimal(token.text,$1,number)').replace(/!number\b/g,'!number.value').replace('result = static_cast<uint32_t>(number);','result.value=number.value;');
 c.MAX_GENERATION=99999999n;c.tokenRead=(cur,key,out,cap)=>c.token(cur,cur.text.length,key,out,cap);c.boundedDecimal=(text,max,out)=>c.decimal(text,out)&&out.value<=max;
 vm.runInContext(`function recovery(header,result){${b}}`,c);
 b=common(body(source,'Record parse(')).replace('Record r;','const r={time:fresh(),boot:0n,sequence:0n,valid:false};').replace('const char* p=line; const char* end=line+length; char value[48]; uint64_t number=0;','const p={text:line,at:0}; const end=length; const value={text:""}; const number={value:0n};')
 .replace(/sizeof\(value\)/g,'48').replace(/strcmp\(value,/g,'strcmp(value.text,').replace(/strlen\(value\)/g,'strlen(value.text)').replace(/value\[/g,'value.text[').replace(/digits\(value\+(\d+),/g,'digits(value.text.slice($1),').replace(/digits\(value,/g,'digits(value.text,')
 .replace('Time t;','const t=fresh();').replace(/decimal\(value,r\.(\w+)\)/g,'readDecimal(value.text,r,"$1")').replace('decimal(value,number)','decimal(value.text,number)')
 .replace("for(const char* c=value;*c;++c) if(!((*c>='A'&&*c<='Z') || (*c>='0'&&*c<='9') || *c=='_')) return r;", "for(const ch of value.text) if(!((ch>='A'&&ch<='Z') || (ch>='0'&&ch<='9') || ch=='_')) return r;")
 .replace('uint32_t generation=0;','let generation=0;').replace('constexpr char test[]=','const test=').replace(/sizeof\(test\)-1/g,'test.length').replace('line+length-(test.length)','line.slice(length-test.length)');
 c.unavailable=q=>({...fresh(),quality:q});vm.runInContext(`function parse(line,length,header){${b}}`,c);
 c.snprintf=(out,cap,fmt,...args)=>{let i=0;out.text=fmt.replace(/%04u|%02u|%02d|%c|%s/g,m=>{let v=String(args[i++]);return m==='%04u'?v.padStart(4,'0'):m==='%02u'||m==='%02d'?v.padStart(2,'0'):v;});assert(out.text.length<cap);};
 for(const name of ['format','prefix']){b=common(body(source,`void ${name}(`)).replace('const char* why=','let why=');vm.runInContext(`function ${name}(t,out,capacity){${b}}`,c);}
 return c;
}
let count=0;function test(name,fn){fn();count++;console.log('PASS '+name);}
const row=(local='2026-09-23T08:00:01.123-04:00',quality='synced',seq=1,boot=116,event='FILE_OPEN',fields='format=1 generation=22')=>`local=${local} time=${quality} seq=${seq} boot=${boot} up_ms=20 level=INFO event=${event} ${fields}\n`;
const p=parser();const parse=(s,h=true)=>p.parse(s,s.length,h);
test('synced endpoint parses and filename uses local offset with seconds truncated',()=>{const r=parse(row());assert(r.valid);assert.equal(r.time.quality,Quality.Synced);const out={};p.prefix(r.time,out,64);assert.equal(out.text,'2026-09-23T080001-0400');});
test('strict metadata calendar rejects dates without changing recovery acceptance',()=>{for(const date of ['2026-02-29T08:00:01.123-04:00','2024-13-23T08:00:01.123-04:00','2024-01-00T08:00:01.123-04:00','2024-01-01T24:00:01.123-04:00']){const s=row(date);assert(p.recovery(s,{}));assert(!parse(s).valid);}assert(parse(row('2024-02-29T23:59:59.999+14:00')).valid);});
test('offset bounds and grammar reject invalid numeric and separator cases',()=>{for(const date of ['2026-09-23T08:00:01.123+14:30','2026-09-23T08:00:01.123+05:20','2026-09-23T08:00:60.123-04:00','2026-09-23T08:00:01.12x-04:00','2026-09-23X08:00:01.123-04:00'])assert(!parse(row(date)).valid);assert(parse(row('2026-09-23T08:00:01.123+05:45')).valid);});
test('approx unknown and test clocks do not create dated filenames',()=>{for(const s of [row('unknown','unknown'),row(undefined,'approx'),row(undefined,'approx',1,116,'FILE_OPEN','format=1 generation=22 clock_source=test')]){const r=parse(s);assert(r.valid);const out={};p.prefix(r.time,out,64);assert.equal(out.text,'start-unknown');}assert.equal(parse(row(undefined,'approx',1,116,'FILE_OPEN','format=1 generation=22 clock_source=test')).time.quality,Quality.Test);});
test('embedded test tokens are not interpreted as final clock provenance',()=>{assert.equal(parse(row(undefined,'approx',1,116,'FILE_OPEN','format=1 generation=22 clock_source=test extra=1')).time.quality,Quality.Approx);});
test('incomplete oversized and malformed common fields have no endpoint',()=>{assert(!parse(row().trimEnd()).valid);assert(!parse(row().replace('seq=1','seq=18446744073709551616')).valid);assert(!parse(row().replace('level=INFO','level=GARBAGE')).valid);assert(!parse(row()+ 'x'.repeat(1024)).valid);assert(!parse(row().replace('FILE_OPEN','HEALTH')).valid);assert(parse(row(undefined,'synced',2,116,'HEALTH','ok=1'),false).valid);});
test('boot and sequence preserve full uint64 precision',()=>{const r=parse(row(undefined,'synced','18446744073709551615','18446744073709551615'));assert(r.valid);assert.equal(r.boot,(1n<<64n)-1n);});

function ioContext(content){
 const c=parser();Object.assign(c,{content,position:0,scratch:'',opens:0,closes:0,reads:0,readFail:0,seekWrong:false,closeFail:false,O_RDONLY:0,SEEK_SET:0,currentEnds:{opened:fresh(),last:fresh(),fragment:false},currentSize:0,currentGeneration:0});
 c.open=()=>{c.opens++;c.position=0;return 7;};c.close=()=>{c.closes++;return c.closeFail?-1:0;};
 c.fstat=(fd,st)=>{st.st_size=content.length;return 0;};
 c.read=(fd,buf,n)=>{c.reads++;if(c.readFail===c.reads)return -1;const bytes=c.content.slice(c.position,c.position+n);c.position+=bytes.length;c.scratch=bytes;return bytes.length;};
 c.lseek=(fd,at)=>{if(c.seekWrong)return 1;c.position=at;return at;};c.memchr=(text,char,n)=>{const at=text.slice(0,n).indexOf(char);return at<0?null:at;};
 for(const [name,args,sig] of [['archive','path,expected','Ends archive('],['readStart','reader,out','const char* readStart('],['restoreCurrent','header,length,size,generation','void restoreCurrent('],['written','line,length,first,size,generation','void written(']]){
  let b=common(body(source,sig)).replace(/Ends result;/g,'const result={opened:fresh(),last:fresh(),fragment:false};').replace('struct stat st{};','const st={};')
   .replace(/const char\* (nl|end)=static_cast<const char\*>\(memchr\(([^;]+)\)\);/g,'const $1=memchr($2);')
   .replace(/scratch\[(n|headSize|tailSize)\]=0;/g,'').replace(/(nl|end)-scratch/g,'$1').replace(/scratch\+begin/g,'scratch.slice(begin)')
   .replace(/currentEnds=\{\}/g,'currentEnds={opened:fresh(),last:fresh(),fragment:false}').replace(/::close/g,'close').replace(/&st/g,'st');
  if(name==='readStart') b=b.replace(/\bout=/g,'out.value=');
  vm.runInContext(`function ${name}(${args}){${b}}`,c);
 }
 // scratch existence refers to allocated storage, even before read in C++.
 c.scratch='allocated';return c;
}
test('archive header-only equality is valid; larger tail identities cross boots',()=>{
 let text=row();let c=ioContext(text);let ends=c.archive('archive',text.length);assert.equal(ends.last.quality,Quality.Synced);assert.equal(c.opens,c.closes);
 text+=row(undefined,'synced',1,117,'BOOT','ok=1');c=ioContext(text);ends=c.archive('archive',text.length);assert.equal(ends.last.quality,Quality.Synced);assert.equal(c.opens,c.closes);
});
test('stale different-record pairs and volatile boot fail tail plausibility',()=>{
 for(const tail of [row(undefined,'synced',1,116,'HEALTH','ok=1'),row(undefined,'synced',999,115,'HEALTH','ok=1'),row(undefined,'synced',2,0,'HEALTH','ok=1')]){const text=row()+tail,c=ioContext(text);assert.equal(c.archive('archive',text.length).last.quality,Quality.Inconsistent);assert.equal(c.opens,c.closes);}
});
test('bounded tail ignores incomplete fragment but never searches past a malformed final record',()=>{
 const text=row()+row(undefined,'synced',2,116,'HEALTH','ok=1')+'partial';let c=ioContext(text);let e=c.archive('archive',text.length);assert(e.fragment);assert.equal(e.last.quality,Quality.Synced);
 const bad=row()+'malformed\n';c=ioContext(bad);e=c.archive('archive',bad.length);assert.equal(e.last.quality,Quality.Malformed);
});
test('large archive uses only head/tail windows and preserves test-clock label',()=>{
 const text=row()+'x'.repeat(5000)+'\n'+row(undefined,'approx',2,116,'HEALTH','clock_source=test');const c=ioContext(text);const e=c.archive('archive',text.length);assert.equal(e.last.quality,Quality.Test);assert.equal(c.reads,2);assert.equal(c.opens,c.closes);
});
test('archive read size mismatch seek read and close failures close exactly once',()=>{
 for(const failure of ['size','seek','read1','read2','close']){const text=row();const c=ioContext(text);c.seekWrong=failure==='seek';c.readFail=failure==='read1'?1:failure==='read2'?2:0;c.closeFail=failure==='close';const e=c.archive('archive',text.length+(failure==='size'?1:0));assert.equal(e.last.quality,Quality.Io);assert.equal(c.closes,1);}
});
test('download head read restores position even on malformed or failed read and rejects wrong restore',()=>{
 for(const text of [row(),'bad\n']){const c=ioContext(text);c.position=0;const out={};assert.equal(c.readStart(7,out),null);assert.equal(c.position,0);assert.equal(out.value.quality,text==='bad\n'?Quality.Malformed:Quality.Synced);}
 const c=ioContext(row());c.readFail=1;assert.equal(c.readStart(7,{}),'read_failed');assert.equal(c.position,0);
 c.readFail=0;c.seekWrong=true;assert.equal(c.readStart(7,{}),'metadata_seek');
});
test('writer endpoints survive boot append and reset on rotation and empty recovery',()=>{
 const c=ioContext('');const opened=row();c.restoreCurrent(opened,opened.length,opened.length,22);assert.equal(c.currentEnds.opened.quality,Quality.Synced);
 const boot=row('2026-09-24T08:00:01.123-04:00','synced',1,117,'BOOT','ok=1');c.written(boot,boot.length,false,500,22);assert.equal(c.currentEnds.opened.day,23);assert.equal(c.currentEnds.last.day,24);
 const unknown=row('unknown','unknown',3,117);c.written(unknown,unknown.length,true,unknown.length,23);assert.equal(c.currentEnds.opened.quality,Quality.Unknown);assert.equal(c.currentGeneration,23);
 const recovery=row(undefined,'synced',4,117,'FILE_OPEN','format=1 generation=24 reason=empty_recovery');c.written(recovery,recovery.length,true,recovery.length,24);assert.equal(c.currentEnds.opened.quality,Quality.Synced);
});
test('writer metadata updates only after successful raw write; header restore precedes append open',()=>{
 for(const sig of ['bool direct(','bool writeRecord(const diag::Stamp& when, const char* level, const char* event, const char* fields) {']){const b=body(sd,sig);assert.match(b,/if \(ok\) diagtime::written/);assert(b.indexOf('rawWrite(line, length)')<b.indexOf('diagtime::written'));}
 const b=body(sd,'bool openStorage()');assert(b.indexOf('diagtime::restoreCurrent')<b.indexOf('fd = openCurrentForWrite(O_WRONLY | O_APPEND)'));
 assert.doesNotMatch(body(inv,'void writerTick('),/open\(CURRENT|read\(|lseek\(/);
});

function inventoryContext(numbers){
 const makeEntry=()=>({file:{size:0,number:0,current:false},times:{opened:fresh(),last:fresh(),fragment:false},retryAt:0});
 const c={wanted:true,storageReady:true,changed:1,quiet:false,quietGeneration:0,activeGeneration:1,valid:false,stale:true,error:'pending',published:-1,
 slots:[{entries:Array.from({length:256},makeEntry),pins:0},{entries:Array.from({length:256},makeEntry),pins:0}],directory:null,staging:-1,seen:0,used:0,nextScan:0,scanChanged:0,metadataAt:0,reads:0,metadataPhase:false,pendingRows:false,t:1000,busy:false,numbers,at:0,readsThisTick:0,totalReads:0,failArchive:false,Quality,makeEntry};
 c.nowMs=()=>c.t;c.reader={busy:()=>c.busy,MAX_ENTRIES:256,archiveName:(s,out)=>/^archive-\d{8}\.log$/.test(s)};
 c.strcmp=(a,b)=>a===b?0:1;c.opendir=()=>{c.at=0;return {};};c.closedir=()=>0;c.readdir=()=>c.at<c.numbers.length?{d_name:c.numbers[c.at++]}:null;
 c.stat=(path,info)=>{info.st_size=100;info.st_mode=1;return 0;};c.S_ISREG=x=>x;c.snprintf=(out,cap,fmt,...args)=>{out.value=fmt.replace(/%s|%08lu/g,()=>String(args.shift()));};
 c.current=()=>({size:321,times:{opened:{quality:Quality.Unknown},last:{quality:Quality.Synced}}});
 c.time={Quality,archive:()=>{c.readsThisTick++;c.totalReads++;return {opened:{quality:c.failArchive?Quality.Io:Quality.Synced},last:{quality:c.failArchive?Quality.Io:Quality.Synced}};}};
 let end=body(inv,'void endScan()').replace(/closedir/g,'closedir');
 let b=body(inv,'void writerTick(').replace(/port(?:ENTER|EXIT)_CRITICAL\(&mux\);/g,'');
 b=common(b).replace(/diagreader::/g,'reader.').replace(/diagtime::/g,'time.').replace(/\bconst (?:Slot|Entry)&? /g,'const ').replace(/Entry& /g,'const ')
 .replace('uint64_t size=0; uint32_t generation=0;','').replace('entry.times=time.current(size,generation); entry.file.size=size;','const meta=current(); entry.times=meta.times; entry.file.size=meta.size;')
 .replace('const Entry value=','const value=').replace(/unsigned batch/g,'let batch').replace('dirent* item','const item').replace(/item->/g,'item.')
 .replace(/uint32_t number\s*=\s*0;/g,'let number=0;').replace('reader.archiveName(item.d_name,number)','((number=Number(item.d_name.slice(8,16))),reader.archiveName(item.d_name))')
 .replace(/char path\[64\];/g,'let path={};').replace(/sizeof\(path\)/g,'64').replace(/\(unsigned long\)/g,'').replace('struct stat info{};','const info={};').replace(/&info/g,'info')
 .replace('Entry entry{};','const entry=makeEntry();').replace('entry.file={Number(info.st_size),number,current};','entry.file={size:Number(info.st_size),number,current};');
 vm.createContext(c);vm.runInContext(`function endScan(){${end.replace(/nullptr/g,'null')}} function fail(reason){endScan();error=reason;nextScan=nowMs()+5000;} function tick(storageReady){${b}}`,c);
 c.turn=()=>{c.readsThisTick=0;c.tick(true);assert(c.readsThisTick<=1);c.t+=20;};return c;
}
test('cold inventory publishes after eight reads and carries progress to remaining entries',()=>{
 const c=inventoryContext(['current.log',...Array.from({length:12},(_,i)=>`archive-${String(12-i).padStart(8,'0')}.log`)]);
 for(let i=0;i<80 && c.totalReads<12;i++)c.turn();
 for(let i=0;i<10;i++)c.turn();assert.equal(c.totalReads,12);assert(c.valid);
 const entries=c.slots[c.published].entries.slice(0,c.slots[c.published].count);
 assert.deepEqual(Array.from(entries,x=>x.file.number),[1,2,3,4,5,6,7,8,9,10,11,12,0]);assert(entries.at(-1).file.current);assert.equal(entries.at(-1).file.size,321);
 c.t+=5000;for(let i=0;i<15;i++)c.turn();assert.equal(c.totalReads,12);
});
test('metadata preempts before next archive and preserves immutable published rows',()=>{
 const c=inventoryContext(['archive-00000001.log']);for(let i=0;i<8;i++)c.turn();assert(c.valid);const before=JSON.stringify(c.slots[c.published]);
 c.t+=5000;c.slots[1-c.published].pins=1;c.turn();assert.equal(c.directory,null);assert.equal(JSON.stringify(c.slots[c.published]),before);
 c.slots[1-c.published].pins=0;c.turn();c.busy=true;c.turn();assert.equal(c.directory,null);assert(!c.metadataPhase);assert.equal(JSON.stringify(c.slots[c.published]),before);
});
test('failed archive reads have a retry deadline and cannot monopolize immediate pending cycles',()=>{
 const c=inventoryContext(Array.from({length:12},(_,i)=>`archive-${String(i+1).padStart(8,'0')}.log`));c.failArchive=true;
 for(let i=0;i<100;i++)c.turn();assert.equal(c.totalReads,12);assert(c.valid);
});

test('revision invalidation and stop discard unfinished staging without archive descriptors',()=>{
 const c=inventoryContext(['archive-00000001.log','archive-00000002.log']);c.turn();assert(c.metadataPhase);c.changed++;c.turn();
 assert.equal(c.scanChanged,c.changed);assert.equal(c.totalReads,0);c.wanted=false;c.turn();assert.equal(c.staging,-1);assert.equal(c.directory,null);assert(c.quiet);
});
test('metadata storage and transfer growth are explicitly bounded',()=>{
 assert.match(header,/static_assert\(sizeof\(Time\)<=16/);
 const ih=fs.readFileSync('src/diagnostics/diagnostics_inventory.h','utf8');assert.match(ih,/static_assert\(sizeof\(Entry\)<=64/);
 assert.match(source,/heap_caps_malloc\(2049,MALLOC_CAP_SPIRAM \| MALLOC_CAP_8BIT\)/);
 const reader=fs.readFileSync('src/diagnostics/diagnostics_reader.cpp','utf8');assert(reader.indexOf('diagtime::readStart(reader,state.opened)')>reader.indexOf('fstat(reader, &st)'));
 const transfer=fs.readFileSync('src/diagnostics/diagnostics_http_transfer.cpp','utf8');assert.match(transfer,/box.size=r.fileSize; box.opened=r.opened; box.metadata=true/);
});
console.log(`${count} log-time checks passed; actual parser source adapted, no firmware build.`);
