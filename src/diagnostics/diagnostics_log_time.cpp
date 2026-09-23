#include "diagnostics_log_time.h"
#include <esp_heap_caps.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
namespace diagtime {
namespace {
char* scratch=nullptr;
Ends currentEnds;
uint64_t currentSize=0;
uint32_t currentGeneration=0;
Time unavailable(Quality q) { Time t; t.quality=q; return t; }
bool token(const char*& p, const char* end, const char* key, char* value, size_t cap) {
  const size_t n=strlen(key);
  if (size_t(end-p)<n || strncmp(p,key,n)) return false;
  p+=n; size_t used=0;
  while(p<end && *p!=' ' && *p!='\n') { if(used+1>=cap) return false; value[used++]=*p++; }
  value[used]=0;
  if(!used || p==end) return false;
  if(*p==' ') ++p;
  return true;
}
bool decimal(const char* p, uint64_t& value) {
  value=0; if(!*p) return false;
  for(;*p;++p) { if(*p<'0'||*p>'9') return false; const unsigned d=*p-'0';
    if(value>(UINT64_MAX-d)/10) return false; value=value*10+d; }
  return true;
}
int digits(const char* p, size_t n) {
  int value=0;
  for(size_t i=0;i<n;++i) { if(p[i]<'0'||p[i]>'9') return -1; value=value*10+p[i]-'0'; }
  return value;
}
}
bool validDate(const Time& t) {
  if(t.year<2024 || t.year>2099 || t.month<1 || t.month>12 || t.hour>23 ||
     t.minute>59 || t.second>59 || t.ms>999 || t.offset < -840 || t.offset > 840) return false;
  const int minutes=(t.offset<0 ? -t.offset : t.offset)%60;
  if(minutes!=0 && minutes!=30 && minutes!=45) return false;
  static const uint8_t days[]={31,28,31,30,31,30,31,31,30,31,30,31};
  const int maxDay=days[t.month-1]+(t.month==2 && t.year%4==0);
  return t.day>=1 && t.day<=maxDay;
}
Record parse(const char* line, size_t length, bool header) {
  Record r; r.time=unavailable(Quality::Malformed);
  if(!length || length>=1024 || line[length-1]!='\n') return r;
  const char* p=line; const char* end=line+length; char value[48]; uint64_t number=0;
  if(!token(p,end,"local=",value,sizeof(value))) return r;
  Time t;
  if(!strcmp(value,"unknown")) t.quality=Quality::Unknown;
  else {
    if(strlen(value)!=29 || value[4]!='-' || value[7]!='-' || value[10]!='T' ||
       value[13]!=':' || value[16]!=':' || value[19]!='.' || value[26]!=':' ||
       (value[23]!='+' && value[23]!='-')) return r;
    const int year=digits(value,4), month=digits(value+5,2), day=digits(value+8,2),
      hour=digits(value+11,2), minute=digits(value+14,2), second=digits(value+17,2),
      ms=digits(value+20,3), oh=digits(value+24,2), om=digits(value+27,2);
    if(year<0 || month<0 || day<0 || hour<0 || minute<0 || second<0 || ms<0 ||
       oh<0 || om<0 || om>59) return r;
    t.year=year; t.month=month; t.day=day; t.hour=hour; t.minute=minute; t.second=second;
    t.ms=ms; t.offset=(oh*60+om)*(value[23]=='-' ? -1 : 1);
    if(!validDate(t)) return r;
  }
  if(!token(p,end,"time=",value,sizeof(value))) return r;
  if(!strcmp(value,"unknown")) { if(t.quality!=Quality::Unknown) return r; }
  else if(!strcmp(value,"approx")) { if(t.quality==Quality::Unknown) return r; t.quality=Quality::Approx; }
  else if(!strcmp(value,"synced")) { if(t.quality==Quality::Unknown) return r; t.quality=Quality::Synced; }
  else return r;
  if(!token(p,end,"seq=",value,sizeof(value)) || !decimal(value,r.sequence) || !r.sequence) return r;
  if(!token(p,end,"boot=",value,sizeof(value)) || !decimal(value,r.boot)) return r;
  if(!token(p,end,"up_ms=",value,sizeof(value)) || !decimal(value,number)) return r;
  if(!token(p,end,"level=",value,sizeof(value)) ||
     (strcmp(value,"INFO") && strcmp(value,"WARN") && strcmp(value,"DEBUG") && strcmp(value,"ERROR"))) return r;
  if(!token(p,end,"event=",value,sizeof(value))) return r;
  for(const char* c=value;*c;++c) if(!((*c>='A'&&*c<='Z') || (*c>='0'&&*c<='9') || *c=='_')) return r;
  uint32_t generation=0;
  if(header && !diagnosticsHeaderValid(line,generation)) return r;
  constexpr char test[]=" clock_source=test\n";
  if(length>=sizeof(test)-1 && !memcmp(line+length-(sizeof(test)-1),test,sizeof(test)-1)) t.quality=Quality::Test;
  r.time=t; r.valid=true; return r;
}
void format(const Time& t, char* out, size_t capacity) {
  const char* why="malformed";
  switch(t.quality) {
    case Quality::Pending: why="pending"; break;
    case Quality::Unknown: why="clock unknown"; break;
    case Quality::Test: why="test clock"; break;
    case Quality::Inconsistent: why="inconsistent"; break;
    case Quality::Io: why="read failed"; break;
    default: break;
  }
  if((t.quality!=Quality::Synced && t.quality!=Quality::Approx) || !validDate(t)) {
    snprintf(out,capacity,"unavailable (%s)",why); return;
  }
  const int offset=t.offset<0 ? -t.offset : t.offset;
  snprintf(out,capacity,"%04u-%02u-%02u %02u:%02u:%02u %c%02d:%02d%s",unsigned(t.year),
    unsigned(t.month),unsigned(t.day),unsigned(t.hour),unsigned(t.minute),unsigned(t.second),
    t.offset<0 ? '-' : '+',offset/60,offset%60,t.quality==Quality::Approx ? " approx" : "");
}
void prefix(const Time& t, char* out, size_t capacity) {
  if(t.quality!=Quality::Synced || !validDate(t)) { snprintf(out,capacity,"start-unknown"); return; }
  const int offset=t.offset<0 ? -t.offset : t.offset;
  snprintf(out,capacity,"%04u-%02u-%02uT%02u%02u%02u%c%02d%02d",unsigned(t.year),
    unsigned(t.month),unsigned(t.day),unsigned(t.hour),unsigned(t.minute),unsigned(t.second),
    t.offset<0 ? '-' : '+',offset/60,offset%60);
}
bool allocate() {
  if(scratch) return false;
  scratch=static_cast<char*>(heap_caps_malloc(2049,MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  return scratch!=nullptr;
}
void dispose() { heap_caps_free(scratch); scratch=nullptr; }
const char* readStart(int reader, Time& out) {
  out=unavailable(Quality::Io);
  if(!scratch) return "memory";
  const ssize_t n=read(reader,scratch,1024);
  if(n>=0) {
    scratch[n]=0;
    const char* end=static_cast<const char*>(memchr(scratch,'\n',size_t(n)));
    out=end ? parse(scratch,size_t(end-scratch)+1,true).time : unavailable(Quality::Malformed);
  }
  // A failed or incorrect restore aborts before publishing metadata/HTTP headers.
  if(lseek(reader,0,SEEK_SET)!=0) return "metadata_seek";
  if(n<0) return "read_failed";
  return nullptr;
}
Ends archive(const char* path, uint64_t expected) {
  Ends result; result.opened=result.last=unavailable(Quality::Io);
  if(!scratch) return result;
  const int fd=open(path,O_RDONLY);
  if(fd<0) return result;
  struct stat st{};
  // Single lexical ownership: every successful open reaches exactly this close.
  do {
    if(fstat(fd,&st) || st.st_size<0 || uint64_t(st.st_size)!=expected) break;
    const size_t headSize=expected<1024 ? size_t(expected) : 1024;
    const ssize_t got=read(fd,scratch,headSize);
    if(got!=ssize_t(headSize)) break;
    scratch[headSize]=0;
    const char* nl=static_cast<const char*>(memchr(scratch,'\n',headSize));
    const size_t firstLength=nl ? size_t(nl-scratch)+1 : 0;
    const Record first=parse(scratch,firstLength,true);
    result.opened=first.time;
    const size_t tailSize=expected<2048 ? size_t(expected) : 2048;
    const uint64_t base=expected-tailSize;
    if(lseek(fd,off_t(base),SEEK_SET)!=off_t(base) || read(fd,scratch,tailSize)!=ssize_t(tailSize)) break;
    scratch[tailSize]=0;
    size_t end=tailSize;
    result.fragment=end && scratch[end-1]!='\n';
    while(end && scratch[end-1]!='\n') --end;
    if(!end) { result.last=unavailable(Quality::Malformed); break; }
    size_t begin=end-1;
    while(begin && scratch[begin-1]!='\n') --begin;
    if(base && !begin) { result.last=unavailable(Quality::Malformed); break; }
    const Record last=parse(scratch+begin,end-begin,false);
    result.last=last.time;
    const bool same=base+begin==0 && end-begin==firstLength;
    if(last.valid && (!first.valid || !first.boot || !last.boot ||
       (same ? (last.boot!=first.boot || last.sequence!=first.sequence) :
       (last.boot<first.boot || (last.boot==first.boot && last.sequence<=first.sequence)))))
      result.last=unavailable(Quality::Inconsistent);
  } while(false);
  if(::close(fd)) result.opened=result.last=unavailable(Quality::Io);
  return result;
}
void restoreCurrent(const char* header, size_t length, uint64_t size, uint32_t generation) {
  currentEnds={}; currentEnds.opened=parse(header,length,true).time;
  currentEnds.last=unavailable(Quality::Unknown); currentSize=size; currentGeneration=generation;
}
void written(const char* line, size_t length, bool first, uint64_t size, uint32_t generation) {
  const Record record=parse(line,length,first);
  if(first || generation!=currentGeneration) { currentEnds={}; currentEnds.opened=first ? record.time : unavailable(Quality::Unknown); }
  currentEnds.last=record.time; currentSize=size; currentGeneration=generation;
}
Ends current(uint64_t& size, uint32_t& generation) { size=currentSize; generation=currentGeneration; return currentEnds; }
}
