"""Compile the actual firmware status formatter with the actual Qt host parser. No device I/O."""
import argparse, os, re, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def run(host, qt, compiler, output):
    output.mkdir(parents=True,exist_ok=True)
    src=(ROOT/'main/mx_log.c').read_text()
    state=src[src.index('typedef struct {'):src.index('static char m_file_buffer')]
    formatter=src[src.index('static void terminal_mx_log_status('):src.index('void mx_log_init(')]
    commands=(ROOT/'main/commands.c').read_text()
    limit=int(re.search(r'#define\s+PRINT_BUFFER_SIZE\s+(\d+)',commands).group(1))
    code=r"""
#include <QString>
#include <QByteArray>
#include <cassert>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include "pages/sdlogstatus.h"
#define MX_LOG_DEFAULT_HZ 20
#define MX_LOG_SLOW_FLUSH_MS 50U
static QString reply;
static void commands_printf(const char *format,...) {
    char buffer[PRINT_BUFFER_SIZE-1];
    va_list ap;va_start(ap,format);vsnprintf(buffer,sizeof(buffer),format,ap);va_end(ap);
    reply=QString::fromUtf8(buffer);
}
"""
    tests=r"""
static void check(const char *name,bool enabled,bool running,unsigned samples,unsigned errors) {
    terminal_mx_log_status(0,nullptr);
    SdLogStatus s;assert(SdLogStatus::parse(reply,s));
    assert(s.enabled==enabled && s.running==running && s.samples==samples && s.errors==errors);
    assert(reply.startsWith("VCU log status\n"));
    const bool hostRunning=s.enabled||s.running;
    assert(hostRunning==(enabled||running));
    std::printf("PASS: %s; parsed=%d/%d samples=%llu errors=%llu path=%s\n",name,
        s.enabled,s.running,(unsigned long long)s.samples,(unsigned long long)s.errors,s.path.toUtf8().constData());
}
int main() {
    check("idle/start available",false,false,0,0);
    const QString idle=reply;
    SdLogStatus s;
    QString broken=idle;broken.replace("VCU log status\n", "VCU log status (parking/brake schema 2; baseline SD; one CSV)\n");
    assert(!SdLogStatus::parse(broken,s));
    QString crlf=idle;crlf.replace("\n","\r\n");assert(SdLogStatus::parse(crlf,s));
    m_log.enabled=true;
    check("starting/stop available",true,false,0,0);
    m_log.task_running=true;m_log.samples_written=25;
    std::strcpy(m_log.active_path,"/sdcard/mx_log/vcu_054.csv");
    check("recording/stop available",true,true,25,0);assert(SdLogStatus::parse(reply,s));assert(s.path==m_log.active_path);
    m_log.enabled=false;
    check("finishing/stop available",false,true,25,0);
    m_log.task_running=false;
    check("stopped/start available",false,false,25,0);
    m_log.file_errors=1;m_log.last_error_errno=EIO;std::strcpy(m_log.last_error_operation,"fopen");
    check("SD error reported",false,false,25,1);assert(SdLogStatus::parse(reply,s));assert(s.lastError.contains("fopen"));
    m_log.samples_written=UINT32_MAX;m_log.file_errors=UINT32_MAX;m_log.flush_count=UINT32_MAX;
    m_log.slow_flush_count=UINT32_MAX;m_log.max_sample_write_ms=UINT32_MAX;m_log.max_flush_ms=UINT32_MAX;
    std::memset(m_log.active_path,'x',sizeof(m_log.active_path)-1);m_log.active_path[sizeof(m_log.active_path)-1]=0;
    check("400-byte limit with long path",false,false,UINT32_MAX,UINT32_MAX);
    std::puts("PASS: original R2 rejected, fixed reply accepted by unchanged production SdLogStatus parser; no device connected");
}
"""
    f=output/'sd_recording_compat.cpp';f.write_text('#define PRINT_BUFFER_SIZE '+str(limit)+'\n'+code+state+formatter+tests)
    exe=output/'sd_recording_compat.exe'
    # The production C initializer intentionally zero-initializes omitted fields.
    command=[str(compiler),'-std=c++11','-Wall','-Wextra','-Werror','-Wno-missing-field-initializers','-D__USE_MINGW_ANSI_STDIO=1',
        '-I'+str(host),'-I'+str(qt/'include'),'-I'+str(qt/'include/QtCore'),str(f),
        '-L'+str(qt/'lib'),'-lQt5Core','-o',str(exe)]
    subprocess.run(command,check=True)
    env=os.environ.copy();env['PATH']=str(qt/'bin')+';'+str(compiler.parent)+';'+env['PATH']
    result=subprocess.run([str(exe)],env=env,capture_output=True,text=True)
    (output/'result.txt').write_text(result.stdout+result.stderr)
    print(result.stdout+result.stderr,end='');result.check_returncode()
if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--host-source',type=Path,required=True);a.add_argument('--qt',type=Path,required=True)
    a.add_argument('--cxx',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    x=a.parse_args();run(x.host_source.resolve(),x.qt.resolve(),x.cxx.resolve(),x.output.resolve())
