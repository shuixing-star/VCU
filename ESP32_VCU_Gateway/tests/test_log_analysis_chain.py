"""Offline STM32 baseline encoder -> actual ESP receiver -> actual SD CSV writer."""
import argparse, csv, re, subprocess
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def run(cc, stm, output):
    output.mkdir(parents=True, exist_ok=True)
    gateway=(ROOT/'main/vcu_stm32_gateway.c').read_text(encoding='utf-8-sig')
    context=gateway[gateway.index('#define VCU_HEARTBEAT_FLAG'):gateway.index('static uint8_t s_command_sequence;')]
    lookups=gateway[gateway.index('static int telemetry_index_from_id'):gateway.index('static const char *drive_state_name')]
    receive=gateway[gateway.index('static bool process_tx_probe'):gateway.index('static void terminal_request_stop')]
    writer=re.sub(r'^#include .*$', '', (ROOT/'main/mx_log.c').read_text(), flags=re.M)
    prelude=r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <limits.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <direct.h>
#include "vcu_stm32_gateway.h"
#include "vcu_log_sample.h"
#include "mx_log.h"
#include "vcu_fault_log.h"
#include "vcu_k2_fault.h"
#define HW_VCU_STM32_GATEWAY 1
#define mkdir(path, mode) _mkdir(path)
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
#define tskNO_AFFINITY 0
#define pdPASS 1
static uint32_t ticks=10000;
static uint32_t now_ms(void) {return ticks;}
static uint32_t xTaskGetTickCount(void) {return ticks;}
static uint32_t utils_ms_tot(void) {return ticks;}
static void vTaskDelayUntil(TickType_t *last, TickType_t delay) {*last+=delay;ticks+=delay;mx_log_stop();}
static void vTaskDelay(TickType_t delay) {ticks+=delay;}
static void vTaskDelete(void *task) {(void)task;}
static void (*task_fn)(void*);
static int xTaskCreatePinnedToCore(void (*fn)(void*),const char *name,int stack,void *arg,int priority,void *handle,int core) {
    (void)name;(void)stack;(void)arg;(void)priority;(void)handle;(void)core;task_fn=fn;return pdPASS;
}
static char file_basepath[1024];
static char terminal_text[2048];
static void commands_printf(const char *format,...) {va_list ap;va_start(ap,format);vsnprintf(terminal_text,sizeof terminal_text,format,ap);va_end(ap);}
static void terminal_register_command_callback(const char *a,const char *b,const char *c,void (*d)(int,const char**)) {(void)a;(void)b;(void)c;(void)d;}
extern void stm_vcu_gateway_pack_heartbeat(const vcu_gateway_heartbeat_t*,uint8_t*);
extern void stm_vcu_gateway_pack_rc_input(const vcu_gateway_rc_input_t*,uint8_t*);
extern void stm_vcu_gateway_pack_targets(const vcu_gateway_targets_t*,uint8_t*);
extern void stm_vcu_gateway_pack_motor_status(const vcu_gateway_motor_status_t*,uint8_t*);
extern void stm_vcu_gateway_pack_diagnostics(const vcu_gateway_diagnostics_t*,uint8_t*);
"""
    tests=r"""
int main(int argc,char **argv) {
    assert(argc==2);(void)s_motor_raw_names;
    snprintf(file_basepath,sizeof file_basepath,"%s/",argv[1]);
    m_ctx.public_state.initialized=true;
    uint8_t data[8];
    vcu_gateway_heartbeat_t hb={1,5,2,16,0,14,1};
    vcu_gateway_rc_input_t rc={1500,992,1800,3,2};
    vcu_gateway_targets_t targets={5000,-5000,5000,-5000};
    vcu_gateway_motor_status_t left={4999,721,45,0,1,3},right={-4998,719,46,7,2,4};
    vcu_gateway_diagnostics_t diag={100,2,3,0,5};
    stm_vcu_gateway_pack_heartbeat(&hb,data);assert(vcu_stm32_gateway_process_can_frame(0x680,data,8,false));
    stm_vcu_gateway_pack_rc_input(&rc,data);assert(vcu_stm32_gateway_process_can_frame(0x681,data,8,false));
    stm_vcu_gateway_pack_targets(&targets,data);assert(vcu_stm32_gateway_process_can_frame(0x682,data,8,false));
    stm_vcu_gateway_pack_motor_status(&left,data);assert(vcu_stm32_gateway_process_can_frame(0x683,data,8,false));
    stm_vcu_gateway_pack_motor_status(&right,data);assert(vcu_stm32_gateway_process_can_frame(0x684,data,8,false));
    stm_vcu_gateway_pack_diagnostics(&diag,data);assert(vcu_stm32_gateway_process_can_frame(0x685,data,8,false));
    uint8_t parts[4][8]={
        {1,0,160,15,176,4,76,4},
        {1,0,133,255,139,255,202,255},
        {1,0,255,3,48,0,65,1},
        {1,0,210,2,150,73,0,3}
    };
    for (unsigned part=0;part<4;part++) {
        assert(vcu_stm32_gateway_process_can_frame(0x18A0D0EFUL|((uint32_t)part<<16),parts[part],8,true));
        assert(vcu_stm32_gateway_process_can_frame(0x18A0D0F0UL|((uint32_t)part<<16),parts[part],8,true));
    }
    assert(mx_log_start(20));task_fn(NULL);
    mx_log_public_state state;assert(mx_log_get_state(&state));
    assert(!state.enabled && !state.task_running && state.samples_written==1 && state.file_errors==0);
    terminal_mx_log_status(0,NULL);assert(strstr(terminal_text,"VCU log status") && strstr(terminal_text,"active path"));
    ticks+=600;assert(mx_log_start(20));task_fn(NULL);
    assert(mx_log_get_state(&state) && state.samples_written==1 && state.file_errors==0);
    snprintf(file_basepath,sizeof file_basepath,"%s/missing/",argv[1]);
    assert(mx_log_start(20));task_fn(NULL);
    assert(mx_log_get_state(&state));
    assert(!state.enabled && !state.task_running && state.samples_written==0 && state.file_errors==1);
    puts("PASS: baseline telemetry -> ESP receive -> CSV; stale/offline and missing SD.");return 0;
}
"""
    source=output/'chain.c';source.write_text(prelude+context+lookups+receive+re.sub(r'^#include .*$', '', (ROOT/'main/vcu_fault_log.c').read_text(), flags=re.M)+writer+tests)
    protocol=stm/'Protocol/Src/vcu_gateway_protocol.c'
    names=set(re.findall(r'\b(vcu_gateway_\w+)\s*\(',protocol.read_text()))
    obj=output/'stm_protocol.o'
    common=[cc,'-std=c11','-Wall','-Wextra','-Werror']
    subprocess.run(common+['-I',str(stm/'Protocol/Inc')]+['-D'+n+'=stm_'+n for n in sorted(names)]+['-c',str(protocol),'-o',str(obj)],check=True)
    exe=output/'chain.exe'
    # MinGW's snprintf length warning is unrelated to the embedded file path bound.
    subprocess.run(common+['-Wno-format-truncation','-I',str(ROOT/'main'),str(source),str(obj),str(ROOT/'main/vcu_gateway_protocol.c'),str(ROOT/'main/vcu_log_sample.c'),str(ROOT/'main/vcu_k2_diag.c'),str(ROOT/'main/vcu_k2_fault.c'),'-o',str(exe)],check=True)
    # Use a fresh folder for each test so prior evidence is not overwritten.
    import tempfile
    with tempfile.TemporaryDirectory(prefix='csv-chain-',dir=output) as temp:
        subprocess.run([str(exe),temp],check=True)
        files=sorted(p for p in (Path(temp)/'mx_log').glob('*.csv') if not p.stem.endswith('_faults'));assert len(files)==2
        rows=list(csv.reader(files[0].read_text().splitlines(),delimiter=';'))
        assert len(rows)==2 and len(rows[0])==len(rows[1])==33
        keys=[cell.split(':')[0] for cell in rows[0]]
        values=dict(zip(keys,map(float,rows[1])))
        assert values['left_target_rpm']==5000 and values['right_target_rpm']==-5000
        assert values['drive_state']==5 and values['drive_reason']==14 and values['output_enabled']==1
        assert values['fresh_mask']==63
        assert values['left_vbus_v']==102.3 and values['right_motor_fault']==0
        assert values['left_iq_ref_a']==-12.3 and values['left_iq_actual_a']==-11.7
        assert values['left_speed_integral_a']==-5.4 and values['left_reference_rpm']==1200
        assert values['left_effective_rpm']==4000 and values['left_actual_rpm']==1100
        assert values['diag_fresh_mask']==3 and values['left_diag_age_ms']==0
        stale=list(csv.reader(files[1].read_text().splitlines(),delimiter=';'))
        stale_values=dict(zip(keys,map(float,stale[1])))
        assert stale_values['fresh_mask']==0
        assert stale_values['output_enabled']==0 and stale_values['diag_fresh_mask']==0
        (output/'SIMULATED_baseline.csv').write_bytes(files[0].read_bytes())
        (output/'SIMULATED_offline.csv').write_bytes(files[1].read_bytes())
    print('PASS: 33 Log Analysis columns, internal ramp/Iq/integral, stale diagnostics and unchanged VCU context.')
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cc',default='gcc');p.add_argument('--stm32-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();run(args.cc,args.stm32_source.resolve(),args.output.resolve())
