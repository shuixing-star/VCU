"""Offline baseline telemetry / GED commands -> ESP parking CSV, no device I/O."""
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
#include <math.h>
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
#include "vcu_parking_sample.h"
#include "mx_log.h"
#include "vcu_fault_log.h"
#include "vcu_k2_fault.h"
#include "vcu_current_trace.h"
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
static void vTaskDelete(void *task) {(void)task;}
static void vTaskDelay(TickType_t delay) {ticks+=delay;}
static int fail_companion,fail_create;
static FILE *test_companion_open(const char *path,const char *mode) {
    if((fail_companion==1 && strstr(path,"_faults.csv")) ||
       (fail_companion==2 && strstr(path,"_current_trace.csv"))) {errno=EACCES;return NULL;}
    return fopen(path,mode);
}
static void (*task_fn)(void*);
static int xTaskCreatePinnedToCore(void (*fn)(void*),const char *name,int stack,void *arg,int priority,void *handle,int core) {
    (void)name;(void)stack;(void)arg;(void)priority;(void)handle;(void)core;task_fn=fn;return fail_create?0:pdPASS;
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
        {1,0,255,3,48,27,193,153},
        {1,0,210,2,150,73,0,3}
    };
    for (unsigned part=0;part<4;part++) {
        assert(vcu_stm32_gateway_process_can_frame(0x18A0D0EFUL|((uint32_t)part<<16),parts[part],8,true));
        assert(vcu_stm32_gateway_process_can_frame(0x18A0D0F0UL|((uint32_t)part<<16),parts[part],8,true));
    }
    uint8_t brake[8]={0,125,0,125,1,1,0,3};
    uint8_t park[8]={0,125,0,125,9,0,0,4};
    uint8_t hold[8]={70,75,25,0,0,0,0,1};
    assert(!vcu_stm32_gateway_process_can_frame(0x0C01EFD0UL,brake,8,false));
    assert(!m_ctx.parking[0].command_seen);
    vcu_stm32_gateway_process_can_frame(0x0C01EFD0UL,brake,8,true);
    vcu_stm32_gateway_process_can_frame(0x0C01F0D0UL,park,8,true);
    vcu_stm32_gateway_process_can_frame(0x1802D0F0UL,hold,8,true);
    /* Wrong address cannot alter the observed left/right commands. */
    vcu_stm32_gateway_process_can_frame(0x0C01EED0UL,park,8,true);
    assert(m_ctx.parking[0].brake_count==1);
    assert(m_ctx.parking[1].park_count==1);
    double sample[VCU_PARKING_FIELD_COUNT];
    vcu_parking_sample_from_state(&m_ctx.public_state,m_ctx.parking,ticks,sample);
    assert(sample[159]==-1 && sample[160]==0 && isnan(sample[161]));
    /* Known vector also asserted against the real STM32 telemetry producer. */
    uint8_t bd[8]={0x11,0x07,0xAD,0x00,0xB7,12,0x23,255};
    assert(!vcu_stm32_gateway_process_can_frame(0x689,bd,8,true));
    assert(!vcu_stm32_gateway_process_can_frame(0x689,bd,7,false));
    bd[4]=0x37;assert(!vcu_stm32_gateway_process_can_frame(0x689,bd,8,false));
    assert(!m_ctx.public_state.has_brake_diag);
    bd[4]=0xB7;bd[6]=0xE3;assert(!vcu_stm32_gateway_process_can_frame(0x689,bd,8,false));
    bd[6]=0x23;assert(vcu_stm32_gateway_process_can_frame(0x689,bd,8,false));
    vcu_parking_sample_from_state(&m_ctx.public_state,m_ctx.parking,ticks,sample);
    assert(sample[161]==1809 && sample[162]==173 && sample[166]==1 && sample[167]==0);
    assert(sample[170]==12 && sample[172]==3 && sample[173]==0 && sample[174]==0 && sample[175]==255);
    bd[4]=0xFB;bd[5]=14;bd[6]=0x3F;bd[7]=0;
    assert(vcu_stm32_gateway_process_can_frame(0x689,bd,8,false));
    vcu_parking_sample_from_state(&m_ctx.public_state,m_ctx.parking,ticks,sample);
    assert(sample[169]==1 && sample[173]==3 && sample[174]==1 && sample[175]==0);
    bd[4]=0xB7;bd[5]=12;bd[6]=0x23;bd[7]=255;
    assert(vcu_stm32_gateway_process_can_frame(0x689,bd,8,false));
    assert(mx_log_start(20));task_fn(NULL);
    mx_log_public_state state;assert(mx_log_get_state(&state));
    assert(!state.enabled && !state.task_running && state.samples_written==1 && state.file_errors==0);
    terminal_mx_log_status(0,NULL);assert(strncmp(terminal_text,"VCU log status\n",15)==0 && strstr(terminal_text,"active path"));
    ticks+=600;assert(mx_log_start(20));task_fn(NULL);
    assert(mx_log_get_state(&state) && state.samples_written==1 && state.file_errors==0);
    for(int fail=1;fail<=2;fail++) {
        fail_companion=fail;assert(mx_log_start(20));task_fn(NULL);
        assert(mx_log_get_state(&state));
        assert(!state.enabled && !state.task_running && state.file_errors==1);
        assert(!strcmp(state.last_error_operation,"open fault/trace"));
        assert(!m_fault_accepting && !m_fault_file && !gw_trace_file);
    }
    fail_companion=0;fail_create=1;assert(!mx_log_start(20));
    assert(!m_fault_accepting && !m_log.task_running && !m_log.enabled);
    fail_create=0;
    snprintf(file_basepath,sizeof file_basepath,"%s/missing/",argv[1]);
    assert(mx_log_start(20));task_fn(NULL);
    assert(mx_log_get_state(&state));
    assert(!state.enabled && !state.task_running && state.samples_written==0 && state.file_errors==1);
    puts("PASS: baseline telemetry -> ESP receive -> CSV; stale/offline and missing SD.");return 0;
}
"""
    companions=(ROOT/'main/vcu_fault_log.c').read_text().replace('#include "vcu_current_trace_log.inc"',(ROOT/'main/vcu_current_trace_log.inc').read_text())
    companions=re.sub(r'^#include .*$', '', companions, flags=re.M)
    source=output/'chain.c';source.write_text(prelude+'\n#define fopen test_companion_open\n'+companions+'\n#undef fopen\n'+context+lookups+receive+writer+tests)
    protocol=stm/'Protocol/Src/vcu_gateway_protocol.c'
    names=set(re.findall(r'\b(vcu_gateway_\w+)\s*\(',protocol.read_text()))
    obj=output/'stm_protocol.o'
    common=[cc,'-D__USE_MINGW_ANSI_STDIO=1','-std=c11','-Wall','-Wextra','-Werror']
    subprocess.run(common+['-I',str(stm/'Protocol/Inc')]+['-D'+n+'=stm_'+n for n in sorted(names)]+['-c',str(protocol),'-o',str(obj)],check=True)
    exe=output/'chain.exe'
    # MinGW's snprintf length warning is unrelated to the embedded file path bound.
    subprocess.run(common+['-Wno-format-truncation','-I',str(ROOT/'main'),str(source),str(obj),str(ROOT/'main/vcu_gateway_protocol.c'),str(ROOT/'main/vcu_log_sample.c'),str(ROOT/'main/vcu_parking_observer.c'),str(ROOT/'main/vcu_parking_sample.c'),str(ROOT/'main/vcu_k2_diag.c'),str(ROOT/'main/vcu_k2_fault.c'),str(ROOT/'main/vcu_current_trace.c'),'-lm','-o',str(exe)],check=True)
    # Use a fresh folder for each test so prior evidence is not overwritten.
    import tempfile
    with tempfile.TemporaryDirectory(prefix='csv-chain-',dir=output) as temp:
        log_dir=Path(temp)/'mx_log';log_dir.mkdir()
        old_log=log_dir/'vcu_053.csv';old_log.write_bytes(b'previous capture preserved\n')
        subprocess.run([str(exe),temp],check=True)
        assert old_log.read_bytes()==b'previous capture preserved\n'
        files=sorted(p for p in (Path(temp)/'mx_log').glob('*.csv') if p.name in ['vcu_054.csv','vcu_055.csv']);assert len(files)==2
        assert [p.name for p in files]==['vcu_054.csv','vcu_055.csv']
        rows=list(csv.reader(files[0].read_text().splitlines(),delimiter=';'))
        assert len(rows)==2 and len(rows[0])==len(rows[1])==179
        keys=[cell.split(':')[0] for cell in rows[0]]
        values=dict(zip(keys,map(float,rows[1])))
        assert values['left_target_rpm']==5000 and values['right_target_rpm']==-5000
        assert values['drive_state']==5 and values['drive_reason']==14 and values['output_enabled']==1
        assert values['fresh_mask']==63
        assert values['left_vbus_v']==102.3 and values['right_motor_fault']==0
        assert values['left_iq_ref_a']==-12.3 and values['left_iq_actual_a']==-11.7
        assert values['left_reference_rpm']==1200
        assert values['left_effective_rpm']==4000
        assert values['diag_fresh_mask']==3 and values['left_diag_age_ms']==0
        assert values['ch6_raw']==1809 and values['ch8_raw']==173
        assert values['brake_diag_version']==1 and values['brake_diag_fresh']==1
        assert values['brake_requested']==1 and values['brake_reset']==1 and values['brake_run']==0
        assert values['brake_reason']==12 and values['brake_tx_queued']==3
        assert values['brake_tx_ok']==0 and values['brake_tx_success']==0
        assert values['parking_schema']==5
        assert len(keys)==len(set(keys))
        assert values['left_speed_integral_a']==-5.4 and values['right_speed_integral_a']==-5.4
        assert values['left_actual_rpm']==1100 and values['left_limit_flags']==0x1B30
        assert values['left_state_flags']==0x99C1 and values['left_control_flags']==(0x99C1|(3<<16))
        assert values['left_startup_state']==3 and values['left_diag_token']==1
        assert values['left_sample_ticks']==1234567890
        for side in ['left','right']:
            for key in ['diag_valid','diag_fresh','direction_change_active','neutral_stop_active',
                        'hall_valid','hall_speed_valid','voltage_saturated','integral_limited',
                        'hall_feedback_limited','handoff_active','low_recovery_active']:
                assert values[side+'_'+key]==1, (side,key)

        import math
        assert math.isnan(values['left_mos_temp_c']) and math.isnan(values['left_motor_temp_c'])
        assert values['right_mos_temp_c']==30 and values['right_motor_temp_c']==35
        assert keys[137:141]==['left_mos_temp_c','left_motor_temp_c','right_mos_temp_c','right_motor_temp_c']
        assert values['left_cmd_kind']==2 and values['left_brake_cmd_n']==1
        assert values['right_cmd_kind']==3 and values['right_park_cmd_n']==1
        assert values['right_hold_reported']==1 and values['right_hold_entries']==1
        assert values['arm_raw']==1800 and values['throttle_raw']==1500
        assert values['left_cmd_control']==1 and values['right_cmd_control']==9
        assert values['left_cmd_brake_raw']==1 and values['left_cmd_fresh']==1
        assert all(re.fullmatch(r'vcu_[0-9]+', p.stem) for p in files)
        for suffix in ['_faults.csv','_current_trace.csv']:
            for main in files:
                companion=main.with_name(main.stem+suffix)
                companion_rows=list(csv.reader(companion.read_text().splitlines(),delimiter=';'))
                assert len(companion_rows)==2 and companion_rows[1][1]=='0'
                assert len(companion_rows[0])==len(companion_rows[1])
        stale=list(csv.reader(files[1].read_text().splitlines(),delimiter=';'))
        stale_values=dict(zip(keys,map(float,stale[1])))
        assert stale_values['fresh_mask']==0
        assert stale_values['output_enabled']==0 and stale_values['diag_fresh_mask']==0
        assert stale_values['left_cmd_fresh']==0 and stale_values['right_status_fresh']==0
        import math
        assert stale_values['brake_diag_fresh']==0 and stale_values['brake_diag_age_ms']>150
        assert math.isnan(stale_values['ch6_raw']) and math.isnan(stale_values['brake_tx_ok'])
        assert all(math.isnan(stale_values[k]) for k in keys[137:141])
        assert math.isnan(stale_values['right_hold_reported']) and math.isnan(stale_values['right_actual_mode'])
        assert stale_values['left_brake_cmd_n']==1 and stale_values['right_park_cmd_n']==1
        for side in ['left','right']:
            assert stale_values[side+'_diag_valid']==1 and stale_values[side+'_diag_fresh']==0
            assert stale_values[side+'_diag_token']==1 and stale_values[side+'_sample_ticks']==1234567890
            for key in ['actual_rpm','speed_integral_a','limit_flags','control_flags','state_flags',
                        'startup_state','direction_change_active','neutral_stop_active','hall_valid',
                        'hall_speed_valid','voltage_saturated','integral_limited','hall_feedback_limited',
                        'handoff_active','low_recovery_active']:
                assert math.isnan(stale_values[side+'_'+key]), (side,key)

        (output/'SIMULATED_baseline.csv').write_bytes(files[0].read_bytes())
        (output/'SIMULATED_offline.csv').write_bytes(files[1].read_bytes())
    print('PASS: 179 chain CSV columns; real receive/write, brake/PARK/HOLD, missing/stale/SD errors; original filename, preserved old capture; fault/trace companions restored.')
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cc',default='gcc');p.add_argument('--stm32-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();run(args.cc,args.stm32_source.resolve(),args.output.resolve())
