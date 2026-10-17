"""Actual K2 queue/encoder -> CAN receiver -> recording task -> companion CSV."""
import argparse,csv,re,subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def run(k2,output,cc="gcc"):
    output.mkdir(parents=True,exist_ok=True)
    app=k2/"Example_2806xCpuTimer"
    strip=lambda s:re.sub(r'^#include .*$', '', s,flags=re.M)
    code=r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <direct.h>
#include "mcpwm_foc.h"
#include "app_manifest_values.h"
#include "app_manifest.h"
const Uint16 g_app_manifest[APP_MANIFEST_TOTAL_WORDS] = {
    [APP_MANIFEST_FIRMWARE_HI]=APP_U32_HI(APP_MANIFEST_FIRMWARE_VERSION),
    [APP_MANIFEST_FIRMWARE_LO]=APP_U32_LO(APP_MANIFEST_FIRMWARE_VERSION)
};
#include "vcu_stm32_gateway.h"
#include "vcu_log_sample.h"
#include "vcu_parking_sample.h"
#include "vcu_fault_log.h"
#include "vcu_k2_fault.h"
#include "vcu_current_trace.h"
#include "mx_log.h"
#undef K2_FAULT_PARTS
#undef K2_FAULT_WORDS
#define HW_VCU_STM32_GATEWAY 1
#define mkdir(path,mode) _mkdir(path)
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
static uint32_t ticks=10000,begin_ms;
static int scenario,created,stopped,failed_once,can_attempts;
static void pump(unsigned ms);
static uint32_t utils_ms_tot(void){return ticks;}
static uint32_t xTaskGetTickCount(void){return ticks;}
static void vTaskDelay(TickType_t t){pump(t);}
static void vTaskDelayUntil(TickType_t *last,TickType_t period){
    uint32_t end=*last+period;
    while((int32_t)(end-ticks)>0 && !stopped)pump(5);
    *last=end;
}
static void vTaskDelete(void *t){(void)t;}
static void (*task_fn)(void*);
static int xTaskCreatePinnedToCore(void(*fn)(void*),const char*n,int sz,void*a,int pr,void*h,int co){
    (void)n;(void)sz;(void)a;(void)pr;(void)h;(void)co;task_fn=fn;return 1;
}
static char file_basepath[1024];
static void commands_printf(const char*fmt,...){(void)fmt;}
static void terminal_register_command_callback(const char*a,const char*b,const char*c,void(*d)(int,const char**)){
    (void)a;(void)b;(void)c;(void)d;
}
bool vcu_stm32_gateway_get_parking_snapshot(vcu_stm32_gateway_state_t *s,vcu_parking_observer_t *o){memset(s,0,sizeof *s);memset(o,0,2*sizeof *o);return true;}
static volatile mcpwm_foc_trip_sample_t m_trip_latest;
static volatile mcpwm_foc_trip_log_info_t m_trip_log_info;
static Uint16 m_active_can_node_id=239;
static Uint16 can_transmit(Uint32 id,const Uint16 *d,Uint16 len){
    uint8_t bytes[8];unsigned part=(id>>16)&255U;
    can_attempts++;
    if(scenario==3 && !failed_once && part==5){failed_once=1;return 0;} /* queue retry */
    for(unsigned i=0;i<8;i++)bytes[i]=(uint8_t)d[i];
    if(scenario==1 && !failed_once && part==5){failed_once=1;return 1;} /* bus loss */
    if(scenario==2 && !failed_once && part==8){failed_once=1;bytes[4]^=32;} /* corruption */
    assert(vcu_fault_log_receive(id,bytes,(uint8_t)len,true,ticks));return 1;
}
static void ged_put_u16(Uint16 *d,Uint16 at,Uint16 v){d[at]=v&255;d[at+1]=v>>8;}
"""
    code+=(app/"motor/fault_tx.inc").read_text()
    code+=strip((app/"interface/k2_fault_can.inc").read_text())
    code+=strip((ROOT/"main/vcu_fault_log.c").read_text().replace('#include "vcu_current_trace_log.inc"', (ROOT/"main/vcu_current_trace_log.inc").read_text()))
    code+=strip((ROOT/"main/mx_log.c").read_text())
    code+=r"""
static void capture(void){
    memset((void*)&m_trip_latest,0,sizeof m_trip_latest);
    memset((void*)&m_trip_log_info,0,sizeof m_trip_log_info);
    m_trip_latest.sample_sequence=ticks*10;m_trip_latest.control_sample_sequence=ticks*10-1;
    m_trip_latest.current_a=610;m_trip_latest.current_b=-300;m_trip_latest.current_c=-310;
    m_trip_latest.current_d=-25;m_trip_latest.current_q=-350;m_trip_latest.current_q_reference=-370;
    m_trip_latest.id_reference=-30;m_trip_latest.iq_requested=-380;m_trip_latest.integral_a=-170;
    m_trip_latest.pole_pairs=4;m_trip_latest.target_hz=-300;m_trip_latest.reference_hz=-90;
    m_trip_latest.mechanical_speed_rpm=201;m_trip_latest.phase_peak=610;
    m_trip_latest.instant_limit=600;m_trip_latest.continuous_limit=400;
    m_trip_latest.continuous_trip_ticks=200000;
    m_trip_latest.adc_a=3300;m_trip_latest.adc_b=1800;
    m_trip_latest.hall_direction=-1;m_trip_latest.q_clip_direction=-1;
    m_trip_latest.hall_valid=m_trip_latest.angle_valid=1;
    m_trip_latest.hall_state=5;m_trip_latest.hall_cos=m_trip_latest.phase_cos=1;
    m_trip_latest.raw_sin=1;m_trip_latest.vbus=105;
    m_trip_latest.hall_period=333;m_trip_latest.hall_elapsed=5;
    m_trip_latest.trip_ost_mask=7;
    m_trip_log_info.latest_fault_reason=MCPWM_FOC_FAULT_OVERCURRENT;
    m_trip_log_info.fault_reason=MCPWM_FOC_FAULT_HALL_DIRECTION;
    m_trip_log_info.trigger_sequence=ticks*10-50;m_trip_log_info.event_count=1;
    fault_tx_capture();
}
static void pump(unsigned ms){
    if(scenario>=6 && scenario!=12){ticks+=ms;return;}
    for(unsigned done=0;done<ms;done+=5){
        ticks+=5;
        if(!created && ticks-begin_ms>=100 && scenario!=5){
            capture();created=1;
            if(scenario==12)for(unsigned n=0;n<3;n++)capture();
            /* Normal re-arm/cleared live state cannot erase the transmit FIFO. */
            memset((void*)&m_trip_latest,0,sizeof m_trip_latest);
            memset((void*)&m_trip_log_info,0,sizeof m_trip_log_info);
        }
        send_fault_diagnostics(ticks*10);
        if(!stopped && ticks-begin_ms>=((scenario==4 || scenario==12)?120U:1200U)){mx_log_stop();stopped=1;}
    }
}
static void direct_packet(unsigned side,unsigned token,uint16_t *w,unsigned omit){
    for(unsigned part=0;part<27;part++){
        uint8_t b[8]={(uint8_t)token,(uint8_t)(token>>8),0,0,0,0,0,0};
        for(unsigned j=0;j<3;j++){unsigned at=part*3+j;uint16_t v=at<80?w[at]:0;b[2+j*2]=(uint8_t)v;b[3+j*2]=(uint8_t)(v>>8);}
        if(part!=omit)assert(vcu_fault_log_receive(0x1900D000UL|((uint32_t)part<<16)|(239+side),b,8,true,ticks));
    }
}
int main(int argc,char**argv){
    assert(argc==4);scenario=atoi(argv[2]);unsigned hz=(unsigned)atoi(argv[3]);
    snprintf(file_basepath,sizeof file_basepath,"%s/",argv[1]);begin_ms=ticks;
    if(scenario<6 || scenario==12){
        assert(mx_log_start(hz));task_fn(NULL);
        assert(!m_log.enabled && !m_log.task_running && m_log.file_errors==0);
        assert(m_fault_tx_count==0);
        if(scenario!=5)assert(m_fault_rx[0].duplicates>=1 || scenario==4);
        if(scenario==2)assert(m_fault_rx[0].crc_errors==1);
        if(scenario==1)assert(m_fault_rx[0].timeouts>=1);
    }else{
        /* Exercise two node assemblers, recording boundaries and stop summary
         * through the real writer without sharing a sender's node identity. */
        assert(mx_log_start(20));FILE *f=NULL;assert(open_log_file(&f));
        assert(vcu_fault_log_open(m_log.active_path));
        uint16_t w[80];mcpwm_foc_trip_sample_t sample;mcpwm_foc_trip_log_info_t info;
        capture();sample=m_trip_latest;info=m_trip_log_info;
        fault_encode(&sample,&info,1,0,ticks*10,w);
        if(scenario==6){
            direct_packet(0,1,w,99);direct_packet(1,1,w,99);
            direct_packet(0,2,w,99);direct_packet(1,2,w,99);
            assert(m_fault_count==2);
        }else if(scenario==7){
            /* Pre-session fault is not relabelled as this recording's event. */
            fault_word32(w,4,ticks*10-1000);w[79]=fault_wire_crc(w);
            direct_packet(0,1,w,99);assert(m_fault_count==0);
        }else if(scenario==8){
            for(unsigned i=1;i<=20;i++){fault_word32(w,2,i);w[79]=fault_wire_crc(w);direct_packet(0,i,w,99);}
            assert(m_fault_count==16 && m_fault_drops==4);
        }else if(scenario==9){
            direct_packet(0,1,w,5); /* incomplete event is visible in final summary */
        }else if(scenario==10){
            /* Counter restart plus backwards sender uptime must not deduplicate a rebooted node. */
            direct_packet(0,1,w,99);
            ticks+=100;fault_word32(w,8,20);fault_word32(w,4,10);w[79]=fault_wire_crc(w);
            direct_packet(0,1,w,99);assert(m_fault_count==2 && m_fault_rx[0].epoch==1);
        }else if(scenario==11){
            /* Stop accepts only already-triggered events during the grace interval. */
            mx_log_stop();ticks+=100;
            fault_word32(w,8,ticks*10);fault_word32(w,4,ticks*10);w[79]=fault_wire_crc(w);
            direct_packet(0,1,w,99);assert(m_fault_count==0);
        }
        stopped=1;created=1;mx_log_stop();assert(vcu_fault_log_finish());fclose(f);
    }
    puts("PASS automatic fault record");return 0;
}
"""
    src=output/"auto_fault.c";src.write_text(code)
    exe=output/"auto_fault.exe"
    subprocess.run([cc,"-std=c11","-O2","-Wall","-Wextra","-Wno-unknown-pragmas",
        "-Wno-format-truncation","-I",str(app/"tests/vofa_host"),"-I",str(app/"motor"),
        "-I",str(app),"-I",str(ROOT/"main"),str(src),
        str(ROOT/"main/vcu_log_sample.c"),str(ROOT/"main/vcu_parking_sample.c"),str(ROOT/"main/vcu_parking_observer.c"),str(ROOT/"main/vcu_k2_fault.c"),str(ROOT/"main/vcu_current_trace.c"),"-lm","-o",str(exe)],check=True)
    runs=0
    for case in range(13):
        for hz in ([1,20,50] if case==0 else [20]):
            with tempfile.TemporaryDirectory(dir=output) as td:
                subprocess.run([str(exe),td,str(case),str(hz)],check=True,capture_output=True,text=True)
                fault=next((Path(td)/"mx_log").glob("*_faults.csv"))
                rows=list(csv.reader(fault.read_text().splitlines(),delimiter=";"))
                keys=[x.split(":")[0] for x in rows[0]]
                assert all(len(r)==len(keys) for r in rows[1:])
                events=[dict(zip(keys,r)) for r in rows[1:] if r[1]=="1"]
                expected=0 if case in [5,7,9,11] else (2 if case in [6,10] else (16 if case==8 else (4 if case==12 else 1)))
                assert len(events)==expected,(case,len(events),expected)
                if events:
                    v=events[0]
                    assert float(v["ia"])==610 and float(v["iq_last"])==-350
                    assert float(v["raw_reference_rpm"])==-1350 and float(v["raw_target_rpm"])==-4500
                    assert float(v["hall_raw_angle"])==90 and float(v["hall_direction"])==-1
                    assert int(v["trigger_ticks"])==int(v["control_ticks"])+1
                if case==6: assert {x["node"] for x in events}=={"239","240"}
                if case==8: assert rows[-1][5]=="4"
                if case==9: assert int(rows[-1][7])>=1
                if case==0 and hz==20:(output/"SIMULATED_vcu_faults.csv").write_bytes(fault.read_bytes())
                runs+=1
    print("PASS",runs,"end-to-end scenarios; 1/20/50 Hz, retries, loss, CRC, both nodes, reboot, stop, overflow")

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--k2",type=Path,required=True);p.add_argument("--output",type=Path,required=True);p.add_argument("--cc",default="gcc")
    a=p.parse_args();run(a.k2,a.output,a.cc)
