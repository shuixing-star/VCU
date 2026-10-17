#include "vcu_flight_log.h"
#include "../../common/vcu_log_store.h"
#include "../../common/vcu_log_motor_decode.h"
#ifdef VCU_LOG_HOST_TEST
#include "vcu_log_esp_mock.h"
#define CAPTURE_SOURCE "SIMULATED_HOST_TEST_NOT_FIELD_DATA"
#else
#include "conf_general.h"
#define CAPTURE_SOURCE "device_capture_not_simulation"
#include "log.h"
#include "terminal.h"
#include "commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>

static vcu_log_store_t *store;
static portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
static bool ready, export_request, exporting, exported, auto_attempted;
static uint32_t file_errors, export_number, bus[8];
static char last_path[160];
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time()/1000); }

void vcu_flight_log_receive(uint32_t id,const uint8_t *d,uint8_t len,bool ext,bool rtr,uint32_t now) {
    if(!ready) return;
    portENTER_CRITICAL(&lock);
    vcu_log_store_frame(store,id,d,len,ext,rtr,now);
    portEXIT_CRITICAL(&lock);
}
void vcu_flight_log_bus(uint32_t state,uint32_t tec,uint32_t rec,uint32_t missed,uint32_t overrun,uint32_t failed,uint32_t errors,uint32_t queue_lost) {
    portENTER_CRITICAL(&lock);
    bus[0]=state;bus[1]=tec;bus[2]=rec;bus[3]=missed;bus[4]=overrun;bus[5]=failed;bus[6]=errors;bus[7]=queue_lost;
    portEXIT_CRITICAL(&lock);
}
static void hex(FILE *f,const uint8_t *data,unsigned n) {
    unsigned i; for(i=0;i<n;i++) fprintf(f,"%02X",data[i]);
}
static void number(FILE *f,bool valid,long long value) { if(valid) fprintf(f,"%lld",value); fputc(',',f); }
static void motor_header(FILE *f,const char *side) {
    const char *names[]={"voltage_dV","bus_current_dA","phase_current_reported_dA","raw_speed_rpm","controller_temperature_reported_C","motor_temperature_reported_C","fault_code","fault_bits_hex","electrical_age_ms","state_age_ms","heartbeat_age_ms","online","electrical_raw_hex","state_raw_hex"};
    unsigned i; for(i=0;i<sizeof(names)/sizeof(names[0]);i++) fprintf(f,"%s_%s,",side,names[i]);
}
static void motor_row(FILE *f,const uint8_t *q) {
    mx_can_status_t status={0};
    bool e=(q[22]&1U) && mx_can_unpack_electrical_status(q,&status);
    bool s=(q[22]&2U) && mx_can_unpack_state_status(q+8,&status);
    number(f,e,status.supply_voltage_decivolts); number(f,e,status.bus_current_deciamps);
    number(f,e,status.phase_current_deciamps); number(f,e,status.speed_rpm);
    number(f,s,status.controller_temperature_c); number(f,s,status.motor_temperature_c); number(f,s,status.fault_code);
    if(q[22]&2U) { hex(f,q+11,3); } fputc(',',f);
    number(f,vcu_log_u16(q+16)!=65535U,vcu_log_u16(q+16));
    number(f,vcu_log_u16(q+18)!=65535U,vcu_log_u16(q+18));
    number(f,vcu_log_u16(q+20)!=65535U,vcu_log_u16(q+20));
    number(f,true,(q[22]&4U)!=0U);
    if(q[22]&1U) { hex(f,q,8); }fputc(',',f);
    if(q[22]&2U) { hex(f,q+8,8); }fputc(',',f);
}
static void snapshot_row(FILE *f,const vcu_log_sample_t *row) {
    const uint8_t *p=row->data;
    unsigned i;
    fprintf(f,"%lu,%lu,%lu,%lu,%lu,%u,%u,%lu,%u,%u,%u,",
      (unsigned long)store->esp_session,(unsigned long)row->rx_ms,(unsigned long)vcu_log_u32(p+12),
      (unsigned long)vcu_log_u32(p+8),(unsigned long)vcu_log_u32(p+4),p[2],p[3],
      (unsigned long)vcu_log_u32(p+16),p[44],p[45],p[46]);
    number(f,p[47],vcu_log_u32(p+20));
    for(i=24;i<32;i+=2) number(f,p[47],vcu_log_u16(p+i));
    for(i=32;i<44;i+=2) number(f,p[47],(int16_t)vcu_log_u16(p+i));
    motor_row(f,p+48);motor_row(f,p+72);
    number(f,vcu_log_u32(p+100)!=0,vcu_log_u32(p+96));
    number(f,vcu_log_u32(p+100)!=0,vcu_log_u32(p+124));
    number(f,true,vcu_log_u32(p+100));number(f,true,p[104]);number(f,true,p[105]);number(f,true,p[106]);
    for(i=0;i<2;i++) {
        const uint8_t *cmd=p+108+i*8;
        number(f,vcu_log_u32(p+100)!=0,(int)vcu_log_u16(cmd+2)-32000);
        number(f,vcu_log_u32(p+100)!=0,cmd[4]);
        number(f,(p[105] & (1U << i)) != 0U,(int)vcu_log_u16(cmd+2)-32000);
        if(vcu_log_u32(p+100)) { hex(f,cmd,8); }fputc(',',f);
    }
    for(i=128;i<148;i+=4) number(f,true,vcu_log_u32(p+i));
    number(f,true,(vcu_log_u32(p+128)>>16)&255U); number(f,true,(vcu_log_u32(p+128)>>24)&255U);
    number(f,true,(vcu_log_u32(p+128)&4U)!=0);
    number(f,true,vcu_log_u16(p+148));number(f,true,vcu_log_u16(p+150));
    for(i=152;i<172;i+=4) number(f,true,vcu_log_u32(p+i));
    number(f,true,p[172]);number(f,p[107]!=255U,vcu_log_u32(p+8)-p[107]);hex(f,p,VCU_LOG_SIZE);fputc('\n',f);
}
static FILE *open_csv(const char *name) {
    char path[200];snprintf(path,sizeof(path),"%s/%s",last_path,name);
    return fopen(path,"w");
}
static bool close_csv(FILE *f) {
    bool ok=!ferror(f); if(fflush(f)) ok=false; if(fclose(f)) ok=false;return ok;
}
static bool export_files(void) {
    FILE *f;
    unsigned k, n;
    uint32_t start, stats[8];
    char dir[100];
    snprintf(dir,sizeof(dir),"%smx_log",file_basepath);
    if(mkdir(dir,0775) && errno!=EEXIST) return false;
    do { snprintf(last_path,sizeof(last_path),"%s/flight_%08lX_%03lu",dir,(unsigned long)store->esp_session,(unsigned long)++export_number); }
    while(mkdir(last_path,0775) && errno==EEXIST);
    f=open_csv("snapshots.csv");if(!f)return false;
    fprintf(f,"esp_session,esp_rx_dequeue_ms,stm_boot_nonce,stm_capture_ms,snapshot_seq,record_kind,gateway_command,stm_build_id,drive_state,drive_reason,flags,rc_rx_ms,throttle_raw,steering_raw,arm_raw,reset_raw,throttle_per_mille,steering_per_mille,left_base_rpm,right_base_rpm,left_target_rpm,right_target_rpm,");
    motor_header(f,"left");motor_header(f,"right");
    fprintf(f,"tx_attempt_ms,tx_observed_done_ms,tx_attempt_seq,mailbox_admitted_mask,txok_mask,pair_success,left_wire_target_rpm,left_command_flags,left_txok_target_rpm,left_command_raw_hex,right_wire_target_rpm,right_command_flags,right_txok_target_rpm,right_command_raw_hex,stm_esr,stm_tx_errors,stm_rx_overflows,stm_rx_errors,stm_log_dropped,stm_tec,stm_rec,stm_bus_off,stm_queue_used,stm_queue_highwater,stm_error_events_lost,left_command_id,right_command_id,stm_tx_log_dropped,crsf_overflows,boot_nonce_valid,control_step_ms,record_raw_hex\n");
    for(k=0;k<2;k++) {
        uint32_t total=k?store->event_total:store->sample_total;
        unsigned cap=k?VCU_LOG_EVENTS:VCU_LOG_SAMPLES;
        const vcu_log_sample_t *rows=k?store->events:store->samples;
        start=total>cap?total-cap:0;
        for(n=start;n<total;n++) if(vcu_log_in_window(store,rows[n%cap].rx_ms)) snapshot_row(f,&rows[n%cap]);
    }
    if(!close_csv(f))return false;
    f=open_csv("tx_results.csv");if(!f)return false;
    fprintf(f,"esp_session,esp_rx_dequeue_ms,stm_attempt_ms_low16,attempt_seq_low16,mailbox_admitted_mask,txok_mask,pair_success,left_command_heartbeat,right_command_heartbeat,elapsed_ms,raw_hex\n");
    start=store->tx_total>VCU_LOG_TX_ROWS?store->tx_total-VCU_LOG_TX_ROWS:0;
    for(n=start;n<store->tx_total;n++) {
        const vcu_log_tx_t *r=&store->tx[n%VCU_LOG_TX_ROWS];const uint8_t *d=r->data;
        if(!vcu_log_in_window(store,r->rx_ms))continue;
        fprintf(f,"%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,",(unsigned long)store->esp_session,(unsigned long)r->rx_ms,vcu_log_u16(d+2),vcu_log_u16(d),d[4]&3U,(d[4]>>2)&3U,(d[4]>>4)&1U,d[5],d[6],d[7]);hex(f,d,8);fputc('\n',f);
    }
    if(!close_csv(f))return false;
    f=open_csv("raw_events.csv");if(!f)return false;
    fprintf(f,"esp_session,esp_rx_dequeue_ms,event_seq,can_id_hex,controller_sample_ms,raw_hex\n");
    start=store->raw_total>VCU_LOG_RAW_ROWS?store->raw_total-VCU_LOG_RAW_ROWS:0;
    for(n=start;n<store->raw_total;n++) {
        const vcu_log_raw_t *r=&store->raw[n%VCU_LOG_RAW_ROWS];if(!vcu_log_in_window(store,r->rx_ms))continue;
        fprintf(f,"%lu,%lu,%lu,%08lX,,",(unsigned long)store->esp_session,(unsigned long)r->rx_ms,(unsigned long)n,(unsigned long)r->id);hex(f,r->data,8);fputc('\n',f);
    }
    if(!close_csv(f))return false;
    portENTER_CRITICAL(&lock);memcpy(stats,bus,sizeof(stats));portEXIT_CRITICAL(&lock);
    f=open_csv("metadata.csv");if(!f)return false;
    fprintf(f,"key,value\nsource,%s\nlog_build,%s\nesp_firmware_base,%d.%d\ncontroller_firmware,unknown_field_confirmation_required\ncontroller_sample_clock,not_in_protocol\nwindow_clock,ESP_driver_dequeue_ms\nesp_session,%lu\ntrigger_ms,%lu\npre_requested_ms,5000\npost_requested_ms,3000\nperiodic_sample_ms,100\nstore_bytes,%u\nheap_free_bytes,%u\nheap_min_bytes,%u\nsnapshot_sequence_gaps,%lu\ntx_sequence_gaps,%lu\nreassembly_incomplete,%lu\nreassembly_invalid,%lu\nsample_ring_overwrites,%lu\nevent_ring_overwrites,%lu\ntx_ring_overwrites,%lu\nraw_ring_overwrites,%lu\nstm_reboots,%lu\n",
      CAPTURE_SOURCE,VCU_LOG_BUILD,FW_VERSION_MAJOR,FW_VERSION_MINOR,(unsigned long)store->esp_session,(unsigned long)store->trigger_ms,(unsigned)sizeof(*store),
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),(unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
      (unsigned long)store->gaps,(unsigned long)store->tx_gaps,(unsigned long)store->decoder.incomplete,(unsigned long)store->decoder.invalid,
      (unsigned long)store->sample_overwrites,(unsigned long)store->event_overwrites,(unsigned long)store->tx_overwrites,(unsigned long)store->raw_overwrites,(unsigned long)store->reboot_count);
    { const char *names[]={"esp_twai_state","esp_tec","esp_rec","esp_rx_missed","esp_rx_overrun","esp_tx_failed","esp_bus_errors","esp_processing_queue_lost"};
      for(n=0;n<8;n++)fprintf(f,"%s,%lu\n",names[n],(unsigned long)stats[n]); }
    fprintf(f,"tx_resets_or_reordered,%lu\n",(unsigned long)store->tx_resets_or_reordered);
    { unsigned used=store->sample_total>VCU_LOG_SAMPLES?VCU_LOG_SAMPLES:store->sample_total;
      unsigned events=store->event_total>VCU_LOG_EVENTS?VCU_LOG_EVENTS:store->event_total;
      fprintf(f,"snapshot_slots_used,%u\nsnapshot_slots_capacity,%u\nevent_slots_used,%u\nevent_slots_capacity,%u\ntx_slots_used,%u\ntx_slots_capacity,%u\nraw_slots_used,%u\nraw_slots_capacity,%u\n",used,VCU_LOG_SAMPLES,events,VCU_LOG_EVENTS,
        (unsigned)(store->tx_total>VCU_LOG_TX_ROWS?VCU_LOG_TX_ROWS:store->tx_total),VCU_LOG_TX_ROWS,
        (unsigned)(store->raw_total>VCU_LOG_RAW_ROWS?VCU_LOG_RAW_ROWS:store->raw_total),VCU_LOG_RAW_ROWS);
    }
    if(!close_csv(f))return false;
    f=open_csv("COMPLETE.txt");if(!f)return false;
    fprintf(f,"All CSV files closed successfully. Original fault capture remains frozen in RAM.\n");return close_csv(f);
}
static void worker(void *arg) {
    (void)arg;
    for(;;) {
        bool run=false;
        portENTER_CRITICAL(&lock);
        vcu_log_store_tick(store,now_ms());
        if(store->frozen && !exporting && ((!auto_attempted)||export_request)) {
            auto_attempted=true;export_request=false;exporting=true;run=true;
        }
        portEXIT_CRITICAL(&lock);
        if(run) {
            bool ok=export_files();
            portENTER_CRITICAL(&lock);exporting=false;exported=ok;if(!ok)file_errors++;portEXIT_CRITICAL(&lock);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
static void terminal_status(int argc,const char **argv) {
    uint32_t counts[10];bool trig,frozen,busy,done; (void)argc;(void)argv;
    if(!ready) { commands_printf("flight logger unavailable: need %u bytes plus 32 KiB headroom; free=%u",(unsigned)sizeof(*store),(unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));return; }
    portENTER_CRITICAL(&lock);
    counts[0]=store->sample_total;counts[1]=store->event_total;counts[2]=store->tx_total;counts[3]=store->raw_total;
    counts[4]=store->gaps;counts[5]=store->tx_gaps;counts[6]=store->decoder.incomplete;counts[7]=store->decoder.invalid;
    counts[8]=file_errors;counts[9]=store->trigger_ms;
    trig=store->triggered;frozen=store->frozen;busy=exporting;done=exported;
    portEXIT_CRITICAL(&lock);
    commands_printf("%s ready=1 triggered=%d frozen=%d exporting=%d exported=%d\nsamples=%lu events=%lu tx=%lu raw=%lu gaps=%lu/%lu incomplete=%lu invalid=%lu file_errors=%lu trigger_esp_ms=%lu\nstore=%u bytes free_heap=%u path=%s",
      VCU_LOG_BUILD,trig,frozen,busy,done,(unsigned long)counts[0],(unsigned long)counts[1],(unsigned long)counts[2],(unsigned long)counts[3],(unsigned long)counts[4],(unsigned long)counts[5],(unsigned long)counts[6],(unsigned long)counts[7],(unsigned long)counts[8],(unsigned long)counts[9],(unsigned)sizeof(*store),(unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),last_path);
}
static void terminal_trigger(int argc,const char **argv) {
    (void)argc;(void)argv;if(!ready)return;
    portENTER_CRITICAL(&lock);vcu_log_store_trigger(store,now_ms());export_request=true;portEXIT_CRITICAL(&lock);
    commands_printf("Capture requested; export after 3 s post window. No motor command sent.");
}
static void terminal_rearm(int argc,const char **argv) {
    bool ok=false;(void)argc;(void)argv;if(!ready)return;
    portENTER_CRITICAL(&lock);
    if(!exporting && (exported || !store->triggered)) {
        uint32_t session=store->esp_session;
        /* Reset only metadata, not the 44 KiB backing arrays under the lock. */
        memset(&store->decoder,0,sizeof(*store)-offsetof(vcu_log_store_t,decoder));store->esp_session=session;
        export_request=exported=auto_attempted=false;ok=true;
    }
    portEXIT_CRITICAL(&lock);
    commands_printf(ok?"Flight recorder rearmed":"Not rearmed: export the retained capture successfully first");
}
void vcu_flight_log_init(void) {
    terminal_register_command_callback("vcu_log_status","Flight recorder state and export path",0,terminal_status);
    terminal_register_command_callback("vcu_log_export","Freeze after 3 seconds and export CSV (or retry failed export)",0,terminal_trigger);
    terminal_register_command_callback("vcu_log_rearm","Rearm after a successful export",0,terminal_rearm);
#ifdef HW_VCU_STM32_GATEWAY
    /* Leave headroom for file download, Wi-Fi buffers and existing tasks. */
    if(heap_caps_get_free_size(MALLOC_CAP_8BIT)<sizeof(*store)+32768U+5120U) return;
    store=calloc(1,sizeof(*store));
    if(!store)return;
    vcu_log_store_init(store,esp_random());
    if(xTaskCreate(worker,"vcu_flight_log",4096,NULL,2,NULL)!=pdPASS) { free(store);store=NULL;return; }
    ready=true;
#endif
}
