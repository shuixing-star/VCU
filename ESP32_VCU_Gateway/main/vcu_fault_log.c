/* Only the CAN callback touches assembly; only the log task touches FILE.
 * Critical sections copy bounded records, never write to the SD card. */
#include "vcu_fault_log.h"
#include "vcu_k2_fault.h"
#include "utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define FAULT_QUEUE_LENGTH 16U
#define FAULT_CLOSE_GRACE_MS 6000U
static portMUX_TYPE m_fault_lock=portMUX_INITIALIZER_UNLOCKED;
static vcu_k2_fault_rx_t m_fault_rx[2];
static vcu_k2_fault_event_t m_fault_queue[FAULT_QUEUE_LENGTH];
static unsigned m_fault_head,m_fault_tail,m_fault_count;
static bool m_fault_accepting,m_fault_closing;
static uint32_t m_fault_start_ms,m_fault_stop_ms,m_fault_drops;
static FILE *m_fault_file;
static char m_fault_buffer[2048];
#include "vcu_current_trace_log.inc"
void vcu_fault_log_begin(uint32_t now) {
    portENTER_CRITICAL(&m_fault_lock);
    memset(m_fault_rx,0,sizeof m_fault_rx);
    trace_log_begin_locked();
    m_fault_head=m_fault_tail=m_fault_count=0;m_fault_drops=0;
    m_fault_start_ms=now;m_fault_stop_ms=0;
    m_fault_closing=false;m_fault_accepting=true;
    portEXIT_CRITICAL(&m_fault_lock);
}
void vcu_fault_log_stop(uint32_t now) {
    portENTER_CRITICAL(&m_fault_lock);
    if(m_fault_accepting && !m_fault_closing) {m_fault_closing=true;m_fault_stop_ms=now;}
    portEXIT_CRITICAL(&m_fault_lock);
}
bool vcu_fault_log_receive(uint32_t id,const uint8_t *data,uint8_t len,bool ext,uint32_t now) {
    unsigned side,part;
    if(!vcu_k2_fault_route(id,ext,&side,&part))return trace_log_receive(id,data,len,ext,now);
    vcu_k2_fault_event_t event;
    portENTER_CRITICAL(&m_fault_lock);
    if(m_fault_accepting && vcu_k2_fault_accept(&m_fault_rx[side],side,part,data,len,now,&event)) {
        if((int32_t)(event.fault_ms-m_fault_start_ms)>=0 &&
           (!m_fault_closing || (int32_t)(event.fault_ms-m_fault_stop_ms)<=0)) {
            if(m_fault_count<FAULT_QUEUE_LENGTH) {
                m_fault_queue[m_fault_tail]=event;
                m_fault_tail=(m_fault_tail+1U)%FAULT_QUEUE_LENGTH;m_fault_count++;
            } else m_fault_drops++;
        }
    }
    portEXIT_CRITICAL(&m_fault_lock);
    return true;
}
bool vcu_fault_log_open(const char *main_path) {
    char path[192];
    size_t n=strlen(main_path);
    if(n<4U || strcmp(main_path+n-4U,".csv") || n+8U>=sizeof(path)) {errno=EINVAL;return false;}
    memcpy(path,main_path,n-4U);strcpy(path+n-4U,"_faults.csv");
    m_fault_file=fopen(path,"w");
    if(!m_fault_file)return false;
    if(setvbuf(m_fault_file,m_fault_buffer,_IOFBF,sizeof m_fault_buffer))return false;
    if(fprintf(m_fault_file,"t_day:Estimated event time:s:3:0:1;record_kind:1 event 0 summary::0:0:0;node:K2 CAN node::0:0:0;rx_ms:Gateway RX:ms:0:0:0;epoch:Controller epoch::0:0:0;queue_drops:Gateway drops::0:0:0;crc_errors:RX CRC errors::0:0:0;partial_packets:Incomplete packets::0:0:0")<0)return false;
    for(unsigned i=0;i<vcu_k2_fault_column_count;i++) {
        const vcu_k2_fault_column_t *c=&vcu_k2_fault_columns[i];
        if(fprintf(m_fault_file,";%s:%s:%s:%u:0:0",c->name,c->name,c->unit,c->decimals)<0)return false;
    }
    return fprintf(m_fault_file,"\n")>=0 && fflush(m_fault_file)==0 && trace_log_open(main_path);
}
bool vcu_fault_log_drain(void) {
    bool wrote=false;
    for(;;) {
        vcu_k2_fault_event_t event;
        uint32_t drops,crc,partial;
        portENTER_CRITICAL(&m_fault_lock);
        bool present=m_fault_count!=0;
        if(present)event=m_fault_queue[m_fault_head];
        drops=m_fault_drops;
        crc=m_fault_rx[0].crc_errors+m_fault_rx[1].crc_errors;
        partial=m_fault_rx[0].timeouts+m_fault_rx[1].timeouts;
        portEXIT_CRITICAL(&m_fault_lock);
        if(!present)break;
        if(!m_fault_file) {errno=EIO;return false;}
        if(fprintf(m_fault_file,"%.3f;1;%u;%lu;%lu;%lu;%lu;%lu",(double)event.fault_ms/1000.0,
                   239U+event.side,(unsigned long)event.rx_ms,(unsigned long)event.epoch,
                   (unsigned long)drops,(unsigned long)crc,(unsigned long)partial)<0)return false;
        for(unsigned i=0;i<vcu_k2_fault_column_count;i++) {
            const vcu_k2_fault_column_t *c=&vcu_k2_fault_columns[i];
            if(fprintf(m_fault_file,";%.*f",(int)c->decimals,vcu_k2_fault_value(event.words,c))<0)return false;
        }
        if(fprintf(m_fault_file,"\n")<0)return false;
        portENTER_CRITICAL(&m_fault_lock);
        m_fault_head=(m_fault_head+1U)%FAULT_QUEUE_LENGTH;m_fault_count--;
        portEXIT_CRITICAL(&m_fault_lock);
        wrote=true;
    }
    return (!wrote || (fflush(m_fault_file)==0 && !ferror(m_fault_file))) && trace_log_drain();
}
bool vcu_fault_log_finish(void) {
    bool ok=true;
    vcu_fault_log_stop((uint32_t)utils_ms_tot());
    if(m_fault_file) {
        /* Allow four queued events (3 x 27 frames each, >=5 ms spacing) to
         * finish/retry, with bus margin, when Stop is
         * clicked during its transfer. No event after the stop boundary is kept. */
        while((uint32_t)((uint32_t)utils_ms_tot()-m_fault_stop_ms)<FAULT_CLOSE_GRACE_MS) {
            if(!vcu_fault_log_drain()) {ok=false;break;}
            vTaskDelay(pdMS_TO_TICKS(10U));
        }
    }
    portENTER_CRITICAL(&m_fault_lock);
    m_fault_accepting=false;
    for(unsigned i=0;i<2;i++) {
        if(m_fault_rx[i].active && m_fault_rx[i].mask) {
            m_fault_rx[i].timeouts++;m_fault_rx[i].active=false;
        }
    }
    portEXIT_CRITICAL(&m_fault_lock);
    if(m_fault_file) {
        if(ok)ok=vcu_fault_log_drain();
        if(ok) {
            /* A summary is explicitly not a motor sample; empty measurement
             * columns prevent zeros from masquerading as fault measurements. */
            ok=fprintf(m_fault_file,"%.3f;0;0;0;0;%lu;%lu;%lu",(double)utils_ms_tot()/1000.0,
                (unsigned long)m_fault_drops,
                (unsigned long)(m_fault_rx[0].crc_errors+m_fault_rx[1].crc_errors),
                (unsigned long)(m_fault_rx[0].timeouts+m_fault_rx[1].timeouts))>=0;
            for(unsigned i=0;i<vcu_k2_fault_column_count && ok;i++)ok=fputc(';',m_fault_file)!=EOF;
            if(ok)ok=fputc('\n',m_fault_file)!=EOF;
        }
        if(fflush(m_fault_file)!=0 || ferror(m_fault_file))ok=false;
        if(fclose(m_fault_file)!=0)ok=false;
        m_fault_file=NULL;
    }
    if(!trace_log_finish())ok=false;
    return ok;
}
