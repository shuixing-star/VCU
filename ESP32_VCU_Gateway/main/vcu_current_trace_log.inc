/* Included by vcu_fault_log.c; shared recording lifecycle and lock.
 * No SD I/O on CAN thread. Bounded row queue; gaps remain identifiable by row/rows. */
#include "vcu_current_trace.h"
#include <math.h>
#define GW_TRACE_QUEUE 32U
static vcu_current_trace_rx_t gw_trace_rx[2];
static vcu_current_trace_event_t gw_trace_queue[GW_TRACE_QUEUE];
static unsigned gw_trace_head,gw_trace_tail,gw_trace_count;
static uint32_t gw_trace_drops;
static FILE *gw_trace_file;
static char gw_trace_buffer[2048];
typedef struct {
    const char *name,*unit;
    unsigned word,width,sign,decimals; double scale; unsigned valid; int legacy_word;
} gw_trace_column_t;
static const gw_trace_column_t gw_trace_columns[]={
    {"trace_version","",1,1,0,0,1.0,0,1},
    {"event_id","",2,2,0,0,1.0,0,2},
    {"trigger_ticks","",4,2,0,0,1.0,0,4},
    {"transmit_ticks","",6,2,0,0,1.0,0,6},
    {"firmware","",8,2,0,0,1.0,0,8},
    {"controller_trace_drops","",10,2,0,0,1.0,0,10},
    {"row","",12,1,0,0,1.0,0,12},
    {"rows","",13,1,0,0,1.0,0,13},
    {"fault","",14,1,0,0,1.0,0,14},
    {"flags","",15,1,0,0,1.0,0,15},
    {"sample_ticks","",16,2,0,0,1.0,0,16},
    {"entry_timer","90MHz_ticks",18,2,0,0,1.0,0,18},
    {"adc_read_timer","90MHz_ticks",20,2,0,0,1.0,0,52},
    {"write_end_timer","90MHz_ticks",22,2,0,0,1.0,2,22},
    {"entry_tbctr","ticks",24,1,0,0,1.0,0,24},
    {"write_end_tbctr","ticks",25,1,0,0,1.0,2,26},
    {"adc_a","counts",26,1,0,0,1.0,0,27},
    {"adc_b","counts",27,1,0,0,1.0,0,28},
    {"hall_state","",28,1,0,0,1.0,0,29},
    {"hall_previous_state","",29,1,0,0,1.0,0,-1},
    {"hall_transition_flags","",30,1,0,0,1.0,0,-1},
    {"cmp_a_written","ticks",31,1,0,0,1.0,2,30},
    {"cmp_b_written","ticks",32,1,0,0,1.0,2,31},
    {"cmp_c_written","ticks",33,1,0,0,1.0,2,32},
    {"ia","A",34,1,1,1,10.0,0,33},
    {"ib","A",35,1,1,1,10.0,0,34},
    {"id_ref","A",37,1,1,1,10.0,1,36},
    {"iq_ref","A",38,1,1,1,10.0,1,37},
    {"id","A",39,1,1,1,10.0,1,38},
    {"iq","A",40,1,1,1,10.0,1,39},
    {"vbus","V",36,1,1,2,100.0,0,35},
    {"vd_raw","V",41,1,1,2,100.0,1,42},
    {"vq_raw","V",42,1,1,2,100.0,1,43},
    {"vd","V",43,1,1,2,100.0,1,46},
    {"vq","V",44,1,1,2,100.0,1,47},
    {"v_alpha_cmd","V",45,1,1,2,100.0,1,-1},
    {"v_beta_cmd","V",46,1,1,2,100.0,1,-1},
    {"voltage_limit","V",47,1,1,2,100.0,1,56},
    {"hall_raw_angle_deg","deg",48,1,1,2,100.0,0,-1},
    {"hall_angle_deg","deg",49,1,1,2,100.0,0,-1},
    {"control_angle_deg","deg",50,1,1,2,100.0,1,-2},
    {"hall_raw_rpm","rpm",51,2,1,1,10.0,0,59},
    {"hall_filtered_rpm","rpm",53,2,1,1,10.0,0,63},
    {"hall_period_ms","ms",55,2,0,1,10.0,0,65},
    {"hall_edge_age_ms","ms",57,2,0,1,10.0,0,67},
    {"hall_flags","",59,1,0,0,1.0,0,69},
    {"hall_pole_pairs","",60,1,0,2,100.0,0,70},
    {"adc_overflows","",61,2,0,0,1.0,0,50},
    {"shadow_full","entryABC_endABC",63,1,0,0,1.0,0,57},
    {"encoding_flags","",64,1,0,0,1.0,0,58},
};
#define GW_TRACE_COLUMNS (sizeof(gw_trace_columns)/sizeof(gw_trace_columns[0]))
static void trace_log_begin_locked(void) {
    memset(gw_trace_rx,0,sizeof gw_trace_rx);
    gw_trace_head=gw_trace_tail=gw_trace_count=0;gw_trace_drops=0;
}
static bool trace_log_receive(uint32_t id,const uint8_t *d,uint8_t len,bool ext,uint32_t now) {
    unsigned side,part;
    if(!vcu_current_trace_route(id,ext,&side,&part))return false;
    vcu_current_trace_event_t e;
    portENTER_CRITICAL(&m_fault_lock);
    if(m_fault_accepting && vcu_current_trace_accept(&gw_trace_rx[side],side,part,d,len,now,&e) &&
       (int32_t)(e.fault_ms-m_fault_start_ms)>=0 &&
       (!m_fault_closing || (int32_t)(e.fault_ms-m_fault_stop_ms)<=0)) {
        if(gw_trace_count<GW_TRACE_QUEUE) {
            gw_trace_queue[gw_trace_tail]=e;gw_trace_tail=(gw_trace_tail+1U)%GW_TRACE_QUEUE;gw_trace_count++;
        } else gw_trace_drops++;
    }
    portEXIT_CRITICAL(&m_fault_lock);return true;
}
static bool trace_log_open(const char *path) {
    char trace_path[208];size_t n=strlen(path);
    if(n<4U || n+16U>=sizeof trace_path) {errno=EINVAL;return false;}
    memcpy(trace_path,path,n-4U);strcpy(trace_path+n-4U,"_current_trace.csv");
    gw_trace_file=fopen(trace_path,"w");if(!gw_trace_file)return false;
    if(setvbuf(gw_trace_file,gw_trace_buffer,_IOFBF,sizeof gw_trace_buffer))return false;
    if(fprintf(gw_trace_file,"t_day:Estimated fault time:s:3:0:0;record_kind:1 row 0 summary::0:0:0;node:K2 node::0:0:0;epoch:Controller epoch::0:0:0;queue_drops:Gateway drops::0:0:0;crc_errors:CRC errors::0:0:0;partial_packets:Incomplete::0:0:0")<0)return false;
    for(unsigned i=0;i<GW_TRACE_COLUMNS;i++) {
        const gw_trace_column_t *c=&gw_trace_columns[i];
        if(fprintf(gw_trace_file,";%s:%s:%s:%u:0:0",c->name,c->name,c->unit,c->decimals)<0)return false;
    }
    return fprintf(gw_trace_file,"\n")>=0 && fflush(gw_trace_file)==0;
}
static bool trace_log_drain(void) {
    bool wrote=false;
    for(;;) {
        vcu_current_trace_event_t e;uint32_t drops,crc,partial;
        portENTER_CRITICAL(&m_fault_lock);
        bool present=gw_trace_count!=0;
        if(present)e=gw_trace_queue[gw_trace_head];
        drops=gw_trace_drops;crc=gw_trace_rx[0].crc_errors+gw_trace_rx[1].crc_errors;
        partial=gw_trace_rx[0].partial+gw_trace_rx[1].partial;
        portEXIT_CRITICAL(&m_fault_lock);
        if(!present)break;
        if(!gw_trace_file) {errno=EIO;return false;}
        if(fprintf(gw_trace_file,"%.3f;1;%u;%lu;%lu;%lu;%lu",(double)e.fault_ms/1000.0,239U+e.side,
            (unsigned long)e.epoch,(unsigned long)drops,(unsigned long)crc,(unsigned long)partial)<0)return false;
        for(unsigned i=0;i<GW_TRACE_COLUMNS;i++) {
            const gw_trace_column_t *c=&gw_trace_columns[i];
            bool modern=e.w[1]>=3U;
            int at=modern?(int)c->word:c->legacy_word;
            unsigned hall_flags=e.w[1]==1U?0U:e.w[modern?59U:69U];
            bool missing=at==-1 || (c->valid && !(e.w[15]&c->valid));
            if(c->word>=51U && c->word<=60U && e.w[1]==1U)missing=true;
            if((c->word==48U || c->word==49U) && !(hall_flags&8U))missing=true;
            if(c->word==51U && (!(hall_flags&2U) || (hall_flags&128U)))missing=true;
            if(c->word==53U && (hall_flags&128U))missing=true;
            if(missing) {if(fputc(';',gw_trace_file)==EOF)return false;continue;}
            double value;
            if(at==-2) { /* V1/V2 stored sine/cosine; same PI-valid guard. */
                value=atan2((double)(int16_t)e.w[48],(double)(int16_t)e.w[49])*5729.577951308;
            } else value=c->width==2U?(c->sign?(double)(int32_t)vcu_k2_fault_u32(e.w,(unsigned)at):(double)vcu_k2_fault_u32(e.w,(unsigned)at)):
                (c->sign?(double)(int16_t)e.w[at]:(double)e.w[at]);
            if(fprintf(gw_trace_file,";%.*f",(int)c->decimals,value/c->scale)<0)return false;
        }
        if(fputc('\n',gw_trace_file)==EOF)return false;
        portENTER_CRITICAL(&m_fault_lock);
        gw_trace_head=(gw_trace_head+1U)%GW_TRACE_QUEUE;gw_trace_count--;
        portEXIT_CRITICAL(&m_fault_lock);wrote=true;
    }
    return !wrote || (fflush(gw_trace_file)==0 && !ferror(gw_trace_file));
}
static bool trace_log_finish(void) {
    bool ok=trace_log_drain();
    if(gw_trace_file) {
        for(unsigned i=0;i<2;i++)if(gw_trace_rx[i].active && gw_trace_rx[i].mask)gw_trace_rx[i].partial++;
        if(fprintf(gw_trace_file,"%.3f;0;0;0;%lu;%lu;%lu",(double)utils_ms_tot()/1000.0,
           (unsigned long)gw_trace_drops,(unsigned long)(gw_trace_rx[0].crc_errors+gw_trace_rx[1].crc_errors),
           (unsigned long)(gw_trace_rx[0].partial+gw_trace_rx[1].partial))<0)ok=false;
        for(unsigned i=0;i<GW_TRACE_COLUMNS;i++)if(fputc(';',gw_trace_file)==EOF)ok=false;
        if(fputc('\n',gw_trace_file)==EOF || fflush(gw_trace_file)!=0 || ferror(gw_trace_file))ok=false;
        if(fclose(gw_trace_file)!=0)ok=false;
        gw_trace_file=NULL;
    }
    return ok;
}
