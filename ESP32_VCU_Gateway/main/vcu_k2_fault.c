#include "vcu_k2_fault.h"
#include <string.h>
uint32_t vcu_k2_fault_u32(const uint16_t *w,unsigned at) {
    return (uint32_t)w[at]|((uint32_t)w[at+1]<<16);
}
bool vcu_k2_fault_route(uint32_t id,bool extended,unsigned *side,unsigned *part) {
    if(!extended) return false;
    unsigned n=id&255U,p=(id>>16)&255U;
    if((n!=239U && n!=240U) || p>=K2_FAULT_PARTS ||
        id!=(0x1900D000UL|((uint32_t)p<<16)|n)) return false;
    *side=n-239U;*part=p;return true;
}
static uint16_t crc_words(const uint16_t *w) {
    uint16_t crc=0xFFFFU;
    for(unsigned i=0;i<79;i++) for(unsigned b=0;b<2;b++) {
        crc^=(uint16_t)(((w[i]>>(b*8))&255U)<<8);
        for(unsigned k=0;k<8;k++)crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
    }
    return crc;
}
void vcu_k2_fault_expire(vcu_k2_fault_rx_t *r,uint32_t now) {
    if(r->active && (uint32_t)(now-r->first_ms)>1000U) {
        r->active=false;r->mask=0;r->timeouts++;
    }
}
bool vcu_k2_fault_accept(vcu_k2_fault_rx_t *r,unsigned side,unsigned part,
    const uint8_t *d,uint8_t len,uint32_t now,vcu_k2_fault_event_t *out) {
    if(!d || !out || side>1 || part>=K2_FAULT_PARTS || len!=8) return false;
    vcu_k2_fault_expire(r,now);
    uint16_t token=(uint16_t)(d[0]|((uint16_t)d[1]<<8));
    if(!r->active || r->token!=token) {
        if(part!=0) return false;
        if(r->active && r->mask) r->timeouts++;
        r->active=true;r->token=token;r->first_ms=now;r->mask=0;
    }
    if(r->mask&(1UL<<part)) return false;
    for(unsigned i=0;i<3;i++) {
        unsigned at=part*3+i;
        if(at<K2_FAULT_WORDS)r->words[at]=(uint16_t)(d[2+i*2]|((uint16_t)d[3+i*2]<<8));
    }
    r->mask|=1UL<<part;
    if(r->mask!=((1UL<<K2_FAULT_PARTS)-1U))return false;
    r->active=false;r->mask=0;
    if(r->words[0]!=0x4B32 || r->words[1]!=1 || crc_words(r->words)!=r->words[79]) {
        r->crc_errors++;return false;
    }
    uint32_t uptime=vcu_k2_fault_u32(r->words,8);
    /* Complete packets from one CAN sender are serialized. A backwards
     * uptime (not natural uint32 wrap) starts a new controller epoch. */
    if(r->have_uptime && (int32_t)(uptime-r->last_uptime)<0) {
        r->seen_count=r->seen_next=0;r->epoch++;
    }
    r->last_uptime=uptime;r->have_uptime=true;
    uint32_t id=vcu_k2_fault_u32(r->words,2),seq=vcu_k2_fault_u32(r->words,4);
    for(unsigned i=0;i<r->seen_count;i++)
        if(r->seen_id[i]==id && r->seen_seq[i]==seq) {r->duplicates++;return false;}
    r->seen_id[r->seen_next]=id;r->seen_seq[r->seen_next]=seq;
    r->seen_next=(r->seen_next+1U)%8U;if(r->seen_count<8U)r->seen_count++;
    memcpy(out->words,r->words,sizeof out->words);
    out->rx_ms=now;out->side=side;out->epoch=r->epoch;
    /* Approximate gateway event time; CAN queue delay is not a shared clock. */
    out->fault_ms=r->first_ms-(uint32_t)(uptime-seq)/10U;
    return true;
}
double vcu_k2_fault_value(const uint16_t *w,const vcu_k2_fault_column_t *c) {
    if(c->width==2U)return (double)vcu_k2_fault_u32(w,c->word)/c->scale;
    return (c->is_signed?(double)(int16_t)w[c->word]:(double)w[c->word])/c->scale;
}

const vcu_k2_fault_column_t vcu_k2_fault_columns[]={
    {"event_id","",2,2,0,0,1.0},
    {"trigger_ticks","",4,2,0,0,1.0},
    {"control_ticks","",6,2,0,0,1.0},
    {"transmit_ticks","",8,2,0,0,1.0},
    {"firmware","",10,2,0,0,1.0},
    {"fault","",12,1,0,0,1.0},
    {"state","",13,1,0,0,1.0},
    {"hall_state","",14,1,0,0,1.0},
    {"hall_valid","",15,1,0,0,1.0},
    {"speed_valid","",16,1,0,0,1.0},
    {"angle_valid","",17,1,0,0,1.0},
    {"hall_direction","",18,1,1,0,1.0},
    {"hall_mode","",19,1,0,0,1.0},
    {"startup","",20,1,0,0,1.0},
    {"limit_flags","",21,1,0,0,1.0},
    {"q_clip","",22,1,1,0,1.0},
    {"adc_a","",23,1,0,0,1.0},
    {"adc_b","",24,1,0,0,1.0},
    {"wrong_direction_ticks","",25,1,0,0,1.0},
    {"startup_reverse_ticks","",26,1,0,0,1.0},
    {"gpio_low","",27,1,0,0,1.0},
    {"ost_at_capture","",28,1,0,0,1.0},
    {"overcurrent_ticks","",29,2,0,0,1.0},
    {"trip_duration_ticks","",31,2,0,0,1.0},
    {"hall_period_ticks","",33,2,0,0,1.0},
    {"hall_edge_age_ticks","",35,2,0,0,1.0},
    {"adc_overflows","",37,2,0,0,1.0},
    {"controller_queue_drops","",39,2,0,0,1.0},
    {"ia","A",41,1,1,1,10.0},
    {"ib","A",42,1,1,1,10.0},
    {"ic","A",43,1,1,1,10.0},
    {"id_last","A",44,1,1,1,10.0},
    {"iq_last","A",45,1,1,1,10.0},
    {"id_reference","A",46,1,1,1,10.0},
    {"iq_reference","A",47,1,1,1,10.0},
    {"iq_requested","A",48,1,1,1,10.0},
    {"speed_integral","A",49,1,1,1,10.0},
    {"phase_peak","A",50,1,0,1,10.0},
    {"instant_limit","A",51,1,0,1,10.0},
    {"continuous_limit","A",52,1,0,1,10.0},
    {"vbus","V",53,1,0,1,10.0},
    {"vd_last","V",54,1,1,1,10.0},
    {"vq_last","V",55,1,1,1,10.0},
    {"voltage_limit","V",56,1,0,1,10.0},
    {"raw_target_rpm","rpm",57,1,1,0,1.0},
    {"raw_reference_rpm","rpm",58,1,1,0,1.0},
    {"raw_actual_rpm","rpm",59,1,1,0,1.0},
    {"hall_raw_angle","deg",60,1,1,2,100.0},
    {"hall_angle","deg",61,1,1,2,100.0},
    {"control_angle_last","deg",62,1,1,2,100.0},
    {"hall_hz","Hz",63,1,0,1,10.0},
    {"hall_nominal_step","rad",64,1,0,4,10000.0},
    {"duty_a","",65,1,0,4,10000.0},
    {"duty_b","",66,1,0,4,10000.0},
    {"duty_c","",67,1,0,4,10000.0},
    {"pole_pairs","",68,1,0,2,100.0},
    {"first_fault","",69,1,0,0,1.0},
    {"first_trigger_ticks","",70,2,0,0,1.0},
    {"run_fault_count","",72,2,0,0,1.0},
    {"encoding_flags","",74,1,0,0,1.0},
};
const unsigned vcu_k2_fault_column_count=sizeof(vcu_k2_fault_columns)/sizeof(vcu_k2_fault_columns[0]);
