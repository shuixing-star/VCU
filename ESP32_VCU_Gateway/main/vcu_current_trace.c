#include "vcu_current_trace.h"
#include <string.h>
static uint32_t trace_u32(const uint16_t *w,unsigned at) {
    return (uint32_t)w[at]|((uint32_t)w[at+1]<<16);
}
bool vcu_current_trace_route(uint32_t id,bool ext,unsigned *side,unsigned *part) {
    unsigned n=id&255U,p=(id>>16)&255U;
    if(!ext || (n!=239U && n!=240U) || p>=VCU_CT_PARTS ||
       id!=(0x1900D100UL|((uint32_t)p<<16)|n)) return false;
    *side=n-239U;*part=p;return true;
}
static uint16_t trace_crc(const uint16_t *w,unsigned words) {
    uint16_t crc=0xFFFFU;
    for(unsigned i=0;i<words-1U;i++) for(unsigned b=0;b<2;b++) {
        crc^=(uint16_t)(((w[i]>>(b*8))&255U)<<8);
        for(unsigned k=0;k<8;k++) crc=(crc&0x8000U)?(uint16_t)((crc<<1)^0x1021U):(uint16_t)(crc<<1);
    }
    return crc;
}
bool vcu_current_trace_accept(vcu_current_trace_rx_t *r,unsigned side,unsigned part,
    const uint8_t *d,uint8_t len,uint32_t now,vcu_current_trace_event_t *out) {
    if(!r || !d || !out || side>1U || part>=VCU_CT_PARTS || len!=8U) return false;
    if(r->active && (uint32_t)(now-r->first_ms)>1000U) {r->active=false;r->mask=0;r->partial++;}
    uint16_t token=(uint16_t)(d[0]|((uint16_t)d[1]<<8));
    if(!r->active || r->token!=token) {
        if(part!=0U)return false;
        if(r->active && r->mask)r->partial++;
        r->active=true;r->token=token;r->first_ms=now;r->mask=0;
    }
    if(r->mask&(1UL<<part))return false;
    for(unsigned i=0;i<3;i++) r->w[part*3+i]=(uint16_t)(d[2+i*2]|((uint16_t)d[3+i*2]<<8));
    r->mask|=1UL<<part;
    unsigned parts=r->w[1]==1U?20U:(r->w[1]==3U?22U:VCU_CT_PARTS);
    if(r->mask!=((1UL<<parts)-1U))return false;
    r->active=false;r->mask=0;
    if(r->w[0]!=0x4B54U || (r->w[1]!=1U && r->w[1]!=2U && r->w[1]!=3U) || r->w[13]==0U || r->w[13]>16U ||
       r->w[12]>=r->w[13] || trace_crc(r->w,parts*3U)!=r->w[parts*3U-1U]) {r->crc_errors++;return false;}
    uint32_t uptime=trace_u32(r->w,6);
    if(r->have_uptime && (int32_t)(uptime-r->last_uptime)<0) {
        r->epoch++;r->seen_count=r->seen_next=0;
    }
    r->last_uptime=uptime;r->have_uptime=true;
    uint32_t id=trace_u32(r->w,2),seq=trace_u32(r->w,4);
    for(unsigned i=0;i<r->seen_count;i++)
        if(r->seen_id[i]==id && r->seen_seq[i]==seq && r->seen_row[i]==r->w[12]) {r->duplicates++;return false;}
    unsigned at=r->seen_next;
    r->seen_id[at]=id;r->seen_seq[at]=seq;r->seen_row[at]=r->w[12];
    r->seen_next=(at+1U)%16U;if(r->seen_count<16U)r->seen_count++;
    memcpy(out->w,r->w,sizeof out->w);out->side=side;out->epoch=r->epoch;out->rx_ms=now;
    out->fault_ms=r->first_ms-(uint32_t)(uptime-seq)/10U;
    return true;
}
