#include "vcu_log_store.h"
#include <string.h>
void vcu_log_store_init(vcu_log_store_t *s, uint32_t session) { memset(s,0,sizeof(*s)); s->esp_session=session; }
void vcu_log_store_trigger(vcu_log_store_t *s,uint32_t now) {
    if(!s->triggered) { s->triggered=true; s->trigger_ms=now; }
}
void vcu_log_store_tick(vcu_log_store_t *s,uint32_t now) {
    if(s->decoder.active && (uint32_t)(now-s->decoder.started_ms)>500U) { s->decoder.incomplete++; s->decoder.active=false; }
    if(s->has_sequence && !s->triggered && (uint32_t)(now-s->last_rx_ms)>750U)
        vcu_log_store_trigger(s,now);
    if(s->triggered && (uint32_t)(now-s->trigger_ms)>=VCU_LOG_POST_MS) s->frozen=true;
}
bool vcu_log_in_window(const vcu_log_store_t *s,uint32_t rx) {
    int32_t delta=(int32_t)(rx-s->trigger_ms);
    return delta>=-(int32_t)VCU_LOG_PRE_MS && delta<=(int32_t)VCU_LOG_POST_MS;
}
void vcu_log_store_frame(vcu_log_store_t *s,uint32_t id,const uint8_t d[8],uint8_t len,bool ext,bool rtr,uint32_t now) {
    bool keep_raw=false;
    int side=-1;
    vcu_log_store_tick(s,now);
    if(s->frozen) { s->frozen_discards++; return; }
    if(rtr || len!=8) return;
    if(!ext && id==VCU_LOG_CAN_ID) {
        if(vcu_log_decode(&s->decoder,d,now)) {
            const uint8_t *p=s->decoder.record;
            uint32_t seq=vcu_log_u32(p+4), nonce=vcu_log_u32(p+12);
            vcu_log_sample_t *row;
            if(s->has_sequence && nonce==s->last_nonce && seq==s->last_sequence) return;
            if(s->has_sequence && (nonce!=s->last_nonce || (uint32_t)(seq-s->last_sequence)>0x80000000UL)) { s->reboot_count++; s->has_sequence=false; s->has_tx_sequence=false; vcu_log_store_trigger(s,now); }
            if(s->has_sequence && seq!=s->last_sequence+1U) s->gaps += seq-s->last_sequence-1U;
            s->has_sequence=true; s->last_sequence=seq; s->last_nonce=nonce; s->last_rx_ms=now;
            if(p[2]==1U) {
                row=&s->samples[s->sample_total++ % VCU_LOG_SAMPLES];
                if(s->sample_total>VCU_LOG_SAMPLES) s->sample_overwrites++;
            } else {
                row=&s->events[s->event_total++ % VCU_LOG_EVENTS];
                if(s->event_total>VCU_LOG_EVENTS) s->event_overwrites++;
            }
            row->rx_ms=now; memcpy(row->data,p,VCU_LOG_SIZE);
            if(p[44]==4U || (vcu_log_u32(p+100)!=0U && !p[106])) vcu_log_store_trigger(s,now);
        }
        return;
    }
    if(!ext && id==VCU_LOG_TX_ID) {
        uint16_t seq=vcu_log_u16(d);
        uint16_t diff=(uint16_t)(seq-(uint16_t)s->last_tx_sequence);
        vcu_log_tx_t *row;
        if(s->has_tx_sequence && diff==0U) return;
        row=&s->tx[s->tx_total++ % VCU_LOG_TX_ROWS];
        if(s->tx_total>VCU_LOG_TX_ROWS) s->tx_overwrites++;
        if(s->has_tx_sequence) {
            if(diff>32768U) s->tx_resets_or_reordered++;
            else s->tx_gaps+=(uint16_t)(diff-1U);
        }
        s->last_tx_sequence=seq; s->has_tx_sequence=true;
        row->rx_ms=now; memcpy(row->data,d,8);
        if(!(d[4]&16U)) vcu_log_store_trigger(s,now);
        return;
    }
    /* Configured field addresses: left EF=239, right F0=240. Preserve the
     * complete raw state frame whenever fault bits change, including clear. */
    if(ext && id==0x1802D0EFUL) side=0;
    if(ext && id==0x1802D0F0UL) side=1;
    if(side>=0 && memcmp(s->last_fault[side],d+3,3)) {
        memcpy(s->last_fault[side],d+3,3); keep_raw=true;
        if(d[3] || (d[4]&7U) || (d[5]&63U)) vcu_log_store_trigger(s,now);
    }
    if(!ext && (id==0x686U || id==0x687U || id==0x688U)) keep_raw=true;
    if(keep_raw) {
        vcu_log_raw_t *row=&s->raw[s->raw_total++ % VCU_LOG_RAW_ROWS];
        if(s->raw_total>VCU_LOG_RAW_ROWS) s->raw_overwrites++;
        row->rx_ms=now; row->id=id; memcpy(row->data,d,8);
    }
}
