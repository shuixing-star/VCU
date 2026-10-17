#ifndef VCU_CURRENT_TRACE_H
#define VCU_CURRENT_TRACE_H
#include <stdint.h>
#include <stdbool.h>
#define VCU_CT_WORDS 72U
#define VCU_CT_PARTS 24U
typedef struct {
    uint16_t w[VCU_CT_WORDS];
    uint32_t fault_ms, rx_ms, epoch;
    unsigned side;
} vcu_current_trace_event_t;
typedef struct {
    uint16_t w[VCU_CT_WORDS], token;
    uint32_t mask, first_ms, crc_errors, partial, duplicates, epoch, last_uptime;
    uint32_t seen_id[16], seen_seq[16];
    uint16_t seen_row[16];
    unsigned seen_count, seen_next;
    bool active, have_uptime;
} vcu_current_trace_rx_t;
bool vcu_current_trace_route(uint32_t id, bool ext, unsigned *side, unsigned *part);
bool vcu_current_trace_accept(vcu_current_trace_rx_t *r,unsigned side,unsigned part,
    const uint8_t *d,uint8_t len,uint32_t now,vcu_current_trace_event_t *out);
#endif
