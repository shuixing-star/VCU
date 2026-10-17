#ifndef VCU_K2_FAULT_H
#define VCU_K2_FAULT_H
#include <stdint.h>
#include <stdbool.h>
#define K2_FAULT_PARTS 27U
#define K2_FAULT_WORDS 80U
typedef struct {
    uint16_t words[K2_FAULT_WORDS];
    uint32_t rx_ms, fault_ms, epoch;
    unsigned side;
} vcu_k2_fault_event_t;
typedef struct {
    uint16_t words[K2_FAULT_WORDS], token;
    uint32_t mask, first_ms, last_uptime, epoch;
    bool active, have_uptime;
    uint32_t seen_id[8], seen_seq[8];
    unsigned seen_count, seen_next;
    uint32_t crc_errors, timeouts, duplicates;
} vcu_k2_fault_rx_t;
typedef struct {
    const char *name, *unit;
    unsigned word, width, is_signed, decimals;
    double scale;
} vcu_k2_fault_column_t;
extern const vcu_k2_fault_column_t vcu_k2_fault_columns[];
extern const unsigned vcu_k2_fault_column_count;
bool vcu_k2_fault_route(uint32_t id, bool extended, unsigned *side, unsigned *part);
bool vcu_k2_fault_accept(vcu_k2_fault_rx_t *r, unsigned side, unsigned part,
    const uint8_t *data, uint8_t len, uint32_t now, vcu_k2_fault_event_t *out);
void vcu_k2_fault_expire(vcu_k2_fault_rx_t *r,uint32_t now);
uint32_t vcu_k2_fault_u32(const uint16_t *w,unsigned at);
double vcu_k2_fault_value(const uint16_t *w,const vcu_k2_fault_column_t *c);
#endif
