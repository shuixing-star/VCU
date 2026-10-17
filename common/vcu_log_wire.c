#include "vcu_log_wire.h"
#include <string.h>
uint16_t vcu_log_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
uint32_t vcu_log_u32(const uint8_t *p) { return (uint32_t)vcu_log_u16(p) | ((uint32_t)vcu_log_u16(p + 2) << 16); }
void vcu_log_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
void vcu_log_put32(uint8_t *p, uint32_t v) { vcu_log_put16(p, (uint16_t)v); vcu_log_put16(p + 2, (uint16_t)(v >> 16)); }
uint16_t vcu_log_crc(const uint8_t *p, size_t n) {
    uint16_t crc = 0xffffU;
    size_t i;
    unsigned b;
    for (i = 0; i < n; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (b = 0; b < 8; ++b) crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
    return crc;
}
bool vcu_log_valid(const uint8_t *p) {
    return p[0] == 0xa7U && p[1] == VCU_LOG_SCHEMA &&
           vcu_log_u16(p + 174) == vcu_log_crc(p, 174);
}
void vcu_log_fragment(const uint8_t *p, uint8_t part, uint8_t out[8]) {
    unsigned i;
    out[0] = p[4]; out[1] = part;
    for (i = 0; i < 6; ++i) out[i + 2] = part * 6U + i < VCU_LOG_SIZE ? p[part * 6U + i] : 0U;
}
bool vcu_log_decode(vcu_log_decoder_t *d, const uint8_t data[8], uint32_t now) {
    unsigned i;
    const uint8_t part = data[1];
    if (part >= VCU_LOG_PARTS) { d->invalid++; return false; }
    if (d->active && (uint32_t)(now - d->started_ms) > 500U) { d->incomplete++; d->active = false; }
    if (part == 0U) {
        if (d->active) d->incomplete++;
        d->active = true; d->token = data[0]; d->mask = 0; d->started_ms = now;
    }
    if (!d->active || data[0] != d->token) { d->invalid++; return false; }
    for (i = 0; i < 6; ++i) if (part * 6U + i < VCU_LOG_SIZE) d->record[part * 6U + i] = data[i + 2];
    d->mask |= 1UL << part;
    if (d->mask != ((1UL << VCU_LOG_PARTS) - 1U)) return false;
    d->active = false;
    if (!vcu_log_valid(d->record) || d->record[4] != d->token) { d->invalid++; return false; }
    return true;
}
