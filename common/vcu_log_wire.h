#ifndef VCU_LOG_WIRE_H
#define VCU_LOG_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define VCU_LOG_BUILD "vcu-log-1.0.0"
#define VCU_LOG_BUILD_ID 0x20260921UL
#define VCU_LOG_CAN_ID 0x690U
#define VCU_LOG_TX_ID 0x691U
#define VCU_LOG_SIZE 176U
#define VCU_LOG_PARTS ((VCU_LOG_SIZE + 5U) / 6U)
#define VCU_LOG_SCHEMA 1U
#define VCU_LOG_SAMPLE_MS 100U
#define VCU_LOG_PRE_MS 5000U
#define VCU_LOG_POST_MS 3000U
uint16_t vcu_log_u16(const uint8_t *p);
uint32_t vcu_log_u32(const uint8_t *p);
void vcu_log_put16(uint8_t *p, uint16_t v);
void vcu_log_put32(uint8_t *p, uint32_t v);
uint16_t vcu_log_crc(const uint8_t *p, size_t n);
bool vcu_log_valid(const uint8_t *record);
void vcu_log_fragment(const uint8_t *record, uint8_t part, uint8_t out[8]);
typedef struct {
    uint8_t record[VCU_LOG_SIZE];
    uint32_t mask, started_ms, incomplete, invalid;
    uint8_t token;
    bool active;
} vcu_log_decoder_t;
bool vcu_log_decode(vcu_log_decoder_t *d, const uint8_t data[8], uint32_t rx_ms);
#endif
