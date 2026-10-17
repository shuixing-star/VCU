#ifndef VCU_LOG_STORE_H
#define VCU_LOG_STORE_H
#include "vcu_log_wire.h"
#define VCU_LOG_SAMPLES 90U
#define VCU_LOG_EVENTS 32U
#define VCU_LOG_TX_ROWS 1792U
#define VCU_LOG_RAW_ROWS 128U
typedef struct { uint32_t rx_ms; uint8_t data[VCU_LOG_SIZE]; } vcu_log_sample_t;
typedef struct { uint32_t rx_ms; uint8_t data[8]; } vcu_log_tx_t;
typedef struct { uint32_t rx_ms, id; uint8_t data[8]; } vcu_log_raw_t;
typedef struct {
    vcu_log_sample_t samples[VCU_LOG_SAMPLES], events[VCU_LOG_EVENTS];
    vcu_log_tx_t tx[VCU_LOG_TX_ROWS];
    vcu_log_raw_t raw[VCU_LOG_RAW_ROWS];
    vcu_log_decoder_t decoder;
    uint32_t sample_total, event_total, tx_total, raw_total;
    uint32_t sample_overwrites, event_overwrites, tx_overwrites, raw_overwrites;
    uint32_t tx_resets_or_reordered;
    uint32_t gaps, tx_gaps, last_sequence, last_nonce, last_tx_sequence;
    uint32_t trigger_ms, last_rx_ms, frozen_discards, reboot_count;
    uint32_t esp_session;
    uint8_t last_fault[2][3];
    bool has_sequence, has_tx_sequence, triggered, frozen;
} vcu_log_store_t;
void vcu_log_store_init(vcu_log_store_t *s, uint32_t session);
void vcu_log_store_trigger(vcu_log_store_t *s, uint32_t now);
void vcu_log_store_tick(vcu_log_store_t *s, uint32_t now);
void vcu_log_store_frame(vcu_log_store_t *s,uint32_t id,const uint8_t data[8],uint8_t len,bool ext,bool rtr,uint32_t rx_ms);
bool vcu_log_in_window(const vcu_log_store_t *s,uint32_t rx_ms);
#endif
