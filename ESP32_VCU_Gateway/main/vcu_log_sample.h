#ifndef VCU_LOG_SAMPLE_H
#define VCU_LOG_SAMPLE_H

#include "vcu_stm32_gateway.h"

/* Ordering is the public vcu-log-values Lisp interface. Append new fields only. */
#define VCU_LOG_FIELD_COUNT 94U
#define VCU_LOG_CSV_FIELD_COUNT 74U
extern const unsigned int vcu_log_csv_fields[VCU_LOG_CSV_FIELD_COUNT];

typedef struct {
    const char *key;
    const char *name;
    const char *unit;
    unsigned int precision;
} vcu_log_field_t;

extern const vcu_log_field_t vcu_log_fields[VCU_LOG_FIELD_COUNT];

/* Cached values are retained; fresh_mask and ages describe their validity.
 * now_ms uses the same FreeRTOS-derived clock as gateway frame_last_rx_ms. */
void vcu_log_sample_from_state(const vcu_stm32_gateway_state_t *state,
                              uint32_t now_ms,
                              double values[VCU_LOG_FIELD_COUNT]);

#endif
