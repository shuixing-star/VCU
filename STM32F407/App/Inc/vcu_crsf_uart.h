#ifndef VCU_CRSF_UART_H
#define VCU_CRSF_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "vcu_crsf.h"

typedef struct {
    bool initialized;
    bool link_ok;
    uint32_t uart_error_count;
    uint32_t uart_overflow_count;
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t last_valid_frame_ms;
    uint16_t channels[VCU_CRSF_CHANNEL_COUNT];
} vcu_crsf_uart_state_t;

bool vcu_crsf_uart_init(void);
void vcu_crsf_uart_step(uint32_t now_ms);
bool vcu_crsf_uart_get_state(uint32_t now_ms,
                             vcu_crsf_uart_state_t *state);

#endif
