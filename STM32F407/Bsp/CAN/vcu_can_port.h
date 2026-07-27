#ifndef VCU_CAN_PORT_H
#define VCU_CAN_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;
    uint32_t received_ms;
    bool extended_id;
    bool remote_frame;
    uint8_t data_length;
    uint8_t data[8];
} vcu_can_port_frame_t;

bool vcu_can_port_init(void);
bool vcu_can_port_read(vcu_can_port_frame_t *frame);
bool vcu_can_port_send(const vcu_can_port_frame_t *frame);
bool vcu_can_port_send_pair(const vcu_can_port_frame_t *first,
                            const vcu_can_port_frame_t *second);
uint32_t vcu_can_port_rx_overflow_count(void);
uint32_t vcu_can_port_rx_error_count(void);
uint32_t vcu_can_port_tx_error_count(void);

#endif
