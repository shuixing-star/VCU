#ifndef VCU_CAN_GATEWAY_H
#define VCU_CAN_GATEWAY_H

#include <stdbool.h>
#include <stdint.h>

#include "vcu_can_protocol.h"
#include "vcu_types.h"

bool vcu_can_gateway_init(void);
void vcu_can_gateway_step(uint32_t now_ms);
bool vcu_can_gateway_process_rx(const vcu_can_frame_t *frame, uint32_t now_ms);
bool vcu_can_gateway_get_vehicle_feedback(vcu_vehicle_feedback_t *feedback);

#endif
