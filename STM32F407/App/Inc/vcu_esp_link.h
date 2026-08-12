#ifndef VCU_ESP_LINK_H
#define VCU_ESP_LINK_H

#include <stdbool.h>

#include "vcu_can_port.h"
#include "vcu_gateway_protocol.h"

void vcu_esp_link_init(void);
bool vcu_esp_link_process_frame(const vcu_can_port_frame_t *frame);
bool vcu_esp_link_take_command(vcu_gateway_command_t *command);
bool vcu_esp_link_send(uint32_t id, const uint8_t data[8]);
bool vcu_esp_link_command_seen(void);
bool vcu_esp_link_invalid_command_seen(void);
bool vcu_esp_link_duplicate_command_seen(void);

#endif
