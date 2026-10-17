#ifndef VCU_FLIGHT_LOG_H
#define VCU_FLIGHT_LOG_H
#include "vcu_drive_runtime.h"
#include "vcu_can_port.h"
void vcu_flight_log_init(uint32_t boot_nonce);
void vcu_flight_log_rx(const vcu_can_port_frame_t *frame);
void vcu_flight_log_command(uint8_t command);
void vcu_flight_log_step(uint32_t now, const vcu_crsf_uart_state_t *rc,
                        const vcu_drive_runtime_state_t *drive,
                        const vcu_motor_can_state_t *motors);
uint32_t vcu_can_port_boot_nonce(void);
#endif
