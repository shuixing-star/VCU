#ifndef VCU_CONTROL_TELEMETRY_H
#define VCU_CONTROL_TELEMETRY_H

#include <stdint.h>

#include "vcu_crsf_uart.h"
#include "vcu_drive_runtime.h"
#include "vcu_motor_can.h"

void vcu_control_telemetry_init(void);
void vcu_control_telemetry_step(
        uint32_t now_ms,
        const vcu_crsf_uart_state_t *crsf,
        const vcu_drive_runtime_state_t *drive,
        const vcu_motor_can_state_t *motors);

#endif
