#ifndef VCU_MOTOR_CAN_H
#define VCU_MOTOR_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "mx_can_protocol.h"
#include "vcu_can_port.h"

#define VCU_MOTOR_COUNT 2U
#define VCU_VEHICLE_RPM_DEADZONE 200

typedef enum {
    VCU_MOTOR_LEFT = 0,
    VCU_MOTOR_RIGHT = 1
} vcu_motor_index_t;

typedef struct {
    bool has_status;
    uint32_t status_id;
    uint32_t command_id;
    uint32_t status_frame_count;
    uint32_t last_status_ms;
    int8_t direction_sign;
    mx_can_status_t status;
} vcu_motor_controller_state_t;

typedef struct {
    bool initialized;
    uint32_t ignored_frame_count;
    uint32_t tx_failure_count;
    vcu_motor_controller_state_t controller[VCU_MOTOR_COUNT];
    uint32_t last_tx_count[VCU_MOTOR_COUNT];
    uint8_t last_tx_data[VCU_MOTOR_COUNT][MX_CAN_FRAME_SIZE];
} vcu_motor_can_state_t;

bool vcu_motor_can_init(void);
bool vcu_motor_can_process_frame(const vcu_can_port_frame_t *frame,
                                 uint32_t now_ms);
bool vcu_motor_can_get_state(vcu_motor_can_state_t *state);
bool vcu_motor_can_send_targets(int16_t left_target,
                                int16_t right_target,
                                uint8_t ramp,
                                bool enable);
bool vcu_motor_can_send_reset(void);
bool vcu_motor_can_send_reboot(vcu_motor_index_t controller);

#endif
