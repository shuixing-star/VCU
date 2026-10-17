#ifndef VCU_MOTOR_CAN_H
#define VCU_MOTOR_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "mx_can_protocol.h"
#include "vcu_can_port.h"
#include "vcu_motor_config.h"

#define VCU_MOTOR_COUNT 2U
#define VCU_VEHICLE_RPM_DEADZONE 200
#define VCU_MOTOR_LEFT_COMMAND_ID VCU_MOTOR_LEFT_DRIVER_RECEIVE_ID
#define VCU_MOTOR_LEFT_ELECTRICAL_STATUS_ID VCU_MOTOR_LEFT_DRIVER_SEND_ID
#define VCU_MOTOR_LEFT_STATE_STATUS_ID \
        (VCU_MOTOR_LEFT_DRIVER_SEND_ID + \
         VCU_MOTOR_SECOND_STATUS_ID_OFFSET)

#define VCU_MOTOR_RIGHT_COMMAND_ID VCU_MOTOR_RIGHT_DRIVER_RECEIVE_ID
#define VCU_MOTOR_RIGHT_ELECTRICAL_STATUS_ID VCU_MOTOR_RIGHT_DRIVER_SEND_ID
#define VCU_MOTOR_RIGHT_STATE_STATUS_ID \
        (VCU_MOTOR_RIGHT_DRIVER_SEND_ID + \
         VCU_MOTOR_SECOND_STATUS_ID_OFFSET)


typedef enum {
    VCU_MOTOR_LEFT = 0,
    VCU_MOTOR_RIGHT = 1
} vcu_motor_index_t;

typedef struct {
    bool has_status;
    bool has_electrical_status;
    bool has_state_status;
    bool heartbeat_seen;
    bool heartbeat_advanced;
    uint32_t electrical_status_id;
    uint32_t state_status_id;
    uint32_t command_id;
    uint32_t status_frame_count;
    uint32_t electrical_status_frame_count;
    uint32_t state_status_frame_count;
    uint32_t last_status_ms;
    uint32_t last_electrical_status_ms;
    uint32_t last_state_status_ms;
    uint32_t last_heartbeat_change_ms;
    uint8_t last_heartbeat;
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
bool vcu_motor_can_controller_recent(
        const vcu_motor_controller_state_t *controller,
        uint32_t now_ms,
        uint32_t timeout_ms);
bool vcu_motor_can_send_targets(int16_t left_target,
                                int16_t right_target,
                                bool enable);
bool vcu_motor_can_send_brake(uint16_t left_current_a, uint16_t right_current_a);
bool vcu_motor_can_send_reset(void);

#endif
