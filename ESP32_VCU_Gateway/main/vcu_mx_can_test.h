#ifndef VCU_MX_CAN_TEST_H
#define VCU_MX_CAN_TEST_H

#include <stdbool.h>
#include <stdint.h>

#include "mx_can_protocol.h"

#define VCU_MX_CAN_CONTROLLER_COUNT 2U
#define VCU_VEHICLE_RPM_DEADZONE 200
#define VCU_VEHICLE_RPM_FILTER_ALPHA 0.25f

typedef enum {
    VCU_MX_CAN_LEFT = 0,
    VCU_MX_CAN_RIGHT = 1
} vcu_mx_can_controller_index_t;

typedef struct {
    bool configured;
    bool has_status;
    uint32_t status_id;
    uint32_t command_id;
    uint32_t status_frame_count;
    uint32_t last_status_ms;
    int8_t direction_sign;
    mx_can_status_t status;
} vcu_mx_can_controller_state_t;

typedef struct {
    bool initialized;
    uint32_t ignored_frame_count;
    vcu_mx_can_controller_state_t controller[VCU_MX_CAN_CONTROLLER_COUNT];
    bool last_tx_valid[VCU_MX_CAN_CONTROLLER_COUNT];
    uint32_t last_tx_count[VCU_MX_CAN_CONTROLLER_COUNT];
    uint32_t last_tx_id[VCU_MX_CAN_CONTROLLER_COUNT];
    uint8_t last_tx_data[VCU_MX_CAN_CONTROLLER_COUNT][MX_CAN_FRAME_SIZE];
    bool last_rx_valid[VCU_MX_CAN_CONTROLLER_COUNT];
    uint32_t last_rx_id[VCU_MX_CAN_CONTROLLER_COUNT];
    uint8_t last_rx_data[VCU_MX_CAN_CONTROLLER_COUNT][MX_CAN_FRAME_SIZE];
} vcu_mx_can_test_state_t;

void vcu_mx_can_test_init(void);
bool vcu_mx_can_test_process_can_frame(uint32_t id, const uint8_t *data, uint8_t len, bool is_ext);
bool vcu_mx_can_test_get_state(vcu_mx_can_test_state_t *state);
bool vcu_mx_can_test_commands_configured(void);
bool vcu_mx_can_test_send_axis2_targets(int16_t left_target, int16_t right_target, uint8_t ramp, bool enable);
bool vcu_mx_can_test_send_reset(void);

#endif