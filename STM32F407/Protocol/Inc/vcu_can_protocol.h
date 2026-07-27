#ifndef VCU_CAN_PROTOCOL_H
#define VCU_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define VCU_CAN_PROTOCOL_VERSION 1U
#define VCU_CAN_MAX_DATA_LENGTH 8U

typedef enum {
    VCU_CAN_MESSAGE_HEARTBEAT = 0,
    VCU_CAN_MESSAGE_DRIVER_INPUT,
    VCU_CAN_MESSAGE_DRIVER_INTENT,
    VCU_CAN_MESSAGE_ACTUATOR_STATE,
    VCU_CAN_MESSAGE_MOTOR_STATUS,
    VCU_CAN_MESSAGE_DIAGNOSTIC,
    VCU_CAN_MESSAGE_CONFIG_REQUEST,
    VCU_CAN_MESSAGE_CONFIG_RESPONSE
} vcu_can_message_type_t;

typedef struct {
    uint32_t id;
    bool extended_id;
    bool remote_frame;
    uint8_t data_length;
    uint8_t data[VCU_CAN_MAX_DATA_LENGTH];
} vcu_can_frame_t;

bool vcu_can_protocol_validate_frame(const vcu_can_frame_t *frame);
bool vcu_can_protocol_version_supported(uint8_t version);

#endif
