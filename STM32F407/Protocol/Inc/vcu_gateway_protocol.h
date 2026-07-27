#ifndef VCU_GATEWAY_PROTOCOL_H
#define VCU_GATEWAY_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define VCU_GATEWAY_PROTOCOL_VERSION 1U

#define VCU_GATEWAY_ID_HEARTBEAT 0x680U
#define VCU_GATEWAY_ID_RC_INPUT 0x681U
#define VCU_GATEWAY_ID_TARGETS 0x682U
#define VCU_GATEWAY_ID_MOTOR_LEFT 0x683U
#define VCU_GATEWAY_ID_MOTOR_RIGHT 0x684U
#define VCU_GATEWAY_ID_DIAGNOSTICS 0x685U
#define VCU_GATEWAY_ID_COMMAND 0x686U

typedef enum {
    VCU_GATEWAY_COMMAND_NONE = 0,
    VCU_GATEWAY_COMMAND_RUN = 1,
    VCU_GATEWAY_COMMAND_STOP = 2,
    VCU_GATEWAY_COMMAND_RESET = 3
} vcu_gateway_command_type_t;

typedef struct {
    vcu_gateway_command_type_t type;
    uint16_t period_ms;
    uint16_t acceleration_rpm_s;
    uint8_t sequence;
} vcu_gateway_command_t;

typedef struct {
    uint8_t version;
    uint8_t state;
    uint8_t source;
    uint8_t flags;
    uint16_t fault_flags;
    uint8_t reason;
    uint8_t sequence;
} vcu_gateway_heartbeat_t;

typedef struct {
    uint16_t throttle_raw;
    uint16_t steering_raw;
    uint16_t arm_raw;
    uint8_t flags;
    uint8_t sequence;
} vcu_gateway_rc_input_t;

typedef struct {
    int16_t left_base_rpm;
    int16_t right_base_rpm;
    int16_t left_target_rpm;
    int16_t right_target_rpm;
} vcu_gateway_targets_t;

typedef struct {
    int16_t speed_rpm;
    uint16_t voltage_decivolts;
    uint8_t temperature_c;
    uint8_t fault_code;
    uint8_t age_10ms;
    uint8_t sequence;
} vcu_gateway_motor_status_t;

typedef struct {
    uint16_t crsf_valid_frames;
    uint16_t crsf_crc_errors;
    uint16_t can_rx_overflows;
    uint8_t flags;
    uint8_t sequence;
} vcu_gateway_diagnostics_t;

bool vcu_gateway_pack_command(const vcu_gateway_command_t *command,
                              uint8_t data[8]);
bool vcu_gateway_unpack_command(const uint8_t data[8],
                                vcu_gateway_command_t *command);
void vcu_gateway_pack_heartbeat(const vcu_gateway_heartbeat_t *message,
                                uint8_t data[8]);
void vcu_gateway_unpack_heartbeat(const uint8_t data[8],
                                  vcu_gateway_heartbeat_t *message);
void vcu_gateway_pack_rc_input(const vcu_gateway_rc_input_t *message,
                               uint8_t data[8]);
void vcu_gateway_unpack_rc_input(const uint8_t data[8],
                                 vcu_gateway_rc_input_t *message);
void vcu_gateway_pack_targets(const vcu_gateway_targets_t *message,
                              uint8_t data[8]);
void vcu_gateway_unpack_targets(const uint8_t data[8],
                                vcu_gateway_targets_t *message);
void vcu_gateway_pack_motor_status(
        const vcu_gateway_motor_status_t *message,
        uint8_t data[8]);
void vcu_gateway_unpack_motor_status(
        const uint8_t data[8],
        vcu_gateway_motor_status_t *message);
void vcu_gateway_pack_diagnostics(
        const vcu_gateway_diagnostics_t *message,
        uint8_t data[8]);
void vcu_gateway_unpack_diagnostics(
        const uint8_t data[8],
        vcu_gateway_diagnostics_t *message);

#endif
