#ifndef VCU_STM32_GATEWAY_H
#define VCU_STM32_GATEWAY_H

#include <stdbool.h>
#include <stdint.h>

#include "vcu_gateway_protocol.h"

#define VCU_STM32_GATEWAY_PROTOCOL_VERSION VCU_GATEWAY_PROTOCOL_VERSION

#define VCU_STM32_GATEWAY_ID_HEARTBEAT VCU_GATEWAY_ID_HEARTBEAT
#define VCU_STM32_GATEWAY_ID_RC_INPUT VCU_GATEWAY_ID_RC_INPUT
#define VCU_STM32_GATEWAY_ID_TARGETS VCU_GATEWAY_ID_TARGETS
#define VCU_STM32_GATEWAY_ID_MOTOR_LEFT VCU_GATEWAY_ID_MOTOR_LEFT
#define VCU_STM32_GATEWAY_ID_MOTOR_RIGHT VCU_GATEWAY_ID_MOTOR_RIGHT
#define VCU_STM32_GATEWAY_ID_DIAGNOSTICS VCU_GATEWAY_ID_DIAGNOSTICS
#define VCU_STM32_GATEWAY_ID_COMMAND VCU_GATEWAY_ID_COMMAND

#define VCU_STM32_GATEWAY_ONLINE_TIMEOUT_MS 500U
#define VCU_STM32_GATEWAY_TELEMETRY_COUNT 6U

#define VCU_STM32_GATEWAY_COMMAND_NONE VCU_GATEWAY_COMMAND_NONE
#define VCU_STM32_GATEWAY_COMMAND_RUN VCU_GATEWAY_COMMAND_RUN
#define VCU_STM32_GATEWAY_COMMAND_STOP VCU_GATEWAY_COMMAND_STOP
#define VCU_STM32_GATEWAY_COMMAND_RESET VCU_GATEWAY_COMMAND_RESET

typedef vcu_gateway_command_type_t vcu_stm32_gateway_command_type_t;

typedef enum {
    VCU_STM32_GATEWAY_TELEMETRY_HEARTBEAT = 0,
    VCU_STM32_GATEWAY_TELEMETRY_RC_INPUT,
    VCU_STM32_GATEWAY_TELEMETRY_TARGETS,
    VCU_STM32_GATEWAY_TELEMETRY_MOTOR_LEFT,
    VCU_STM32_GATEWAY_TELEMETRY_MOTOR_RIGHT,
    VCU_STM32_GATEWAY_TELEMETRY_DIAGNOSTICS
} vcu_stm32_gateway_telemetry_index_t;

typedef vcu_gateway_heartbeat_t vcu_stm32_gateway_heartbeat_t;
typedef vcu_gateway_rc_input_t vcu_stm32_gateway_rc_input_t;
typedef vcu_gateway_targets_t vcu_stm32_gateway_targets_t;
typedef vcu_gateway_motor_status_t vcu_stm32_gateway_motor_status_t;
typedef vcu_gateway_diagnostics_t vcu_stm32_gateway_diagnostics_t;

typedef struct {
    bool initialized;
    bool online;
    bool protocol_compatible;
    uint32_t heartbeat_age_ms;
    uint32_t last_any_rx_ms;
    uint32_t invalid_frame_count;
    uint32_t protocol_mismatch_count;
    uint32_t frame_rx_count[VCU_STM32_GATEWAY_TELEMETRY_COUNT];
    uint32_t frame_last_rx_ms[VCU_STM32_GATEWAY_TELEMETRY_COUNT];

    bool has_heartbeat;
    bool has_rc_input;
    bool has_targets;
    bool has_motor_left;
    bool has_motor_right;
    bool has_diagnostics;
    vcu_stm32_gateway_heartbeat_t heartbeat;
    vcu_stm32_gateway_rc_input_t rc_input;
    vcu_stm32_gateway_targets_t targets;
    vcu_stm32_gateway_motor_status_t motor_left;
    vcu_stm32_gateway_motor_status_t motor_right;
    vcu_stm32_gateway_diagnostics_t diagnostics;

    uint32_t command_tx_attempt_count;
    vcu_stm32_gateway_command_type_t last_command_type;
    uint16_t last_command_period_ms;
    uint16_t last_command_acceleration_rpm_s;
    uint8_t last_command_sequence;
} vcu_stm32_gateway_state_t;

void vcu_stm32_gateway_init(void);

/*
 * Consume one CAN frame.
 *
 * Returns true only for a valid 8-byte standard telemetry frame in the
 * inclusive ID range 0x680..0x685.
 */
bool vcu_stm32_gateway_process_can_frame(uint32_t id,
                                         const uint8_t *data,
                                         uint8_t len,
                                         bool is_ext);

/*
 * Return an atomic snapshot. The online and heartbeat_age_ms fields are
 * evaluated at the time of this call.
 */
bool vcu_stm32_gateway_get_state(vcu_stm32_gateway_state_t *state);

#endif
