#ifndef VCU_STM32_GATEWAY_H
#define VCU_STM32_GATEWAY_H

#include <stdbool.h>
#include <stdint.h>

#include "vcu_gateway_protocol.h"
#include "vcu_k2_diag.h"
#include "vcu_brake_diag.h"

#define VCU_STM32_GATEWAY_PROTOCOL_VERSION VCU_GATEWAY_PROTOCOL_VERSION

#define VCU_STM32_GATEWAY_ID_HEARTBEAT VCU_GATEWAY_ID_HEARTBEAT
#define VCU_STM32_GATEWAY_ID_RC_INPUT VCU_GATEWAY_ID_RC_INPUT
#define VCU_STM32_GATEWAY_ID_TARGETS VCU_GATEWAY_ID_TARGETS
#define VCU_STM32_GATEWAY_ID_MOTOR_LEFT VCU_GATEWAY_ID_MOTOR_LEFT
#define VCU_STM32_GATEWAY_ID_MOTOR_RIGHT VCU_GATEWAY_ID_MOTOR_RIGHT
#define VCU_STM32_GATEWAY_ID_DIAGNOSTICS VCU_GATEWAY_ID_DIAGNOSTICS
#define VCU_STM32_GATEWAY_ID_CAN_ERROR VCU_GATEWAY_ID_CAN_ERROR
#define VCU_STM32_GATEWAY_ONLINE_TIMEOUT_MS 500U
#define VCU_STM32_GATEWAY_TELEMETRY_COUNT 6U

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
    uint32_t can_error_event_count;
    uint32_t can_error_sequence_gap_count;
    uint8_t can_error_stored_count;
    uint32_t frame_rx_count[VCU_STM32_GATEWAY_TELEMETRY_COUNT];
    uint32_t frame_last_rx_ms[VCU_STM32_GATEWAY_TELEMETRY_COUNT];

    vcu_k2_diag_t k2_diag[2];

    bool has_brake_diag;
    uint32_t brake_diag_rx_ms;
    vcu_brake_diag_t brake_diag;
    bool has_heartbeat;
    bool has_rc_input;
    bool has_targets;
    bool has_motor_left;
    bool has_motor_right;
    bool has_diagnostics;
    bool has_can_error;
    vcu_stm32_gateway_heartbeat_t heartbeat;
    vcu_stm32_gateway_rc_input_t rc_input;
    vcu_stm32_gateway_targets_t targets;
    vcu_stm32_gateway_motor_status_t motor_left;
    vcu_stm32_gateway_motor_status_t motor_right;
    vcu_stm32_gateway_diagnostics_t diagnostics;
    vcu_gateway_can_error_t last_can_error;
    uint32_t last_can_error_rx_ms;
} vcu_stm32_gateway_state_t;

void vcu_stm32_gateway_init(void);

/*
 * Consume one CAN frame.
 *
 * Returns true only for a valid 8-byte standard telemetry frame in the
 * inclusive ID range 0x680..0x685, or a CAN error event on 0x687.
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

bool vcu_stm32_gateway_request_stop(void);
bool vcu_stm32_gateway_request_reset(void);
bool vcu_stm32_gateway_request_run(uint16_t period_ms,
                                    uint16_t acceleration_rpm_s);

#endif
