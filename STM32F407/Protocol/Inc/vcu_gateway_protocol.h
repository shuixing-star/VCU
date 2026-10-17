#ifndef VCU_GATEWAY_PROTOCOL_H
#define VCU_GATEWAY_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define VCU_GATEWAY_REBOOT_MAGIC 0xA5U
#define VCU_GATEWAY_PROTOCOL_VERSION 1U

#define VCU_GATEWAY_ID_HEARTBEAT 0x680U
#define VCU_GATEWAY_ID_RC_INPUT 0x681U
#define VCU_GATEWAY_ID_TARGETS 0x682U
#define VCU_GATEWAY_ID_MOTOR_LEFT 0x683U
#define VCU_GATEWAY_ID_MOTOR_RIGHT 0x684U
#define VCU_GATEWAY_ID_DIAGNOSTICS 0x685U
#define VCU_GATEWAY_ID_COMMAND 0x686U
#define VCU_GATEWAY_ID_CAN_ERROR 0x687U
/* Additive diagnostic extension: seq, timestamp LE16, part, value LE32.
 * Parts: TSR, ESR, first ID, second ID, elapsed us, max check gap us,
 * metadata (source, first mailbox, second mailbox, trigger), low byte first.
 * source byte: bits 0..1 source (1=pair, 2=background), bits 2..7 task
 * switch hook count (63=saturated). trigger: 1=deadline, 2=no TXOK,
 * 3=mailbox deadline, 4=enqueue failure. DWT wall time includes interrupts.
 * A long check gap alone does not establish RTOS preemption. */
#define VCU_GATEWAY_ID_CAN_TX_PROBE 0x688U
#define VCU_GATEWAY_TX_PROBE_PARTS 7U
#define VCU_GATEWAY_TX_PROBE_COMPLETE 0x7FU

#define VCU_GATEWAY_CAN_ERROR_LEC_MASK 0x07U
#define VCU_GATEWAY_CAN_ERROR_WARNING (1U << 3)
#define VCU_GATEWAY_CAN_ERROR_PASSIVE (1U << 4)
#define VCU_GATEWAY_CAN_ERROR_BUS_OFF (1U << 5)
#define VCU_GATEWAY_CAN_ERROR_TX (1U << 6)
#define VCU_GATEWAY_CAN_ERROR_RX_OVERFLOW (1U << 7)

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

typedef enum {
    VCU_GATEWAY_CAN_ERROR_HAL_IRQ = 1,
    VCU_GATEWAY_CAN_ERROR_MAILBOX_TIMEOUT,
    VCU_GATEWAY_CAN_ERROR_TX_TIMEOUT,
    VCU_GATEWAY_CAN_ERROR_TX_ADD_FAILED,
    VCU_GATEWAY_CAN_ERROR_SOFT_RECOVERY_FAILED,
    VCU_GATEWAY_CAN_ERROR_HARD_RECOVERY_FAILED
} vcu_gateway_can_error_type_t;

typedef struct {
    uint8_t sequence;
    uint8_t type;
    uint8_t flags;
    uint8_t hal_state;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint16_t timestamp_10ms;
} vcu_gateway_can_error_t;

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
void vcu_gateway_pack_can_error(
        const vcu_gateway_can_error_t *message,
        uint8_t data[8]);
void vcu_gateway_unpack_can_error(
        const uint8_t data[8],
        vcu_gateway_can_error_t *message);

void vcu_gateway_pack_tx_probe(uint8_t sequence, uint16_t timestamp_10ms,
                               uint8_t part, uint32_t value, uint8_t data[8]);
bool vcu_gateway_unpack_tx_probe(const uint8_t data[8], uint8_t *sequence,
                                 uint16_t *timestamp_10ms, uint8_t *part,
                                 uint32_t *value);

#endif
