#include "vcu_control_telemetry.h"

#include <limits.h>

#include "vcu_can_port.h"
#include "vcu_esp_link.h"
#include "vcu_gateway_protocol.h"

#define VCU_TELEMETRY_SLOT_PERIOD_MS 5U
#define VCU_TELEMETRY_SLOT_COUNT 6U

#define VCU_HEARTBEAT_FLAG_RUN_REQUESTED (1U << 0)
#define VCU_HEARTBEAT_FLAG_CRSF_LINK (1U << 1)
#define VCU_HEARTBEAT_FLAG_LEFT_ONLINE (1U << 2)
#define VCU_HEARTBEAT_FLAG_RIGHT_ONLINE (1U << 3)
#define VCU_HEARTBEAT_FLAG_OUTPUT_ENABLED (1U << 4)

#define VCU_RC_FLAG_LINK (1U << 0)
#define VCU_RC_FLAG_ARM (1U << 1)
#define VCU_RC_FLAG_RESET (1U << 2)

#define VCU_DIAG_FLAG_UART_ERROR (1U << 0)
#define VCU_DIAG_FLAG_UART_OVERFLOW (1U << 1)
#define VCU_DIAG_FLAG_CAN_RX_ERROR (1U << 2)
#define VCU_DIAG_FLAG_CAN_TX_ERROR (1U << 3)

static uint32_t s_last_slot_ms;
static uint8_t s_slot;
static uint8_t s_sequence;

static uint8_t saturating_age_10ms(bool valid,
                                   uint32_t now_ms,
                                   uint32_t timestamp_ms)
{
    uint32_t age;

    if (!valid) {
        return UINT8_MAX;
    }

    age = (uint32_t)(now_ms - timestamp_ms) / 10U;
    return age > UINT8_MAX ? UINT8_MAX : (uint8_t)age;
}

static bool controller_online(
        const vcu_motor_controller_state_t *controller,
        uint32_t now_ms)
{
    return controller->has_status &&
           (uint32_t)(now_ms - controller->last_status_ms) <= 500U;
}

static uint16_t build_fault_flags(
        const vcu_crsf_uart_state_t *crsf,
        const vcu_motor_can_state_t *motors,
        uint32_t now_ms)
{
    uint16_t flags = 0U;

    if (!crsf->link_ok) {
        flags |= (uint16_t)(1U << 0);
    }
    if (!controller_online(
                &motors->controller[VCU_MOTOR_LEFT], now_ms)) {
        flags |= (uint16_t)(1U << 1);
    }
    if (!controller_online(
                &motors->controller[VCU_MOTOR_RIGHT], now_ms)) {
        flags |= (uint16_t)(1U << 2);
    }
    if (motors->controller[VCU_MOTOR_LEFT].status.fault_code != 0U) {
        flags |= (uint16_t)(1U << 3);
    }
    if (motors->controller[VCU_MOTOR_RIGHT].status.fault_code != 0U) {
        flags |= (uint16_t)(1U << 4);
    }
    if (motors->tx_failure_count != 0U) {
        flags |= (uint16_t)(1U << 5);
    }
    return flags;
}

void vcu_control_telemetry_init(void)
{
    s_last_slot_ms = 0U;
    s_slot = 0U;
    s_sequence = 0U;
}

void vcu_control_telemetry_step(
        uint32_t now_ms,
        const vcu_crsf_uart_state_t *crsf,
        const vcu_drive_runtime_state_t *drive,
        const vcu_motor_can_state_t *motors)
{
    uint8_t data[8];
    uint32_t id;

    if (crsf == NULL || drive == NULL || motors == NULL ||
        (uint32_t)(now_ms - s_last_slot_ms) <
                VCU_TELEMETRY_SLOT_PERIOD_MS) {
        return;
    }

    s_last_slot_ms = now_ms;
    s_sequence++;

    switch (s_slot) {
    case 0U:
    {
        vcu_gateway_heartbeat_t message = {
            VCU_GATEWAY_PROTOCOL_VERSION,
            (uint8_t)drive->state,
            2U,
            0U,
            build_fault_flags(crsf, motors, now_ms),
            (uint8_t)drive->reason,
            s_sequence
        };

        if (drive->requested_run) {
            message.flags |= VCU_HEARTBEAT_FLAG_RUN_REQUESTED;
        }
        if (crsf->link_ok) {
            message.flags |= VCU_HEARTBEAT_FLAG_CRSF_LINK;
        }
        if (controller_online(
                    &motors->controller[VCU_MOTOR_LEFT], now_ms)) {
            message.flags |= VCU_HEARTBEAT_FLAG_LEFT_ONLINE;
        }
        if (controller_online(
                    &motors->controller[VCU_MOTOR_RIGHT], now_ms)) {
            message.flags |= VCU_HEARTBEAT_FLAG_RIGHT_ONLINE;
        }
        if (drive->state == VCU_DRIVE_STATE_ARMED &&
            drive->reason == VCU_DRIVE_REASON_RUNNING) {
            message.flags |= VCU_HEARTBEAT_FLAG_OUTPUT_ENABLED;
        }
        vcu_gateway_pack_heartbeat(&message, data);
        id = VCU_GATEWAY_ID_HEARTBEAT;
        break;
    }
    case 1U:
    {
        vcu_gateway_rc_input_t message = {
            drive->preview.throttle_raw,
            drive->preview.steering_raw,
            drive->preview.arm_raw,
            0U,
            s_sequence
        };

        if (drive->preview.link_ok) {
            message.flags |= VCU_RC_FLAG_LINK;
        }
        if (drive->preview.arm_requested) {
            message.flags |= VCU_RC_FLAG_ARM;
        }
        if (drive->preview.reset_requested) {
            message.flags |= VCU_RC_FLAG_RESET;
        }
        vcu_gateway_pack_rc_input(&message, data);
        id = VCU_GATEWAY_ID_RC_INPUT;
        break;
    }
    case 2U:
    {
        const vcu_gateway_targets_t message = {
            drive->latest_left_base_target,
            drive->latest_right_base_target,
            drive->latest_left_target,
            drive->latest_right_target
        };
        vcu_gateway_pack_targets(&message, data);
        id = VCU_GATEWAY_ID_TARGETS;
        break;
    }
    case 3U:
    case 4U:
    {
        const unsigned int index =
                s_slot == 3U ? VCU_MOTOR_LEFT : VCU_MOTOR_RIGHT;
        const vcu_motor_controller_state_t *controller =
                &motors->controller[index];
        const vcu_gateway_motor_status_t message = {
            controller->status.axis2_speed_rpm,
            controller->status.supply_voltage_decivolts,
            controller->status.controller_temperature_c,
            controller->status.fault_code,
            saturating_age_10ms(
                    controller->has_status,
                    now_ms, controller->last_status_ms),
            s_sequence
        };
        vcu_gateway_pack_motor_status(&message, data);
        id = index == VCU_MOTOR_LEFT ?
                VCU_GATEWAY_ID_MOTOR_LEFT :
                VCU_GATEWAY_ID_MOTOR_RIGHT;
        break;
    }
    default:
    {
        vcu_gateway_diagnostics_t message = {
            (uint16_t)crsf->valid_frame_count,
            (uint16_t)crsf->crc_error_count,
            (uint16_t)vcu_can_port_rx_overflow_count(),
            0U,
            s_sequence
        };

        if (crsf->uart_error_count != 0U) {
            message.flags |= VCU_DIAG_FLAG_UART_ERROR;
        }
        if (crsf->uart_overflow_count != 0U) {
            message.flags |= VCU_DIAG_FLAG_UART_OVERFLOW;
        }
        if (vcu_can_port_rx_error_count() != 0U) {
            message.flags |= VCU_DIAG_FLAG_CAN_RX_ERROR;
        }
        if (vcu_can_port_tx_error_count() != 0U) {
            message.flags |= VCU_DIAG_FLAG_CAN_TX_ERROR;
        }
        vcu_gateway_pack_diagnostics(&message, data);
        id = VCU_GATEWAY_ID_DIAGNOSTICS;
        break;
    }
    }

    (void)vcu_esp_link_send(id, data);
    s_slot = (uint8_t)((s_slot + 1U) % VCU_TELEMETRY_SLOT_COUNT);
}
