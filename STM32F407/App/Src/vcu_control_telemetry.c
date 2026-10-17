#include "vcu_control_telemetry.h"

#include <limits.h>

#include "vcu_can_port.h"
#include "vcu_esp_link.h"
#include "vcu_gateway_protocol.h"

#define VCU_TELEMETRY_SLOT_PERIOD_MS 5U
#define VCU_TELEMETRY_SLOT_COUNT 7U

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
#define VCU_DIAG_FLAG_COMMAND_DUPLICATE (1U << 4)
#define VCU_DIAG_FLAG_COMMAND_INVALID (1U << 5)
#define VCU_DIAG_FLAG_COMMAND_RECEIVED (1U << 6)
#define VCU_DIAG_FLAG_COMMAND_SUPPORTED (1U << 7)

static uint32_t s_last_slot_ms;
static uint8_t s_slot;
static uint8_t s_sequence;
static bool s_can_error_pending;
static uint8_t s_can_error_part;
static vcu_can_port_error_event_t s_can_error_event;

static void report_one_can_error(void)
{
    vcu_gateway_can_error_t message;
    uint8_t data[8];

    if (!s_can_error_pending) {
        s_can_error_pending = vcu_can_port_take_error_event(
                &s_can_error_event);
    }
    if (!s_can_error_pending) {
        return;
    }

    if (s_can_error_part != 0U) {
        const uint8_t part = (uint8_t)(s_can_error_part - 1U);
        vcu_gateway_pack_tx_probe(s_can_error_event.sequence,
                s_can_error_event.timestamp_10ms, part,
                s_can_error_event.tx_probe[part], data);
        if (vcu_esp_link_send_diagnostic(VCU_GATEWAY_ID_CAN_TX_PROBE, data)) {
            s_can_error_part++;
            if (s_can_error_part > VCU_GATEWAY_TX_PROBE_PARTS) {
                s_can_error_pending = false;
                s_can_error_part = 0U;
            }
        }
        return;
    }

    message.sequence = s_can_error_event.sequence;
    message.type = s_can_error_event.type;
    message.flags = s_can_error_event.flags;
    message.hal_state = s_can_error_event.hal_state;
    message.tx_error_count = s_can_error_event.tx_error_count;
    message.rx_error_count = s_can_error_event.rx_error_count;
    message.timestamp_10ms = s_can_error_event.timestamp_10ms;
    vcu_gateway_pack_can_error(&message, data);
    if (vcu_esp_link_send_diagnostic(
                VCU_GATEWAY_ID_CAN_ERROR, data)) {
        if (s_can_error_event.has_tx_probe) {
            s_can_error_part = 1U;
        } else {
            s_can_error_pending = false;
        }
    }
}

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
    return vcu_motor_can_controller_recent(controller, now_ms, 500U);
}

static uint8_t encode_temperature(int16_t temperature_c)
{
    if (temperature_c < 0) {
        return (uint8_t)(int8_t)temperature_c;
    }
    return temperature_c > UINT8_MAX ? UINT8_MAX : (uint8_t)temperature_c;
}

static uint16_t build_fault_flags(
        const vcu_drive_runtime_state_t *drive,
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
    if (drive->state == VCU_DRIVE_STATE_FAULT &&
        drive->reason == VCU_DRIVE_REASON_CAN_TX_FAILURE) {
        flags |= (uint16_t)(1U << 5);
    }
    return flags;
}

void vcu_control_telemetry_init(void)
{
    s_last_slot_ms = 0U;
    s_slot = 0U;
    s_sequence = 0U;
    s_can_error_pending = false;
    s_can_error_part = 0U;
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

    /* Motor control may already occupy two of the three TX mailboxes.
     * Send at most one telemetry/error frame per slot; errors have their
     * own slot so ordinary telemetry cannot starve them. */
    if (s_slot == VCU_TELEMETRY_SLOT_COUNT - 1U) {
        report_one_can_error();
        s_slot = 0U;
        return;
    }

    switch (s_slot) {
    case 0U:
    {
        vcu_gateway_heartbeat_t message = {
            VCU_GATEWAY_PROTOCOL_VERSION,
            (uint8_t)drive->state,
            2U,
            0U,
            build_fault_flags(drive, crsf, motors, now_ms),
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
        if ((drive->state == VCU_DRIVE_STATE_ARMED &&
             drive->reason == VCU_DRIVE_REASON_RUNNING) ||
            drive->state == VCU_DRIVE_STATE_BRAKING) {
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
            controller->status.speed_rpm,
            controller->status.supply_voltage_decivolts,
            encode_temperature(controller->status.controller_temperature_c),
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
            VCU_DIAG_FLAG_COMMAND_SUPPORTED,
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
        if (vcu_esp_link_duplicate_command_seen()) {
            message.flags |= VCU_DIAG_FLAG_COMMAND_DUPLICATE;
        }
        if (vcu_esp_link_invalid_command_seen()) {
            message.flags |= VCU_DIAG_FLAG_COMMAND_INVALID;
        }
        if (vcu_esp_link_command_seen()) {
            message.flags |= VCU_DIAG_FLAG_COMMAND_RECEIVED;
        }
        vcu_gateway_pack_diagnostics(&message, data);
        id = VCU_GATEWAY_ID_DIAGNOSTICS;
        break;
    }
    }

    (void)vcu_esp_link_send(id, data);
    s_slot = (uint8_t)((s_slot + 1U) % VCU_TELEMETRY_SLOT_COUNT);
}
