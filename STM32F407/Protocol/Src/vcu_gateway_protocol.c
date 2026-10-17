#include "vcu_gateway_protocol.h"

#include <stddef.h>

static void put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
}

static uint16_t get_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void put_i16(uint8_t *data, int16_t value)
{
    put_u16(data, (uint16_t)value);
}

static int16_t get_i16(const uint8_t *data)
{
    return (int16_t)get_u16(data);
}

bool vcu_gateway_pack_command(const vcu_gateway_command_t *command,
                              uint8_t data[8])
{
    if (command == NULL || data == NULL ||
        command->type <= VCU_GATEWAY_COMMAND_NONE ||
        command->type > VCU_GATEWAY_COMMAND_RESET) {
        return false;
    }

    data[0] = VCU_GATEWAY_PROTOCOL_VERSION;
    data[1] = (uint8_t)command->type;
    put_u16(&data[2], command->period_ms);
    put_u16(&data[4], command->acceleration_rpm_s);
    data[6] = command->sequence;
    data[7] = 0U;
    return true;
}

bool vcu_gateway_unpack_command(const uint8_t data[8],
                                vcu_gateway_command_t *command)
{
    if (data == NULL || command == NULL ||
        data[0] != VCU_GATEWAY_PROTOCOL_VERSION ||
        data[1] <= (uint8_t)VCU_GATEWAY_COMMAND_NONE ||
        data[1] > (uint8_t)VCU_GATEWAY_COMMAND_RESET) {
        return false;
    }

    command->type = (vcu_gateway_command_type_t)data[1];
    command->period_ms = get_u16(&data[2]);
    command->acceleration_rpm_s = get_u16(&data[4]);
    command->sequence = data[6];
    return true;
}

void vcu_gateway_pack_heartbeat(const vcu_gateway_heartbeat_t *message,
                                uint8_t data[8])
{
    data[0] = message->version;
    data[1] = message->state;
    data[2] = message->source;
    data[3] = message->flags;
    put_u16(&data[4], message->fault_flags);
    data[6] = message->reason;
    data[7] = message->sequence;
}

void vcu_gateway_unpack_heartbeat(const uint8_t data[8],
                                  vcu_gateway_heartbeat_t *message)
{
    message->version = data[0];
    message->state = data[1];
    message->source = data[2];
    message->flags = data[3];
    message->fault_flags = get_u16(&data[4]);
    message->reason = data[6];
    message->sequence = data[7];
}

void vcu_gateway_pack_rc_input(const vcu_gateway_rc_input_t *message,
                               uint8_t data[8])
{
    put_u16(&data[0], message->throttle_raw);
    put_u16(&data[2], message->steering_raw);
    put_u16(&data[4], message->arm_raw);
    data[6] = message->flags;
    data[7] = message->sequence;
}

void vcu_gateway_unpack_rc_input(const uint8_t data[8],
                                 vcu_gateway_rc_input_t *message)
{
    message->throttle_raw = get_u16(&data[0]);
    message->steering_raw = get_u16(&data[2]);
    message->arm_raw = get_u16(&data[4]);
    message->flags = data[6];
    message->sequence = data[7];
}

void vcu_gateway_pack_can_error(
        const vcu_gateway_can_error_t *message,
        uint8_t data[8])
{
    data[0] = message->sequence;
    data[1] = message->type;
    data[2] = message->flags;
    data[3] = message->hal_state;
    data[4] = message->tx_error_count;
    data[5] = message->rx_error_count;
    put_u16(&data[6], message->timestamp_10ms);
}

void vcu_gateway_unpack_can_error(
        const uint8_t data[8],
        vcu_gateway_can_error_t *message)
{
    message->sequence = data[0];
    message->type = data[1];
    message->flags = data[2];
    message->hal_state = data[3];
    message->tx_error_count = data[4];
    message->rx_error_count = data[5];
    message->timestamp_10ms = get_u16(&data[6]);
}

void vcu_gateway_pack_targets(const vcu_gateway_targets_t *message,
                              uint8_t data[8])
{
    put_i16(&data[0], message->left_base_rpm);
    put_i16(&data[2], message->right_base_rpm);
    put_i16(&data[4], message->left_target_rpm);
    put_i16(&data[6], message->right_target_rpm);
}

void vcu_gateway_unpack_targets(const uint8_t data[8],
                                vcu_gateway_targets_t *message)
{
    message->left_base_rpm = get_i16(&data[0]);
    message->right_base_rpm = get_i16(&data[2]);
    message->left_target_rpm = get_i16(&data[4]);
    message->right_target_rpm = get_i16(&data[6]);
}

void vcu_gateway_pack_motor_status(
        const vcu_gateway_motor_status_t *message,
        uint8_t data[8])
{
    put_i16(&data[0], message->speed_rpm);
    put_u16(&data[2], message->voltage_decivolts);
    data[4] = message->temperature_c;
    data[5] = message->fault_code;
    data[6] = message->age_10ms;
    data[7] = message->sequence;
}

void vcu_gateway_unpack_motor_status(
        const uint8_t data[8],
        vcu_gateway_motor_status_t *message)
{
    message->speed_rpm = get_i16(&data[0]);
    message->voltage_decivolts = get_u16(&data[2]);
    message->temperature_c = data[4];
    message->fault_code = data[5];
    message->age_10ms = data[6];
    message->sequence = data[7];
}

void vcu_gateway_pack_diagnostics(
        const vcu_gateway_diagnostics_t *message,
        uint8_t data[8])
{
    put_u16(&data[0], message->crsf_valid_frames);
    put_u16(&data[2], message->crsf_crc_errors);
    put_u16(&data[4], message->can_rx_overflows);
    data[6] = message->flags;
    data[7] = message->sequence;
}

void vcu_gateway_unpack_diagnostics(
        const uint8_t data[8],
        vcu_gateway_diagnostics_t *message)
{
    message->crsf_valid_frames = get_u16(&data[0]);
    message->crsf_crc_errors = get_u16(&data[2]);
    message->can_rx_overflows = get_u16(&data[4]);
    message->flags = data[6];
    message->sequence = data[7];
}

void vcu_gateway_pack_tx_probe(uint8_t sequence, uint16_t timestamp_10ms,
                               uint8_t part, uint32_t value, uint8_t data[8])
{
    unsigned int i;
    data[0] = sequence;
    data[1] = (uint8_t)timestamp_10ms;
    data[2] = (uint8_t)(timestamp_10ms >> 8U);
    data[3] = part;
    for (i = 0U; i < 4U; ++i) data[4U + i] = (uint8_t)(value >> (8U * i));
}

bool vcu_gateway_unpack_tx_probe(const uint8_t data[8], uint8_t *sequence,
                                 uint16_t *timestamp_10ms, uint8_t *part,
                                 uint32_t *value)
{
    unsigned int i;
    if (data[3] >= VCU_GATEWAY_TX_PROBE_PARTS) return false;
    *sequence = data[0];
    *timestamp_10ms = (uint16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8U));
    *part = data[3];
    *value = 0U;
    for (i = 0U; i < 4U; ++i) *value |= (uint32_t)data[4U + i] << (8U * i);
    return true;
}
