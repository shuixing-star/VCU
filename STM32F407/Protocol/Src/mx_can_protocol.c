#include "mx_can_protocol.h"

#include <stddef.h>

#define MX_CAN_ELECTRICAL_MAX_VOLTAGE_DECIVOLTS 3000U
#define MX_CAN_ELECTRICAL_MAX_CURRENT_DECIAMPS 32000
#define MX_CAN_ELECTRICAL_ZERO_OFFSET 32000L
#define MX_CAN_TEMPERATURE_OFFSET_C 40
#define MX_CAN_TEMPERATURE_MAX_RAW 250U
#define MX_CAN_STATE_RESERVED_MASK 0xF8U
#define MX_CAN_FAULT_GROUP_2_MASK 0x07U
#define MX_CAN_FAULT_GROUP_3_MASK 0x3FU

static bool signed_target_valid(int16_t value)
{
    return value >= -MX_CAN_MAX_TARGET_RAW &&
           value <= MX_CAN_MAX_TARGET_RAW;
}

static void put_biased_i16_le(uint8_t *data, int16_t value)
{
    const uint16_t raw = (uint16_t)((int32_t)value +
                                    MX_CAN_ELECTRICAL_ZERO_OFFSET);

    data[0] = (uint8_t)raw;
    data[1] = (uint8_t)(raw >> 8U);
}

static void put_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t get_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] |
           ((uint16_t)data[1] << 8U);
}

static bool decode_temperature(uint8_t raw, int16_t *temperature_c)
{
    if (temperature_c == NULL || raw > MX_CAN_TEMPERATURE_MAX_RAW) {
        return false;
    }

    *temperature_c = (int16_t)raw - MX_CAN_TEMPERATURE_OFFSET_C;
    return true;
}

bool mx_can_pack_command(const mx_can_command_t *command,
                         uint8_t data[MX_CAN_FRAME_SIZE])
{
    uint8_t control = 0U;

    if (command == NULL || data == NULL ||
        !signed_target_valid(command->torque_deciamps) ||
        !signed_target_valid(command->speed_rpm) ||
        command->mode > MX_CAN_CONTROL_SPEED ||
        command->brake_current_a >
                MX_CAN_MAX_BRAKE_CURRENT_A) {
        return false;
    }
    if (command->run) {
        control |= MX_CAN_COMMAND_RUN_MASK;
    }
    if (command->mode == MX_CAN_CONTROL_SPEED) {
        control |= MX_CAN_COMMAND_SPEED_MODE_MASK;
    }
    if (command->reset_enable) {
        control |= MX_CAN_COMMAND_RESET_ENABLE_MASK;
    }

    put_biased_i16_le(&data[0], command->torque_deciamps);
    put_biased_i16_le(&data[2], command->speed_rpm);
    data[4] = control;
    put_u16_le(&data[5], command->brake_current_a);
    data[7] = command->heartbeat;
    return true;
}

bool mx_can_pack_speed_command(int16_t speed_rpm,
                               bool run,
                               uint8_t heartbeat,
                               uint8_t data[MX_CAN_FRAME_SIZE])
{
    const mx_can_command_t command = {
        .torque_deciamps = 0,
        .speed_rpm = speed_rpm,
        .mode = MX_CAN_CONTROL_SPEED,
        .run = run,
        .reset_enable = false,
        .brake_current_a = 0U,
        .heartbeat = heartbeat,
    };

    return mx_can_pack_command(&command, data);
}

bool mx_can_unpack_electrical_status(
        const uint8_t data[MX_CAN_FRAME_SIZE],
        mx_can_status_t *status)
{
    uint16_t voltage;
    int32_t bus_current;
    int32_t phase_current;
    int32_t speed;

    if (data == NULL || status == NULL) {
        return false;
    }

    voltage = get_u16_le(&data[0]);
    bus_current = (int32_t)get_u16_le(&data[2]) -
                  MX_CAN_ELECTRICAL_ZERO_OFFSET;
    phase_current = (int32_t)get_u16_le(&data[4]) -
                    MX_CAN_ELECTRICAL_ZERO_OFFSET;
    speed = (int32_t)get_u16_le(&data[6]) -
            MX_CAN_ELECTRICAL_ZERO_OFFSET;
    if (voltage > MX_CAN_ELECTRICAL_MAX_VOLTAGE_DECIVOLTS ||
        bus_current < -MX_CAN_ELECTRICAL_MAX_CURRENT_DECIAMPS ||
        bus_current > MX_CAN_ELECTRICAL_MAX_CURRENT_DECIAMPS ||
        phase_current < -MX_CAN_ELECTRICAL_MAX_CURRENT_DECIAMPS ||
        phase_current > MX_CAN_ELECTRICAL_MAX_CURRENT_DECIAMPS ||
        speed < -MX_CAN_MAX_TARGET_RAW ||
        speed > MX_CAN_MAX_TARGET_RAW) {
        return false;
    }

    status->supply_voltage_decivolts = voltage;
    status->bus_current_deciamps = (int16_t)bus_current;
    status->phase_current_deciamps = (int16_t)phase_current;
    status->speed_rpm = (int16_t)speed;
    return true;
}

bool mx_can_unpack_state_status(
        const uint8_t data[MX_CAN_FRAME_SIZE],
        mx_can_status_t *status)
{
    int16_t controller_temperature;
    int16_t motor_temperature;

    if (data == NULL || status == NULL ||
        (data[2] & MX_CAN_STATE_RESERVED_MASK) != 0U ||
        !decode_temperature(data[0], &controller_temperature) ||
        !decode_temperature(data[1], &motor_temperature)) {
        return false;
    }

    status->controller_temperature_c = controller_temperature;
    status->motor_temperature_c = motor_temperature;
    status->state_flags = data[2];
    status->fault_flags[0] = data[3];
    status->fault_flags[1] = data[4];
    status->fault_flags[2] = data[5];
    status->heartbeat = data[7];
    status->fault_code = mx_can_status_primary_fault_code(status);
    return true;
}

bool mx_can_status_has_fault(const mx_can_status_t *status)
{
    return status != NULL &&
           (status->fault_flags[0] != 0U ||
            (status->fault_flags[1] & MX_CAN_FAULT_GROUP_2_MASK) != 0U ||
            (status->fault_flags[2] & MX_CAN_FAULT_GROUP_3_MASK) != 0U);
}

uint8_t mx_can_status_primary_fault_code(const mx_can_status_t *status)
{
    static const uint8_t group_1_code[8] = {
        MX_CAN_FAULT_OVERCURRENT,
        MX_CAN_FAULT_MOTOR_1_OVERLOAD,
        MX_CAN_FAULT_OVERVOLTAGE,
        MX_CAN_FAULT_UNDERVOLTAGE,
        MX_CAN_FAULT_CONTROLLER_OVERTEMP,
        MX_CAN_FAULT_MOTOR_1_OVERTEMP,
        MX_CAN_FAULT_MOTOR_1_STALL,
        MX_CAN_FAULT_MOTOR_1_PHASE_LOSS
    };
    static const uint8_t group_2_code[3] = {
        MX_CAN_FAULT_MOTOR_1_HALL,
        MX_CAN_FAULT_MOTOR_2_HALL,
        MX_CAN_FAULT_MOTOR_1_ENCODER
    };
    static const uint8_t group_3_code[6] = {
        MX_CAN_FAULT_DRIVER_1,
        MX_CAN_FAULT_DRIVER_2,
        MX_CAN_FAULT_DRIVER_3,
        MX_CAN_FAULT_RS485,
        MX_CAN_FAULT_CAN,
        MX_CAN_FAULT_DRIVER_4
    };
    unsigned int bit;

    if (status == NULL) {
        return MX_CAN_FAULT_NONE;
    }
    for (bit = 0U; bit < 8U; bit++) {
        if ((status->fault_flags[0] & (1U << bit)) != 0U) {
            return group_1_code[bit];
        }
    }
    for (bit = 0U; bit < 3U; bit++) {
        if ((status->fault_flags[1] & (1U << bit)) != 0U) {
            return group_2_code[bit];
        }
    }
    for (bit = 0U; bit < 6U; bit++) {
        if ((status->fault_flags[2] & (1U << bit)) != 0U) {
            return group_3_code[bit];
        }
    }
    return MX_CAN_FAULT_NONE;
}

const char *mx_can_fault_name(uint8_t fault_code) {
    switch (fault_code) {
    case 0: return "Normal";
    case 1: case 2: case 3: case 4: return "Driver fault";
    case 5: return "Overcurrent";
    case 6: return "Overvoltage";
    case 7: return "Undervoltage";
    case 8: return "Controller overtemperature";
    case 9: case 10: return "Reserved/unknown";
    case 11: return "Motor 2 overspeed";
    case 12: return "Motor 1 overspeed";
    case 13: return "Motor 2 overload";
    case 14: return "Motor 1 overload";
    case 15: return "Motor 2 phase loss";
    case 16: return "Motor 1 phase loss";
    case 17: return "Motor 2 holding-brake fault";
    case 18: return "Motor 1 holding-brake fault";
    case 19: return "Motor 2 encoder fault";
    case 20: return "Motor 1 encoder fault";
    case 21: return "Motor 2 overtemperature";
    case 22: return "Motor 1 overtemperature";
    case 23: return "Motor 2 Hall-sensor fault";
    case 24: return "Motor 1 Hall-sensor fault";
    case 25: return "Motor 2 stall";
    case 26: return "Motor 1 stall";
    case 27: return "UART communication fault";
    case 28: return "RS485 communication fault";
    case 29: return "CAN communication fault";
    case 30: return "Joystick fault";
    case 31: return "Throttle fault";
    default: return "Unknown fault code";
    }
}

mx_can_fault_action_t mx_can_fault_action(uint8_t fault_code) {
    switch (fault_code) {
    case MX_CAN_FAULT_NONE:
        return MX_CAN_FAULT_ACTION_NONE;
    case MX_CAN_FAULT_OVERVOLTAGE:
    case MX_CAN_FAULT_UNDERVOLTAGE:
    case MX_CAN_FAULT_CONTROLLER_OVERTEMP:
    case MX_CAN_FAULT_MOTOR_2_OVERTEMP:
    case MX_CAN_FAULT_MOTOR_1_OVERTEMP:
    case MX_CAN_FAULT_UART:
    case MX_CAN_FAULT_RS485:
    case MX_CAN_FAULT_CAN:
        return MX_CAN_FAULT_ACTION_AUTO_RECOVER;
    default:
        return MX_CAN_FAULT_ACTION_LATCH_STOP;
    }
}

mx_can_fault_action_t mx_can_status_fault_action(
        const mx_can_status_t *status)
{
    if (!mx_can_status_has_fault(status)) {
        return MX_CAN_FAULT_ACTION_NONE;
    }
    return mx_can_fault_action(
            mx_can_status_primary_fault_code(status));
}
