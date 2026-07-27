#include "mx_can_protocol.h"

#include <stddef.h>

static bool target_valid(int16_t target) {
    return target >= -30000 && target <= 30000;
}

static void put_i16_be(uint8_t *data, int16_t value) {
    const uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)(raw >> 8U);
    data[1] = (uint8_t)raw;
}

static int16_t get_i16_be(const uint8_t *data) {
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

bool mx_can_pack_command(const mx_can_command_t *command, uint8_t data[MX_CAN_FRAME_SIZE]) {
    if (command == NULL || data == NULL || !target_valid(command->axis1_target) ||
        !target_valid(command->axis2_target)) {
        return false;
    }

    data[0] = 0U;
    if (command->axis1_mode == MX_CAN_CONTROL_SPEED) data[0] |= 0x80U;
    if (command->axis2_mode == MX_CAN_CONTROL_SPEED) data[0] |= 0x40U;
    if (command->coast) data[0] |= 0x08U;
    if (command->reset) data[0] |= 0x04U;
    if (command->axis1_enable) data[0] |= 0x02U;
    if (command->axis2_enable) data[0] |= 0x01U;
    put_i16_be(&data[1], command->axis2_target);
    data[3] = command->axis2_ramp;
    put_i16_be(&data[4], command->axis1_target);
    data[6] = command->axis1_ramp;
    data[7] = 0U;
    return true;
}


bool mx_can_pack_axis2_speed_command(int16_t target, uint8_t ramp, bool enable, uint8_t data[MX_CAN_FRAME_SIZE]) {
    mx_can_command_t command = {
        .axis1_mode = MX_CAN_CONTROL_TORQUE,
        .axis2_mode = MX_CAN_CONTROL_SPEED,
        .axis1_enable = false,
        .axis2_enable = enable,
        .reset = false,
        .coast = false,
        .axis1_target = 0,
        .axis2_target = target,
        .axis1_ramp = 0,
        .axis2_ramp = ramp,
    };
    return mx_can_pack_command(&command, data);
}
bool mx_can_unpack_status(const uint8_t data[MX_CAN_FRAME_SIZE], mx_can_status_t *status) {
    if (data == NULL || status == NULL) return false;
    status->fault_code = data[0];
    status->controller_temperature_c = data[1];
    status->axis2_speed_rpm = get_i16_be(&data[2]);
    status->axis1_speed_rpm = get_i16_be(&data[4]);
    status->supply_voltage_decivolts = (uint16_t)(((uint16_t)data[6] << 8U) | data[7]);
    return true;
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
