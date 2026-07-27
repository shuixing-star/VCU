#ifndef MX_CAN_PROTOCOL_H
#define MX_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define MX_CAN_COMMAND_ID 0x1801E600UL
#define MX_CAN_STATUS_ID 0x1801E001UL
#define MX_CAN_FRAME_SIZE 8U

typedef enum {
    MX_CAN_CONTROL_TORQUE = 0,
    MX_CAN_CONTROL_SPEED = 1
} mx_can_control_mode_t;

typedef struct {
    mx_can_control_mode_t axis1_mode;
    mx_can_control_mode_t axis2_mode;
    bool axis1_enable;
    bool axis2_enable;
    bool reset;
    bool coast;
    int16_t axis1_target;
    int16_t axis2_target;
    uint8_t axis1_ramp;
    uint8_t axis2_ramp;
} mx_can_command_t;

typedef struct {
    uint8_t fault_code;
    uint8_t controller_temperature_c;
    int16_t axis1_speed_rpm;
    int16_t axis2_speed_rpm;
    uint16_t supply_voltage_decivolts;
} mx_can_status_t;

bool mx_can_pack_command(const mx_can_command_t *command, uint8_t data[MX_CAN_FRAME_SIZE]);
bool mx_can_pack_axis2_speed_command(int16_t target, uint8_t ramp, bool enable, uint8_t data[MX_CAN_FRAME_SIZE]);
bool mx_can_unpack_status(const uint8_t data[MX_CAN_FRAME_SIZE], mx_can_status_t *status);
const char *mx_can_fault_name(uint8_t fault_code);

#endif
