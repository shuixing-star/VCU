#ifndef MX_CAN_PROTOCOL_H
#define MX_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define MX_CAN_DEFAULT_COMMAND_ID 0x0C01EFD0UL
#define MX_CAN_DEFAULT_ELECTRICAL_STATUS_ID 0x1801D0EFUL
#define MX_CAN_DEFAULT_STATE_STATUS_ID 0x1802D0EFUL
#define MX_CAN_FRAME_SIZE 8U

#define MX_CAN_COMMAND_RUN_MASK (1U << 0)
#define MX_CAN_COMMAND_SPEED_MODE_MASK (1U << 1)
#define MX_CAN_COMMAND_RESET_ENABLE_MASK (1U << 2)

#define MX_CAN_STATE_RUN_MASK (1U << 0)
#define MX_CAN_STATE_SPEED_MODE_MASK (1U << 1)
#define MX_CAN_STATE_RESET_ENABLE_MASK (1U << 2)

#define MX_CAN_MAX_TARGET_RAW 30000
#define MX_CAN_MAX_BRAKE_CURRENT_A 4000U

typedef enum {
    MX_CAN_FAULT_NONE = 0,
    MX_CAN_FAULT_DRIVER_1 = 1,
    MX_CAN_FAULT_DRIVER_2 = 2,
    MX_CAN_FAULT_DRIVER_3 = 3,
    MX_CAN_FAULT_DRIVER_4 = 4,
    MX_CAN_FAULT_OVERCURRENT = 5,
    MX_CAN_FAULT_OVERVOLTAGE = 6,
    MX_CAN_FAULT_UNDERVOLTAGE = 7,
    MX_CAN_FAULT_CONTROLLER_OVERTEMP = 8,
    MX_CAN_FAULT_RESERVED_9 = 9,
    MX_CAN_FAULT_RESERVED_10 = 10,
    MX_CAN_FAULT_MOTOR_2_OVERSPEED = 11,
    MX_CAN_FAULT_MOTOR_1_OVERSPEED = 12,
    MX_CAN_FAULT_MOTOR_2_OVERLOAD = 13,
    MX_CAN_FAULT_MOTOR_1_OVERLOAD = 14,
    MX_CAN_FAULT_MOTOR_2_PHASE_LOSS = 15,
    MX_CAN_FAULT_MOTOR_1_PHASE_LOSS = 16,
    MX_CAN_FAULT_MOTOR_2_BRAKE = 17,
    MX_CAN_FAULT_MOTOR_1_BRAKE = 18,
    MX_CAN_FAULT_MOTOR_2_ENCODER = 19,
    MX_CAN_FAULT_MOTOR_1_ENCODER = 20,
    MX_CAN_FAULT_MOTOR_2_OVERTEMP = 21,
    MX_CAN_FAULT_MOTOR_1_OVERTEMP = 22,
    MX_CAN_FAULT_MOTOR_2_HALL = 23,
    MX_CAN_FAULT_MOTOR_1_HALL = 24,
    MX_CAN_FAULT_MOTOR_2_STALL = 25,
    MX_CAN_FAULT_MOTOR_1_STALL = 26,
    MX_CAN_FAULT_UART = 27,
    MX_CAN_FAULT_RS485 = 28,
    MX_CAN_FAULT_CAN = 29,
    MX_CAN_FAULT_JOYSTICK = 30,
    MX_CAN_FAULT_THROTTLE = 31
} mx_can_fault_code_t;

typedef enum {
    MX_CAN_FAULT_ACTION_NONE = 0,
    MX_CAN_FAULT_ACTION_AUTO_RECOVER,
    MX_CAN_FAULT_ACTION_LATCH_STOP
} mx_can_fault_action_t;


typedef enum {
    MX_CAN_CONTROL_TORQUE = 0,
    MX_CAN_CONTROL_SPEED = 1
} mx_can_control_mode_t;

typedef struct {
    int16_t torque_deciamps;
    int16_t speed_rpm;
    mx_can_control_mode_t mode;
    bool run;
    bool reset_enable;
    uint16_t brake_current_a; /* GED D5-D6: little endian, 1 A/bit. */
    uint8_t heartbeat;
} mx_can_command_t;

typedef struct {
    uint16_t supply_voltage_decivolts;
    int16_t bus_current_deciamps;
    int16_t phase_current_deciamps;
    int16_t speed_rpm;
    int16_t controller_temperature_c;
    int16_t motor_temperature_c;
    uint8_t state_flags;
    uint8_t fault_flags[3];
    uint8_t heartbeat;
    uint8_t fault_code;
} mx_can_status_t;

bool mx_can_pack_command(const mx_can_command_t *command,
                         uint8_t data[MX_CAN_FRAME_SIZE]);
bool mx_can_pack_speed_command(int16_t speed_rpm,
                               bool run,
                               uint8_t heartbeat,
                               uint8_t data[MX_CAN_FRAME_SIZE]);
bool mx_can_unpack_electrical_status(
        const uint8_t data[MX_CAN_FRAME_SIZE],
        mx_can_status_t *status);
bool mx_can_unpack_state_status(
        const uint8_t data[MX_CAN_FRAME_SIZE],
        mx_can_status_t *status);
bool mx_can_status_has_fault(const mx_can_status_t *status);
uint8_t mx_can_status_primary_fault_code(const mx_can_status_t *status);
const char *mx_can_fault_name(uint8_t fault_code);
mx_can_fault_action_t mx_can_fault_action(uint8_t fault_code);
mx_can_fault_action_t mx_can_status_fault_action(
        const mx_can_status_t *status);

#endif
