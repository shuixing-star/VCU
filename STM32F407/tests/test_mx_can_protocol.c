#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "mx_can_protocol.h"

static void test_speed_command(void)
{
    uint8_t data[MX_CAN_FRAME_SIZE];
    const uint8_t expected[MX_CAN_FRAME_SIZE] = {
        0x00U, 0x7DU, 0x64U, 0x7DU,
        0x03U, 0x00U, 0x00U, 0x2AU
    };

    assert(mx_can_pack_speed_command(100, true, 0x2AU, data));
    assert(memcmp(data, expected, sizeof(expected)) == 0);
}

static void test_full_command(void)
{
    const mx_can_command_t command = {
        .torque_deciamps = -100,
        .speed_rpm = 1234,
        .mode = MX_CAN_CONTROL_TORQUE,
        .run = false,
        .reset_enable = true,
        .brake_current_a = 500U,
        .heartbeat = 0x7EU
    };
    const uint8_t expected[MX_CAN_FRAME_SIZE] = {
        0x9CU, 0x7CU, 0xD2U, 0x81U,
        0x04U, 0xF4U, 0x01U, 0x7EU
    };
    uint8_t data[MX_CAN_FRAME_SIZE];

    assert(mx_can_pack_command(&command, data));
    assert(memcmp(data, expected, sizeof(expected)) == 0);
}

static void test_electrical_status(void)
{
    const uint8_t data[MX_CAN_FRAME_SIZE] = {
        0x7DU, 0x00U, 0x9CU, 0x7CU,
        0xFAU, 0x7DU, 0x64U, 0x7DU
    };
    mx_can_status_t status = {0};

    assert(mx_can_unpack_electrical_status(data, &status));
    assert(status.supply_voltage_decivolts == 125U);
    assert(status.bus_current_deciamps == -100);
    assert(status.phase_current_deciamps == 250);
    assert(status.speed_rpm == 100);
}

static void test_captured_zero_electrical_status(void)
{
    const uint8_t data[MX_CAN_FRAME_SIZE] = {
        0xFAU, 0x03U, 0x00U, 0x7DU,
        0x00U, 0x7DU, 0x00U, 0x7DU
    };
    mx_can_status_t status = {0};

    assert(mx_can_unpack_electrical_status(data, &status));
    assert(status.supply_voltage_decivolts == 1018U);
    assert(status.bus_current_deciamps == 0);
    assert(status.phase_current_deciamps == 0);
    assert(status.speed_rpm == 0);
}

static void test_state_status(void)
{
    const uint8_t overcurrent[MX_CAN_FRAME_SIZE] = {
        82U, 30U, 0x03U, 0x01U, 0x00U, 0x00U, 0x00U, 0x12U
    };
    const uint8_t overvoltage[MX_CAN_FRAME_SIZE] = {
        82U, 75U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U, 0x13U
    };
    const uint8_t captured[MX_CAN_FRAME_SIZE] = {
        0x4FU, 0x3CU, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0xB2U
    };
    const uint8_t invalid[MX_CAN_FRAME_SIZE] = {
        0x55U, 0x55U, 0x55U, 0x55U,
        0x55U, 0x55U, 0x55U, 0x55U
    };
    mx_can_status_t status = {0};

    assert(mx_can_unpack_state_status(overcurrent, &status));
    assert(status.controller_temperature_c == 42);
    assert(status.motor_temperature_c == -10);
    assert(status.state_flags == 0x03U);
    assert(status.heartbeat == 0x12U);
    assert(status.fault_code == MX_CAN_FAULT_OVERCURRENT);
    assert(mx_can_status_fault_action(&status) ==
           MX_CAN_FAULT_ACTION_LATCH_STOP);

    assert(mx_can_unpack_state_status(overvoltage, &status));
    assert(status.fault_code == MX_CAN_FAULT_OVERVOLTAGE);
    assert(mx_can_status_fault_action(&status) ==
           MX_CAN_FAULT_ACTION_AUTO_RECOVER);

    assert(mx_can_unpack_state_status(captured, &status));
    assert(status.controller_temperature_c == 39);
    assert(status.motor_temperature_c == 20);
    assert(status.heartbeat == 0xB2U);
    assert(!mx_can_unpack_state_status(invalid, &status));
}

int main(void)
{
    test_speed_command();
    test_full_command();
    test_electrical_status();
    test_captured_zero_electrical_status();
    test_state_status();
    return 0;
}
