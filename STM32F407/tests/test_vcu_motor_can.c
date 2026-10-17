#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "vcu_motor_can.h"

static vcu_can_port_frame_t s_last_pair[VCU_MOTOR_COUNT];

bool vcu_can_port_init(void)
{
    return true;
}

bool vcu_can_port_read(vcu_can_port_frame_t *frame)
{
    (void)frame;
    return false;
}

bool vcu_can_port_send(const vcu_can_port_frame_t *frame)
{
    (void)frame;
    return true;
}

bool vcu_can_port_send_pair(const vcu_can_port_frame_t *first,
                            const vcu_can_port_frame_t *second)
{
    s_last_pair[VCU_MOTOR_LEFT] = *first;
    s_last_pair[VCU_MOTOR_RIGHT] = *second;
    return true;
}

uint32_t vcu_can_port_rx_overflow_count(void) { return 0U; }
uint32_t vcu_can_port_rx_error_count(void) { return 0U; }
uint32_t vcu_can_port_tx_error_count(void) { return 0U; }

static vcu_can_port_frame_t make_frame(uint32_t id,
                                       const uint8_t data[8])
{
    vcu_can_port_frame_t frame = {
        .id = id,
        .extended_id = true,
        .remote_frame = false,
        .data_length = 8U
    };
    unsigned int index;

    for (index = 0U; index < 8U; index++) {
        frame.data[index] = data[index];
    }
    return frame;
}

static void test_dual_feedback_and_heartbeat(void)
{
    const uint8_t electrical[8] = {
        0x7DU, 0x00U, 0x00U, 0x7DU,
        0x00U, 0x7DU, 0xE8U, 0x80U
    };
    uint8_t state_data[8] = {
        82U, 75U, 0x03U, 0U, 0U, 0U, 0U, 10U
    };
    vcu_can_port_frame_t frame;
    vcu_motor_can_state_t state;

    assert(vcu_motor_can_init());
    assert(vcu_motor_can_get_state(&state));
    assert(state.controller[VCU_MOTOR_LEFT].command_id ==
           VCU_MOTOR_LEFT_DRIVER_RECEIVE_ID);
    assert(state.controller[VCU_MOTOR_LEFT].electrical_status_id ==
           VCU_MOTOR_LEFT_DRIVER_SEND_ID);
    assert(state.controller[VCU_MOTOR_LEFT].state_status_id ==
           VCU_MOTOR_LEFT_DRIVER_SEND_ID +
                   VCU_MOTOR_SECOND_STATUS_ID_OFFSET);

    frame = make_frame(VCU_MOTOR_LEFT_ELECTRICAL_STATUS_ID, electrical);
    assert(vcu_motor_can_process_frame(&frame, 0U));
    frame = make_frame(VCU_MOTOR_LEFT_STATE_STATUS_ID, state_data);
    assert(vcu_motor_can_process_frame(&frame, 0U));
    assert(vcu_motor_can_get_state(&state));
    assert(!vcu_motor_can_controller_recent(
            &state.controller[VCU_MOTOR_LEFT], 0U, 500U));

    state_data[7] = 11U;
    frame = make_frame(VCU_MOTOR_LEFT_STATE_STATUS_ID, state_data);
    assert(vcu_motor_can_process_frame(&frame, 20U));
    assert(vcu_motor_can_get_state(&state));
    assert(vcu_motor_can_controller_recent(
            &state.controller[VCU_MOTOR_LEFT], 20U, 500U));
    assert(state.controller[VCU_MOTOR_LEFT].status.speed_rpm == -1000);
    assert(!vcu_motor_can_controller_recent(
            &state.controller[VCU_MOTOR_LEFT], 521U, 500U));
}

/* RENG wire IDs are independent of the VCU configuration macros. */
static void test_reng_node_240_feedback(void)
{
    const uint8_t electrical[8] = {0x7DU, 0U, 0U, 0x7DU, 0U, 0x7DU, 0xE8U, 0x80U};
    uint8_t status[8] = {82U, 75U, 3U, 0U, 0U, 0U, 0U, 10U};
    vcu_can_port_frame_t frame;
    vcu_motor_can_state_t state;

    assert(vcu_motor_can_init());
    frame = make_frame(0x1801D0F0UL, electrical);
    assert(vcu_motor_can_process_frame(&frame, 0U));
    frame = make_frame(0x1802D0F0UL, status);
    assert(vcu_motor_can_process_frame(&frame, 0U));
    status[7]++;
    frame = make_frame(0x1802D0F0UL, status);
    assert(vcu_motor_can_process_frame(&frame, 20U));
    assert(vcu_motor_can_get_state(&state));
    assert(!vcu_motor_can_controller_recent(&state.controller[VCU_MOTOR_LEFT], 20U, 500U));
    assert(vcu_motor_can_controller_recent(&state.controller[VCU_MOTOR_RIGHT], 20U, 500U));

    frame = make_frame(0x1801D0EFUL, electrical);
    assert(vcu_motor_can_process_frame(&frame, 20U));
    frame = make_frame(0x1802D0EFUL, status);
    assert(vcu_motor_can_process_frame(&frame, 20U));
    status[7]++;
    frame = make_frame(0x1802D0EFUL, status);
    assert(vcu_motor_can_process_frame(&frame, 40U));
    assert(vcu_motor_can_get_state(&state));
    assert(vcu_motor_can_controller_recent(&state.controller[VCU_MOTOR_LEFT], 40U, 500U));
    assert(vcu_motor_can_controller_recent(&state.controller[VCU_MOTOR_RIGHT], 40U, 500U));

    frame = make_frame(0x1801D1EFUL, electrical);
    assert(!vcu_motor_can_process_frame(&frame, 600U));
    frame = make_frame(0x1802D1EFUL, status);
    assert(!vcu_motor_can_process_frame(&frame, 600U));
    assert(vcu_motor_can_get_state(&state));
    assert(!vcu_motor_can_controller_recent(&state.controller[VCU_MOTOR_RIGHT], 600U, 500U));
}

static void test_command_pair(void)
{
    assert(vcu_motor_can_send_targets(100, 200, true));
    assert(s_last_pair[VCU_MOTOR_LEFT].id == 0x0C01EFD0UL);
    assert(s_last_pair[VCU_MOTOR_RIGHT].id == 0x0C01F0D0UL);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[2] == 0x9CU);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[3] == 0x7CU);
    assert(s_last_pair[VCU_MOTOR_RIGHT].data[2] == 0xC8U);
    assert(s_last_pair[VCU_MOTOR_RIGHT].data[3] == 0x7DU);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[4] == 0x03U);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[7] == 0U);

    assert(vcu_motor_can_send_reset());
    assert(s_last_pair[VCU_MOTOR_LEFT].id == 0x0C01EFD0UL);
    assert(s_last_pair[VCU_MOTOR_RIGHT].id == 0x0C01F0D0UL);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[0] == 0x00U);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[1] == 0x7DU);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[2] == 0x00U);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[3] == 0x7DU);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[4] == 0x04U);
    assert(s_last_pair[VCU_MOTOR_LEFT].data[7] == 1U);
}

int main(void)
{
    test_dual_feedback_and_heartbeat();
    test_reng_node_240_feedback();
    test_command_pair();
    return 0;
}
