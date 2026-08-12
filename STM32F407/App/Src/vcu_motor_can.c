#include "vcu_motor_can.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define VCU_RPM_FILTER_SHIFT 2

typedef struct {
    vcu_motor_can_state_t public_state;
    bool rpm_filter_initialized[VCU_MOTOR_COUNT];
    int16_t rpm_history_1[VCU_MOTOR_COUNT];
    int16_t rpm_history_2[VCU_MOTOR_COUNT];
    int32_t filtered_rpm[VCU_MOTOR_COUNT];
} vcu_motor_can_context_t;

static vcu_motor_can_context_t s_ctx;

static int16_t apply_direction_sign(int16_t value, int8_t direction_sign)
{
    if (direction_sign >= 0) {
        return value;
    }
    if (value == INT16_MIN) {
        return INT16_MAX;
    }
    return (int16_t)-value;
}

static int16_t normalize_vehicle_rpm(int16_t controller_rpm,
                                     int8_t direction_sign)
{
    const int16_t vehicle_rpm =
            apply_direction_sign(controller_rpm, direction_sign);

    if (vehicle_rpm > -VCU_VEHICLE_RPM_DEADZONE &&
        vehicle_rpm < VCU_VEHICLE_RPM_DEADZONE) {
        return 0;
    }
    return vehicle_rpm;
}

static int16_t median3(int16_t a, int16_t b, int16_t c)
{
    int16_t temporary;

    if (a > b) {
        temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c) {
        temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b) {
        temporary = a;
        a = b;
        b = temporary;
    }
    return b;
}

static int16_t filter_vehicle_rpm(unsigned int index,
                                  int16_t vehicle_rpm)
{
    int16_t median;

    if (!s_ctx.rpm_filter_initialized[index]) {
        s_ctx.rpm_history_1[index] = vehicle_rpm;
        s_ctx.rpm_history_2[index] = vehicle_rpm;
        s_ctx.filtered_rpm[index] = vehicle_rpm;
        s_ctx.rpm_filter_initialized[index] = true;
        return vehicle_rpm;
    }

    median = median3(s_ctx.rpm_history_2[index],
                     s_ctx.rpm_history_1[index], vehicle_rpm);
    s_ctx.rpm_history_2[index] = s_ctx.rpm_history_1[index];
    s_ctx.rpm_history_1[index] = vehicle_rpm;
    s_ctx.filtered_rpm[index] +=
            ((int32_t)median - s_ctx.filtered_rpm[index]) >>
            VCU_RPM_FILTER_SHIFT;

    if (vehicle_rpm == 0 &&
        abs(s_ctx.filtered_rpm[index]) < VCU_VEHICLE_RPM_DEADZONE) {
        s_ctx.filtered_rpm[index] = 0;
    }
    return (int16_t)s_ctx.filtered_rpm[index];
}

static int16_t controller_target(unsigned int index,
                                 int16_t vehicle_target)
{
    return apply_direction_sign(
            vehicle_target,
            s_ctx.public_state.controller[index].direction_sign);
}

static bool prepare_controller_frame(
        unsigned int index,
        const uint8_t data[MX_CAN_FRAME_SIZE],
        vcu_can_port_frame_t *frame)
{
    if (index >= VCU_MOTOR_COUNT || data == NULL || frame == NULL) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = s_ctx.public_state.controller[index].command_id;
    frame->extended_id = true;
    frame->data_length = MX_CAN_FRAME_SIZE;
    memcpy(frame->data, data, MX_CAN_FRAME_SIZE);
    return true;
}

static bool send_controller_pair(
        uint8_t data[VCU_MOTOR_COUNT][MX_CAN_FRAME_SIZE])
{
    vcu_can_port_frame_t frame[VCU_MOTOR_COUNT];
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        if (!prepare_controller_frame(
                    index, data[index], &frame[index])) {
            s_ctx.public_state.tx_failure_count++;
            return false;
        }
    }

    if (!vcu_can_port_send_pair(
                &frame[VCU_MOTOR_LEFT],
                &frame[VCU_MOTOR_RIGHT])) {
        s_ctx.public_state.tx_failure_count++;
        return false;
    }

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        s_ctx.public_state.last_tx_count[index]++;
        memcpy(s_ctx.public_state.last_tx_data[index],
               data[index], MX_CAN_FRAME_SIZE);
    }
    return true;
}

bool vcu_motor_can_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));

    /* Controller ID 1 is the vehicle-left motor. */
    s_ctx.public_state.controller[VCU_MOTOR_LEFT].status_id =
            MX_CAN_STATUS_ID;
    s_ctx.public_state.controller[VCU_MOTOR_LEFT].command_id =
            MX_CAN_COMMAND_ID;
    s_ctx.public_state.controller[VCU_MOTOR_LEFT].direction_sign = -1;

    /* Controller ID 2 is the vehicle-right motor. */
    s_ctx.public_state.controller[VCU_MOTOR_RIGHT].status_id =
            MX_CAN_STATUS_ID + 1U;
    s_ctx.public_state.controller[VCU_MOTOR_RIGHT].command_id =
            MX_CAN_COMMAND_ID + 1U;
    s_ctx.public_state.controller[VCU_MOTOR_RIGHT].direction_sign = 1;

    s_ctx.public_state.initialized = vcu_can_port_init();
    return s_ctx.public_state.initialized;
}

bool vcu_motor_can_process_frame(const vcu_can_port_frame_t *frame,
                                 uint32_t now_ms)
{
    unsigned int index;
    mx_can_status_t status;
    int8_t direction_sign;

    if (frame == NULL || !frame->extended_id || frame->remote_frame) {
        return false;
    }

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        if (frame->id ==
            s_ctx.public_state.controller[index].status_id) {
            break;
        }
    }
    if (index >= VCU_MOTOR_COUNT) {
        return false;
    }

    if (frame->data_length != MX_CAN_FRAME_SIZE ||
        !mx_can_unpack_status(frame->data, &status)) {
        s_ctx.public_state.ignored_frame_count++;
        return true;
    }

    direction_sign =
            s_ctx.public_state.controller[index].direction_sign;
    status.axis1_speed_rpm =
            normalize_vehicle_rpm(status.axis1_speed_rpm,
                                  direction_sign);
    status.axis2_speed_rpm =
            filter_vehicle_rpm(
                    index,
                    normalize_vehicle_rpm(
                            status.axis2_speed_rpm,
                            direction_sign));

    s_ctx.public_state.controller[index].status = status;
    s_ctx.public_state.controller[index].has_status = true;
    s_ctx.public_state.controller[index].status_frame_count++;
    s_ctx.public_state.controller[index].last_status_ms = now_ms;
    return true;
}

bool vcu_motor_can_get_state(vcu_motor_can_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    *state = s_ctx.public_state;
    return true;
}

bool vcu_motor_can_send_targets(int16_t left_target,
                                int16_t right_target,
                                uint8_t ramp,
                                bool enable)
{
    const int16_t target[VCU_MOTOR_COUNT] = {
        controller_target(VCU_MOTOR_LEFT, left_target),
        controller_target(VCU_MOTOR_RIGHT, right_target)
    };
    uint8_t data[VCU_MOTOR_COUNT][MX_CAN_FRAME_SIZE];
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        if (!mx_can_pack_axis2_speed_command(
                    target[index], ramp, enable, data[index])) {
            return false;
        }
    }
    return send_controller_pair(data);
}

bool vcu_motor_can_send_reset(void)
{
    const mx_can_command_t command = {
        .axis1_mode = MX_CAN_CONTROL_TORQUE,
        .axis2_mode = MX_CAN_CONTROL_TORQUE,
        .axis1_enable = false,
        .axis2_enable = false,
        .reset = true,
        .reboot = false,
        .coast = false,
        .axis1_target = 0,
        .axis2_target = 0,
        .axis1_ramp = 0U,
        .axis2_ramp = 0U,
    };
    uint8_t data[VCU_MOTOR_COUNT][MX_CAN_FRAME_SIZE];
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        if (!mx_can_pack_command(&command, data[index])) {
            return false;
        }
    }
    return send_controller_pair(data);
}

bool vcu_motor_can_send_reboot(vcu_motor_index_t controller)
{
    uint8_t data[MX_CAN_FRAME_SIZE];
    vcu_can_port_frame_t frame;
    const unsigned int index = (unsigned int)controller;

    if (index >= VCU_MOTOR_COUNT ||
        !mx_can_pack_reboot_command(data) ||
        !prepare_controller_frame(index, data, &frame)) {
        return false;
    }
    if (!vcu_can_port_send(&frame)) {
        s_ctx.public_state.tx_failure_count++;
        return false;
    }

    s_ctx.public_state.last_tx_count[index]++;
    memcpy(s_ctx.public_state.last_tx_data[index],
           data, MX_CAN_FRAME_SIZE);
    return true;
}
