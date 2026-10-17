#include "vcu_motor_can.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define VCU_RPM_FILTER_SHIFT 2

typedef struct {
    uint8_t tx_heartbeat[VCU_MOTOR_COUNT];
    vcu_motor_can_state_t public_state;
    bool rpm_filter_initialized[VCU_MOTOR_COUNT];
    int16_t rpm_history_1[VCU_MOTOR_COUNT];
    int16_t rpm_history_2[VCU_MOTOR_COUNT];
    int32_t filtered_rpm[VCU_MOTOR_COUNT];
} vcu_motor_can_context_t;

static vcu_motor_can_context_t s_ctx;
static bool controller_ids_valid(void)
{
    const uint32_t ids[] = {
        VCU_MOTOR_LEFT_ELECTRICAL_STATUS_ID,
        VCU_MOTOR_LEFT_STATE_STATUS_ID,
        VCU_MOTOR_LEFT_COMMAND_ID,
        VCU_MOTOR_RIGHT_ELECTRICAL_STATUS_ID,
        VCU_MOTOR_RIGHT_STATE_STATUS_ID,
        VCU_MOTOR_RIGHT_COMMAND_ID
    };
    unsigned int first;
    unsigned int second;

    for (first = 0U; first < (sizeof(ids) / sizeof(ids[0])); first++) {
        if (ids[first] == 0U || ids[first] > 0x1FFFFFFFUL) {
            return false;
        }
        for (second = first + 1U;
             second < (sizeof(ids) / sizeof(ids[0]));
             second++) {
            if (ids[first] == ids[second]) {
                return false;
            }
        }
    }
    return true;
}

static void refresh_complete_status(vcu_motor_controller_state_t *controller,
                                    uint32_t now_ms)
{
    uint32_t electrical_age;
    uint32_t state_age;
    uint32_t heartbeat_age;

    controller->has_status = controller->has_electrical_status &&
            controller->has_state_status &&
            controller->heartbeat_advanced;
    if (!controller->has_status) {
        return;
    }

    electrical_age = now_ms - controller->last_electrical_status_ms;
    state_age = now_ms - controller->last_state_status_ms;
    heartbeat_age = now_ms - controller->last_heartbeat_change_ms;
    controller->last_status_ms = controller->last_electrical_status_ms;
    if (state_age > electrical_age) {
        controller->last_status_ms = controller->last_state_status_ms;
        electrical_age = state_age;
    }
    if (heartbeat_age > electrical_age) {
        controller->last_status_ms = controller->last_heartbeat_change_ms;
    }
}

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

    s_ctx.public_state.controller[VCU_MOTOR_LEFT].electrical_status_id =
            VCU_MOTOR_LEFT_ELECTRICAL_STATUS_ID;
    s_ctx.public_state.controller[VCU_MOTOR_LEFT].state_status_id =
            VCU_MOTOR_LEFT_STATE_STATUS_ID;
    s_ctx.public_state.controller[VCU_MOTOR_LEFT].command_id =
            VCU_MOTOR_LEFT_COMMAND_ID;
    s_ctx.public_state.controller[VCU_MOTOR_LEFT].direction_sign = -1;

    s_ctx.public_state.controller[VCU_MOTOR_RIGHT].electrical_status_id =
            VCU_MOTOR_RIGHT_ELECTRICAL_STATUS_ID;
    s_ctx.public_state.controller[VCU_MOTOR_RIGHT].state_status_id =
            VCU_MOTOR_RIGHT_STATE_STATUS_ID;
    s_ctx.public_state.controller[VCU_MOTOR_RIGHT].command_id =
            VCU_MOTOR_RIGHT_COMMAND_ID;
    s_ctx.public_state.controller[VCU_MOTOR_RIGHT].direction_sign = 1;

    s_ctx.public_state.initialized = controller_ids_valid() &&
            vcu_can_port_init();
    return s_ctx.public_state.initialized;
}

bool vcu_motor_can_process_frame(const vcu_can_port_frame_t *frame,
                                 uint32_t now_ms)
{
    unsigned int index;
    bool electrical_frame = false;
    mx_can_status_t status;
    vcu_motor_controller_state_t *controller;

    if (frame == NULL || !frame->extended_id || frame->remote_frame) {
        return false;
    }

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        controller = &s_ctx.public_state.controller[index];
        if (frame->id == controller->electrical_status_id) {
            electrical_frame = true;
            break;
        }
        if (frame->id == controller->state_status_id) {
            break;
        }
    }
    if (index >= VCU_MOTOR_COUNT) {
        return false;
    }

    if (frame->data_length != MX_CAN_FRAME_SIZE) {
        s_ctx.public_state.ignored_frame_count++;
        return true;
    }

    controller = &s_ctx.public_state.controller[index];
    status = controller->status;
    if (electrical_frame) {
        if (!mx_can_unpack_electrical_status(frame->data, &status)) {
            s_ctx.public_state.ignored_frame_count++;
            return true;
        }
        status.speed_rpm = filter_vehicle_rpm(
                index,
                normalize_vehicle_rpm(status.speed_rpm,
                                      controller->direction_sign));
        controller->has_electrical_status = true;
        controller->electrical_status_frame_count++;
        controller->last_electrical_status_ms = now_ms;
    } else {
        if (!mx_can_unpack_state_status(frame->data, &status)) {
            s_ctx.public_state.ignored_frame_count++;
            return true;
        }
        if (!controller->heartbeat_seen) {
            controller->heartbeat_seen = true;
            controller->last_heartbeat = status.heartbeat;
        } else if (status.heartbeat != controller->last_heartbeat) {
            controller->heartbeat_advanced = true;
            controller->last_heartbeat = status.heartbeat;
            controller->last_heartbeat_change_ms = now_ms;
        }
        controller->has_state_status = true;

        controller->state_status_frame_count++;
        controller->last_state_status_ms = now_ms;
    }

    controller->status = status;
    controller->status_frame_count++;
    refresh_complete_status(controller, now_ms);
    return true;
}

bool vcu_motor_can_controller_recent(
        const vcu_motor_controller_state_t *controller,
        uint32_t now_ms,
        uint32_t timeout_ms)
{
    if (controller == NULL || !controller->has_status) {
        return false;
    }
    return (uint32_t)(now_ms - controller->last_electrical_status_ms) <=
                   timeout_ms &&
           (uint32_t)(now_ms - controller->last_state_status_ms) <=
                   timeout_ms &&
           (uint32_t)(now_ms - controller->last_heartbeat_change_ms) <=
                   timeout_ms;
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
                                bool enable)
{
    const int16_t target[VCU_MOTOR_COUNT] = {
        controller_target(VCU_MOTOR_LEFT, left_target),
        controller_target(VCU_MOTOR_RIGHT, right_target)
    };
    uint8_t data[VCU_MOTOR_COUNT][MX_CAN_FRAME_SIZE];
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        if (!mx_can_pack_speed_command(
                    target[index], enable,
                    s_ctx.tx_heartbeat[index], data[index])) {
            return false;
        }
    }
    if (!send_controller_pair(data)) {
        return false;
    }
    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        s_ctx.tx_heartbeat[index]++;
    }
    return true;
}

/* Zero disables that side; runtime gates each current by fresh feedback. */
bool vcu_motor_can_send_brake(uint16_t left_current_a, uint16_t right_current_a)
{
    const uint16_t current[VCU_MOTOR_COUNT] = {left_current_a, right_current_a};
    uint8_t data[VCU_MOTOR_COUNT][MX_CAN_FRAME_SIZE];
    unsigned int index;
    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        const mx_can_command_t command = {
            .torque_deciamps = 0, .speed_rpm = 0,
            .mode = MX_CAN_CONTROL_TORQUE,
            .run = current[index] != 0U, .reset_enable = false,
            .brake_current_a = current[index],
            .heartbeat = s_ctx.tx_heartbeat[index]
        };
        if (!mx_can_pack_command(&command, data[index])) return false;
    }
    if (!send_controller_pair(data)) return false;
    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        s_ctx.tx_heartbeat[index]++;
    }
    return true;
}

bool vcu_motor_can_send_reset(void)
{
    uint8_t data[VCU_MOTOR_COUNT][MX_CAN_FRAME_SIZE];
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        const mx_can_command_t command = {
            .torque_deciamps = 0,
            .speed_rpm = 0,
            .mode = MX_CAN_CONTROL_TORQUE,
            .run = false,
            .reset_enable = true,
            .brake_current_a = 0U,
            .heartbeat = s_ctx.tx_heartbeat[index]
        };
        if (!mx_can_pack_command(&command, data[index])) {
            return false;
        }
    }
    if (!send_controller_pair(data)) {
        return false;
    }
    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        s_ctx.tx_heartbeat[index]++;
    }
    return true;
}
