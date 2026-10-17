#include "vcu_esp_link.h"

#include <string.h>

static bool s_command_pending;
static bool s_has_sequence;
static uint8_t s_last_sequence;
static vcu_gateway_command_t s_pending_command;
static bool s_command_seen;
static bool s_invalid_command_seen;
static bool s_duplicate_command_seen;

void vcu_esp_link_init(void)
{
    s_command_pending = false;
    s_has_sequence = false;
    s_last_sequence = 0U;
    memset(&s_pending_command, 0, sizeof(s_pending_command));
    s_command_seen = false;
    s_invalid_command_seen = false;
    s_duplicate_command_seen = false;
}

bool vcu_esp_link_process_frame(const vcu_can_port_frame_t *frame)
{
    vcu_gateway_command_t command;

    if (frame == NULL || frame->id != VCU_GATEWAY_ID_COMMAND) {
        return false;
    }
    if (frame->extended_id || frame->remote_frame ||
        frame->data_length != 8U ||
        !vcu_gateway_unpack_command(frame->data, &command)) {
        s_invalid_command_seen = true;
        return true;
    }
    s_command_seen = true;

    if (s_has_sequence && command.sequence == s_last_sequence) {
        s_duplicate_command_seen = true;
        return true;
    }

    s_last_sequence = command.sequence;
    s_has_sequence = true;
    s_pending_command = command;
    s_command_pending = true;
    return true;
}

bool vcu_esp_link_take_command(vcu_gateway_command_t *command)
{
    if (command == NULL || !s_command_pending) {
        return false;
    }

    *command = s_pending_command;
    s_command_pending = false;
    return true;
}

bool vcu_esp_link_command_seen(void)
{
    return s_command_seen;
}

bool vcu_esp_link_invalid_command_seen(void)
{
    return s_invalid_command_seen;
}

bool vcu_esp_link_duplicate_command_seen(void)
{
    return s_duplicate_command_seen;
}

bool vcu_esp_link_send(uint32_t id, const uint8_t data[8])
{
    vcu_can_port_frame_t frame;

    if (data == NULL || id > 0x7FFU) {
        return false;
    }

    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.data_length = 8U;
    memcpy(frame.data, data, 8U);
    return vcu_can_port_send(&frame);
}

bool vcu_esp_link_send_diagnostic(uint32_t id, const uint8_t data[8])
{
    vcu_can_port_frame_t frame;

    if (data == NULL || id > 0x7FFU) {
        return false;
    }

    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.data_length = 8U;
    memcpy(frame.data, data, 8U);
#ifdef VCU_FLIGHT_LOG_ENABLED
    return vcu_can_port_send_diagnostic_async(&frame);
#else
    return vcu_can_port_send_diagnostic(&frame);
#endif
}
