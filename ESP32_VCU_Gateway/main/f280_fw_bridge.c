#include "f280_fw_bridge.h"

#include "buffer.h"
#include "commands.h"
#include "datatypes.h"
#include "f280_fw_stream.h"
#include "hwconf/hw.h"
#include "terminal.h"
#ifdef HW_VCU_STM32_GATEWAY
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vcu_stm32_gateway.h"
#else
#include "vcu_drive_runtime.h"
#endif

#ifndef F280_FW_TARGET_CAN_ID
#define F280_FW_TARGET_CAN_ID 1U
#endif

#ifdef HW_VCU_STM32_GATEWAY
#define FW_SAFE_FLAG_RUN_REQUESTED (1U << 0)
#define FW_SAFE_FLAG_LEFT_ONLINE (1U << 2)
#define FW_SAFE_FLAG_RIGHT_ONLINE (1U << 3)
#define FW_SAFE_FLAG_OUTPUT_ENABLED (1U << 4)
#define FW_SAFE_RC_FLAG_ARM (1U << 1)
#define FW_SAFE_MOTOR_MAX_AGE_10MS 50U
#define FW_SAFE_STOP_TIMEOUT_MS 500U
#define FW_SAFE_STOP_RETRY_MS 100U
#define FW_SAFE_POLL_MS 10U
#define FW_RESUME_PERIOD_MS 5U
#define FW_RESUME_ACCEL_RPM_S 2550U
#endif

static uint8_t s_upload_target = 0xFFU;
static bool s_initialized;
static bool s_terminal_registered;

typedef enum {
    BRIDGE_REJECT_NONE = 0,
    BRIDGE_REJECT_TARGET,
    BRIDGE_REJECT_ARGUMENT,
    BRIDGE_REJECT_DECODE,
    BRIDGE_REJECT_UNSUPPORTED,
    BRIDGE_REJECT_INIT,
    BRIDGE_REJECT_ERASE_FORMAT,
    BRIDGE_REJECT_VEHICLE_UNSAFE,
    BRIDGE_REJECT_F280_BEGIN,
    BRIDGE_REJECT_WRITE_FORMAT,
    BRIDGE_REJECT_NO_SESSION,
    BRIDGE_REJECT_WRITE,
    BRIDGE_REJECT_FINISH
} bridge_reject_t;

static uint32_t s_bridge_dispatch_count;
static uint32_t s_bridge_request_count;
static uint8_t s_bridge_last_target = 0xFFU;
static uint8_t s_bridge_last_raw_command = 0xFFU;
static uint8_t s_bridge_last_command = 0xFFU;
static uint32_t s_bridge_last_length;
static uint32_t s_bridge_last_payload_index;
static bridge_reject_t s_bridge_last_reject;
static const char *s_bridge_last_safety_block = "not-evaluated";

#ifdef HW_VCU_STM32_GATEWAY
static bool stm32_state_is_safe(const vcu_stm32_gateway_state_t *state,
                                bool require_stopped)
{
    uint8_t flags;

    if (state == NULL || !state->online || !state->protocol_compatible ||
        !state->has_heartbeat || !state->has_rc_input ||
        !state->has_targets) {
        s_bridge_last_safety_block = "state-unavailable";
        return false;
    }
    flags = state->heartbeat.flags;
    if ((flags & FW_SAFE_FLAG_OUTPUT_ENABLED) != 0U) {
        s_bridge_last_safety_block = "output-enabled";
        return false;
    }
    if (require_stopped &&
        (flags & FW_SAFE_FLAG_RUN_REQUESTED) != 0U) {
        s_bridge_last_safety_block = "run-requested";
        return false;
    }
    if ((state->rc_input.flags & FW_SAFE_RC_FLAG_ARM) != 0U) {
        s_bridge_last_safety_block = "rc-armed";
        return false;
    }
    if (state->targets.left_target_rpm != 0 ||
        state->targets.right_target_rpm != 0) {
        s_bridge_last_safety_block = "target-nonzero";
        return false;
    }
    if ((flags & FW_SAFE_FLAG_LEFT_ONLINE) != 0U &&
        (!state->has_motor_left ||
         state->motor_left.age_10ms > FW_SAFE_MOTOR_MAX_AGE_10MS ||
         state->motor_left.speed_rpm <= -10 ||
         state->motor_left.speed_rpm >= 10)) {
        s_bridge_last_safety_block = "left-motor-not-ready";
        return false;
    }
    if ((flags & FW_SAFE_FLAG_RIGHT_ONLINE) != 0U &&
        (!state->has_motor_right ||
         state->motor_right.age_10ms > FW_SAFE_MOTOR_MAX_AGE_10MS ||
         state->motor_right.speed_rpm <= -10 ||
         state->motor_right.speed_rpm >= 10)) {
        s_bridge_last_safety_block = "right-motor-not-ready";
        return false;
    }
    s_bridge_last_safety_block = "none";
    return true;
}
#endif

#ifndef HW_VCU_STM32_GATEWAY
static bool vehicle_is_safe(void)
{
    vcu_drive_runtime_state_t state;

    if (!vcu_drive_runtime_get_state(&state) || !state.initialized ||
        state.requested_run || state.state == VCU_DRIVE_STATE_ARMED ||
        state.state == VCU_DRIVE_STATE_ARMING_NEUTRAL) {
        return false;
    }
    return state.latest_left_target == 0 && state.latest_right_target == 0 &&
           state.average_rpm > -10 && state.average_rpm < 10;
}
#endif

static bool prepare_vehicle_safe(void)
{
#ifdef HW_VCU_STM32_GATEWAY
    vcu_stm32_gateway_state_t state;
    uint32_t elapsed_ms;

    if (!vcu_stm32_gateway_get_state(&state)) {
        s_bridge_last_safety_block = "state-unavailable";
        return false;
    }
    if (!stm32_state_is_safe(&state, false)) {
        return false;
    }
    if (stm32_state_is_safe(&state, true)) {
        return true;
    }
    for (elapsed_ms = 0U; elapsed_ms < FW_SAFE_STOP_TIMEOUT_MS;
         elapsed_ms += FW_SAFE_POLL_MS) {
        if ((elapsed_ms % FW_SAFE_STOP_RETRY_MS) == 0U &&
            !vcu_stm32_gateway_request_stop()) {
            s_bridge_last_safety_block = "stop-command-tx";
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(FW_SAFE_POLL_MS));
        if (!vcu_stm32_gateway_get_state(&state)) {
            s_bridge_last_safety_block = "state-unavailable";
            continue;
        }
        if (stm32_state_is_safe(&state, true)) {
            return true;
        }
    }
    return false;
#else
    bool safe = vehicle_is_safe();

    s_bridge_last_safety_block = safe ? "none" : "local-drive-state";
    return safe;
#endif
}

static void send_simple_ack(f280_fw_reply_fn reply, uint8_t command, bool ok)
{
    unsigned char response[2];
    response[0] = command;
    response[1] = ok ? 1U : 0U;
    reply(response, sizeof(response));
}

static void send_write_ack(f280_fw_reply_fn reply, uint8_t command,
                           bool ok, uint32_t offset)
{
    unsigned char response[6];
    int32_t index = 0;

    response[index++] = command;
    response[index++] = ok ? 1U : 0U;
    buffer_append_uint32(response, offset, &index);
    reply(response, (unsigned int)index);
}
static const char *stream_state_name(f280_fw_stream_state_t state)
{
    switch (state) {
    case F280_FW_STREAM_IDLE: return "idle";
    case F280_FW_STREAM_ENTERING: return "entering";
    case F280_FW_STREAM_ERASING: return "erasing";
    case F280_FW_STREAM_PROGRAMMING: return "programming";
    case F280_FW_STREAM_VERIFYING: return "verifying";
    case F280_FW_STREAM_COMPLETE: return "complete";
    case F280_FW_STREAM_FAILED: return "failed";
    default: return "unknown";
    }
}

static const char *bridge_reject_name(bridge_reject_t reject)
{
    switch (reject) {
    case BRIDGE_REJECT_NONE: return "none";
    case BRIDGE_REJECT_TARGET: return "target-id";
    case BRIDGE_REJECT_ARGUMENT: return "argument";
    case BRIDGE_REJECT_DECODE: return "tool-command-decode";
    case BRIDGE_REJECT_UNSUPPORTED: return "unsupported-command";
    case BRIDGE_REJECT_INIT: return "stream-init";
    case BRIDGE_REJECT_ERASE_FORMAT: return "erase-format";
    case BRIDGE_REJECT_VEHICLE_UNSAFE: return "vehicle-unsafe";
    case BRIDGE_REJECT_F280_BEGIN: return "f280-enter-or-erase";
    case BRIDGE_REJECT_WRITE_FORMAT: return "write-format";
    case BRIDGE_REJECT_NO_SESSION: return "no-upload-session";
    case BRIDGE_REJECT_WRITE: return "f280-write";
    case BRIDGE_REJECT_FINISH: return "f280-finish";
    default: return "unknown";
    }
}

static void terminal_f280_fw_status(int argc, const char **argv)
{
    f280_fw_stream_status_t stream;
    f280_fw_stream_status_t retained;
    bool retained_valid;

    (void)argc;
    (void)argv;
    f280_fw_stream_get_status(&stream);
    retained_valid = f280_fw_stream_get_retained_status(&retained);
    commands_printf(
        "F280 firmware stream\n"
        "bridge diag : v5\n"
        "dispatches  : %lu\n"
        "firmware req: %lu\n"
        "last target : %u\n"
        "last command: raw=%u normalized=%u\n"
        "last length : %lu payload=%lu\n"
        "last reject : %s\n"
        "safety block: %s\n"
        "active      : %d\n"
        "image bytes : %lu\n"
        "target CAN  : %u\n"
        "state       : %s\n"
        "next offset : %lu\n"
        "retries     : %lu\n"
        "last status : %u\n"
        "retained log: %s\n"
        "ret state   : %s\n"
        "ret active  : %d\n"
        "ret image   : %lu\n"
        "ret offset  : %lu\n"
        "ret retries : %lu\n"
        "ret status  : %u",
        (unsigned long)s_bridge_dispatch_count,
        (unsigned long)s_bridge_request_count,
        s_bridge_last_target,
        s_bridge_last_raw_command,
        s_bridge_last_command,
        (unsigned long)s_bridge_last_length,
        (unsigned long)s_bridge_last_payload_index,
        bridge_reject_name(s_bridge_last_reject),
        s_bridge_last_safety_block, stream.active,
        (unsigned long)stream.image_size, stream.target_id,
        stream_state_name(stream.state), (unsigned long)stream.next_offset,
        (unsigned long)stream.retry_count, stream.last_status,
        retained_valid ? "valid" : "none",
        retained_valid ? stream_state_name(retained.state) : "none",
        retained_valid ? retained.active : 0,
        (unsigned long)(retained_valid ? retained.image_size : 0U),
        (unsigned long)(retained_valid ? retained.next_offset : 0U),
        (unsigned long)(retained_valid ? retained.retry_count : 0U),
        retained_valid ? retained.last_status : 0U);
}


bool f280_fw_bridge_init(void)
{
    if (s_initialized) {
        return true;
    }
    if (!s_terminal_registered) {
        terminal_register_command_callback(
            "f280_fw_status",
            "Print direct F280 CAN firmware stream status.",
            0, terminal_f280_fw_status);
        s_terminal_registered = true;
    }
    s_initialized = f280_fw_stream_init();
    return s_initialized;
}

static bool is_firmware_command(uint8_t command)
{
    switch (command) {
    case COMM_JUMP_TO_BOOTLOADER:
    case COMM_ERASE_NEW_APP:
    case COMM_WRITE_NEW_APP_DATA:
    case COMM_JUMP_TO_BOOTLOADER_ALL_CAN:
    case COMM_ERASE_NEW_APP_ALL_CAN:
    case COMM_WRITE_NEW_APP_DATA_ALL_CAN:
    case COMM_ERASE_BOOTLOADER:
    case COMM_ERASE_BOOTLOADER_ALL_CAN:
    case COMM_WRITE_NEW_APP_DATA_LZO:
    case COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO:
    case COMM_JUMP_TO_BOOTLOADER_HW:
    case COMM_ERASE_NEW_APP_HW:
    case COMM_WRITE_NEW_APP_DATA_HW:
    case COMM_ERASE_BOOTLOADER_HW:
    case COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW:
    case COMM_ERASE_NEW_APP_ALL_CAN_HW:
    case COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW:
    case COMM_ERASE_BOOTLOADER_ALL_CAN_HW:
        return true;
    default:
        return false;
    }
}


static bool decode_tool_command(const uint8_t *packet,
                                unsigned int length,
                                uint8_t *command,
                                unsigned int *payload_index)
{
    unsigned int index;

    if (packet == NULL || length == 0U || command == NULL ||
        payload_index == NULL) {
        return false;
    }

    *command = packet[0];
    *payload_index = 1U;
    switch (*command) {
    case COMM_JUMP_TO_BOOTLOADER_ALL_CAN:
        *command = COMM_JUMP_TO_BOOTLOADER;
        return true;
    case COMM_ERASE_NEW_APP_ALL_CAN:
        *command = COMM_ERASE_NEW_APP;
        return true;
    case COMM_WRITE_NEW_APP_DATA_ALL_CAN:
        *command = COMM_WRITE_NEW_APP_DATA;
        return true;
    case COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW:
    case COMM_ERASE_NEW_APP_ALL_CAN_HW:
    case COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW:
        if (length < 3U) {
            return false;
        }
        index = 2U; /* Skip command and HW_TYPE. */
        while (index < length && packet[index] != 0U) {
            index++;
        }
        if (index >= length) {
            return false;
        }
        *payload_index = index + 1U;
        if (*command == COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW) {
            *command = COMM_JUMP_TO_BOOTLOADER;
        } else if (*command == COMM_ERASE_NEW_APP_ALL_CAN_HW) {
            *command = COMM_ERASE_NEW_APP;
        } else {
            *command = COMM_WRITE_NEW_APP_DATA;
        }
        return true;
    default:
        return true;
    }
}

bool f280_fw_bridge_handle(uint8_t target_id,
                           const uint8_t *packet,
                           unsigned int length,
                           f280_fw_reply_fn reply)
{
    uint8_t command;
    int32_t index;
    uint32_t value;
    bool ok;
    unsigned int payload_index;

    s_bridge_dispatch_count++;
    if (packet == NULL || length == 0U ||
        !is_firmware_command(packet[0])) {
        return false;
    }
    s_bridge_request_count++;
    s_bridge_last_target = target_id;
    s_bridge_last_length = length;
    s_bridge_last_payload_index = 0U;
    s_bridge_last_raw_command =
            (packet != NULL && length != 0U) ? packet[0] : 0xFFU;
    s_bridge_last_command = s_bridge_last_raw_command;
    s_bridge_last_reject = BRIDGE_REJECT_NONE;

    if (target_id != F280_FW_TARGET_CAN_ID) {
        s_bridge_last_reject = BRIDGE_REJECT_TARGET;
        return false;
    }
    if (packet == NULL || length == 0U || reply == NULL) {
        s_bridge_last_reject = BRIDGE_REJECT_ARGUMENT;
        return false;
    }
    if (!decode_tool_command(packet, length, &command, &payload_index)) {
        s_bridge_last_reject = BRIDGE_REJECT_DECODE;
        return true;
    }
    s_bridge_last_command = command;
    s_bridge_last_payload_index = payload_index;
    if (command != COMM_JUMP_TO_BOOTLOADER &&
        command != COMM_ERASE_NEW_APP &&
        command != COMM_WRITE_NEW_APP_DATA &&
        command != COMM_WRITE_NEW_APP_DATA_LZO &&
        command != COMM_ERASE_BOOTLOADER &&
        command != COMM_ERASE_BOOTLOADER_ALL_CAN &&
        command != COMM_ERASE_BOOTLOADER_ALL_CAN_HW) {
        s_bridge_last_reject = BRIDGE_REJECT_UNSUPPORTED;
        return false;
    }

    if (!s_initialized && !f280_fw_bridge_init()) {
        s_bridge_last_reject = BRIDGE_REJECT_INIT;
        if (command == COMM_ERASE_NEW_APP) {
            send_simple_ack(reply, command, false);
        } else if (command == COMM_WRITE_NEW_APP_DATA) {
            send_write_ack(reply, command, false, 0U);
        }
        return true;
    }

    switch (command) {
    case COMM_ERASE_NEW_APP:
        if (payload_index > length || length - payload_index != 4U) {
            s_bridge_last_reject = BRIDGE_REJECT_ERASE_FORMAT;
            send_simple_ack(reply, command, false);
            return true;
        }
        if (!prepare_vehicle_safe()) {
            s_bridge_last_reject = BRIDGE_REJECT_VEHICLE_UNSAFE;
            send_simple_ack(reply, command, false);
            return true;
        }
        index = (int32_t)payload_index;
        value = buffer_get_uint32(packet, &index);
        s_upload_target = f280_fw_stream_begin(target_id, value) ?
                          target_id : 0xFFU;
        if (s_upload_target != target_id) {
            s_bridge_last_reject = BRIDGE_REJECT_F280_BEGIN;
        }
        send_simple_ack(reply, command, s_upload_target == target_id);
        return true;

    case COMM_WRITE_NEW_APP_DATA:
        if (payload_index > length || length - payload_index < 4U) {
            s_bridge_last_reject = BRIDGE_REJECT_WRITE_FORMAT;
            send_write_ack(reply, command, false, 0U);
            return true;
        }
        if (s_upload_target != target_id) {
            s_bridge_last_reject = BRIDGE_REJECT_NO_SESSION;
            send_write_ack(reply, command, false, 0U);
            return true;
        }
        index = (int32_t)payload_index;
        value = buffer_get_uint32(packet, &index);
        ok = f280_fw_stream_write(value, &packet[index],
                                  length - (unsigned int)index);
        if (!ok) {
            s_bridge_last_reject = BRIDGE_REJECT_WRITE;
        }
        send_write_ack(reply, command, ok, value);
        return true;

    case COMM_JUMP_TO_BOOTLOADER:
        if (s_upload_target != target_id) {
            s_bridge_last_reject = BRIDGE_REJECT_NO_SESSION;
            return true;
        }
        ok = f280_fw_stream_finish();
        s_upload_target = 0xFFU;
        if (!ok) {
            s_bridge_last_reject = BRIDGE_REJECT_FINISH;
        }
#ifdef HW_VCU_STM32_GATEWAY
        if (ok) {
            (void)vcu_stm32_gateway_request_run(
                    FW_RESUME_PERIOD_MS, FW_RESUME_ACCEL_RPM_S);
        }
#endif
        return true;

    case COMM_WRITE_NEW_APP_DATA_LZO:
        send_write_ack(reply, command, false, 0U);
        return true;

    case COMM_ERASE_BOOTLOADER:
    case COMM_ERASE_BOOTLOADER_ALL_CAN:
    case COMM_ERASE_BOOTLOADER_ALL_CAN_HW:
        send_simple_ack(reply, command, false);
        return true;

    default:
        return true;
    }
}
