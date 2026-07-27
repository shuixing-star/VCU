#include "vcu_stm32_gateway.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "comm_can.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "terminal.h"

#define VCU_DRIVE_DEFAULT_PERIOD_MS 5U
#define VCU_DRIVE_MIN_PERIOD_MS 5U
#define VCU_DRIVE_MAX_PERIOD_MS 100U
#define VCU_DRIVE_DEFAULT_ACCEL_RPM_S 2550U
#define VCU_DRIVE_MIN_ACCEL_RPM_S 100U
#define VCU_DRIVE_MAX_ACCEL_RPM_S 2550U

#define VCU_HEARTBEAT_FLAG_RUN_REQUESTED (1U << 0)
#define VCU_HEARTBEAT_FLAG_CRSF_LINK (1U << 1)
#define VCU_HEARTBEAT_FLAG_LEFT_ONLINE (1U << 2)
#define VCU_HEARTBEAT_FLAG_RIGHT_ONLINE (1U << 3)
#define VCU_HEARTBEAT_FLAG_OUTPUT_ENABLED (1U << 4)

#define VCU_RC_FLAG_LINK (1U << 0)
#define VCU_RC_FLAG_ARM (1U << 1)
#define VCU_RC_FLAG_RESET (1U << 2)

typedef struct {
    vcu_stm32_gateway_state_t public_state;
    uint8_t next_command_sequence;
    portMUX_TYPE lock;
} vcu_stm32_gateway_context_t;

static vcu_stm32_gateway_context_t m_ctx = {
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}


static int telemetry_index_from_id(uint32_t id)
{
    if (id < VCU_STM32_GATEWAY_ID_HEARTBEAT ||
        id > VCU_STM32_GATEWAY_ID_DIAGNOSTICS) {
        return -1;
    }
    return (int)(id - VCU_STM32_GATEWAY_ID_HEARTBEAT);
}


static bool parse_u32_arg(const char *arg, uint32_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if (arg == NULL || value == NULL || arg[0] == '\0' ||
        arg[0] == '-') {
        return false;
    }

    parsed = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }

    *value = (uint32_t)parsed;
    return true;
}

static uint32_t clamp_u32(uint32_t value,
                          uint32_t minimum,
                          uint32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool send_command(vcu_stm32_gateway_command_type_t type,
                         uint16_t period_ms,
                         uint16_t acceleration_rpm_s,
                         uint8_t *sent_sequence)
{
    uint8_t data[8] = {0};
    uint8_t sequence;
    bool initialized;

    if (type <= VCU_STM32_GATEWAY_COMMAND_NONE ||
        type > VCU_STM32_GATEWAY_COMMAND_RESET) {
        return false;
    }

    portENTER_CRITICAL(&m_ctx.lock);
    initialized = m_ctx.public_state.initialized;
    sequence = (uint8_t)(m_ctx.next_command_sequence + 1U);
    if (initialized) {
        m_ctx.next_command_sequence = sequence;
    }
    portEXIT_CRITICAL(&m_ctx.lock);

    if (!initialized) {
        return false;
    }

    const vcu_gateway_command_t command = {
        .type = type,
        .period_ms = period_ms,
        .acceleration_rpm_s = acceleration_rpm_s,
        .sequence = sequence,
    };
    if (!vcu_gateway_pack_command(&command, data)) {
        return false;
    }

    /*
     * comm_can_transmit_sid() has no result value. This counter therefore
     * records transmission attempts, not CAN-controller acknowledgements.
     */
    comm_can_transmit_sid(VCU_STM32_GATEWAY_ID_COMMAND, data, sizeof(data));

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.public_state.command_tx_attempt_count++;
    m_ctx.public_state.last_command_type = type;
    m_ctx.public_state.last_command_period_ms = period_ms;
    m_ctx.public_state.last_command_acceleration_rpm_s =
            acceleration_rpm_s;
    m_ctx.public_state.last_command_sequence = sequence;
    portEXIT_CRITICAL(&m_ctx.lock);

    if (sent_sequence != NULL) {
        *sent_sequence = sequence;
    }
    return true;
}

static const char *drive_state_name(uint8_t state)
{
    switch (state) {
    case 0U: return "STOPPED";
    case 1U: return "DISARMED";
    case 2U: return "ARMING_NEUTRAL";
    case 3U: return "ARMED";
    case 4U: return "FAULT";
    default: return "UNKNOWN";
    }
}

static const char *drive_reason_name(uint8_t reason)
{
    switch (reason) {
    case 0U: return "INIT";
    case 1U: return "STOPPED";
    case 2U: return "RESET";
    case 3U: return "CRSF_LOST";
    case 4U: return "MOTOR_TIMEOUT";
    case 5U: return "MOTOR_FAULT";
    case 6U: return "KILL";
    case 7U: return "WAITING_NEUTRAL";
    case 8U: return "NEUTRAL_DWELL";
    case 9U: return "ARMED";
    case 10U: return "RUNNING";
    case 11U: return "CONTROL_FAILURE";
    case 12U: return "CAN_TX_FAILURE";
    case 13U: return "SCHEDULE_OVERRUN";
    default: return "UNKNOWN";
    }
}

static void terminal_drive_start(int argc, const char **argv)
{
    uint32_t period_ms = VCU_DRIVE_DEFAULT_PERIOD_MS;
    uint32_t acceleration_rpm_s = VCU_DRIVE_DEFAULT_ACCEL_RPM_S;
    uint8_t sequence;

    if (argc > 3 ||
        (argc >= 2 && !parse_u32_arg(argv[1], &period_ms)) ||
        (argc >= 3 &&
         !parse_u32_arg(argv[2], &acceleration_rpm_s))) {
        commands_printf(
                "Usage: vcu_drive_start [period_ms] [accel_rpm_s]");
        return;
    }

    period_ms = clamp_u32(period_ms,
                          VCU_DRIVE_MIN_PERIOD_MS,
                          VCU_DRIVE_MAX_PERIOD_MS);
    acceleration_rpm_s = clamp_u32(
            acceleration_rpm_s,
            VCU_DRIVE_MIN_ACCEL_RPM_S,
            VCU_DRIVE_MAX_ACCEL_RPM_S);

    if (!send_command(
                VCU_STM32_GATEWAY_COMMAND_RUN,
                (uint16_t)period_ms,
                (uint16_t)acceleration_rpm_s,
                &sequence)) {
        commands_printf("STM32 VCU RUN request failed");
        return;
    }

    commands_printf(
            "STM32 VCU RUN requested: period=%lu ms accel=%lu rpm/s seq=%u",
            (unsigned long)period_ms,
            (unsigned long)acceleration_rpm_s,
            sequence);
}

static void terminal_drive_stop(int argc, const char **argv)
{
    uint8_t sequence;
    (void)argc;
    (void)argv;

    if (!send_command(VCU_STM32_GATEWAY_COMMAND_STOP,
                      0U, 0U, &sequence)) {
        commands_printf("STM32 VCU STOP request failed");
        return;
    }
    commands_printf("STM32 VCU STOP requested: seq=%u", sequence);
}

static void terminal_drive_reset(int argc, const char **argv)
{
    uint8_t sequence;
    (void)argc;
    (void)argv;

    if (!send_command(VCU_STM32_GATEWAY_COMMAND_RESET,
                      0U, 0U, &sequence)) {
        commands_printf("STM32 VCU RESET request failed");
        return;
    }
    commands_printf("STM32 VCU RESET requested: seq=%u", sequence);
}

static void terminal_drive_status(int argc, const char **argv)
{
    vcu_stm32_gateway_state_t state;
    (void)argc;
    (void)argv;

    if (!vcu_stm32_gateway_get_state(&state)) {
        commands_printf("STM32 VCU gateway status unavailable");
        return;
    }

    commands_printf(
            "STM32 VCU gateway\n"
            "online       : %d\n"
            "protocol     : %s\n"
            "heartbeat age: %s\n"
            "rx frames    : %lu/%lu/%lu/%lu/%lu/%lu\n"
            "invalid rx   : %lu\n"
            "version errs : %lu\n"
            "command tx   : %lu\n"
            "last command : type=%u seq=%u period=%u accel=%u",
            state.online,
            state.protocol_compatible ? "compatible" :
                    (state.has_heartbeat ? "mismatch" : "unknown"),
            state.has_heartbeat ? "see next line" : "never",
            (unsigned long)state.frame_rx_count[0],
            (unsigned long)state.frame_rx_count[1],
            (unsigned long)state.frame_rx_count[2],
            (unsigned long)state.frame_rx_count[3],
            (unsigned long)state.frame_rx_count[4],
            (unsigned long)state.frame_rx_count[5],
            (unsigned long)state.invalid_frame_count,
            (unsigned long)state.protocol_mismatch_count,
            (unsigned long)state.command_tx_attempt_count,
            (unsigned int)state.last_command_type,
            state.last_command_sequence,
            state.last_command_period_ms,
            state.last_command_acceleration_rpm_s);

    if (!state.has_heartbeat) {
        return;
    }

    commands_printf(
            "heartbeat    : age=%lu ms version=%u seq=%u\n"
            "drive state  : %s (%u)\n"
            "drive reason : %s (%u)\n"
            "source       : %u\n"
            "run requested: %d\n"
            "CRSF link    : %d\n"
            "motors online: left=%d right=%d\n"
            "output enable: %d\n"
            "fault flags  : 0x%04x",
            (unsigned long)state.heartbeat_age_ms,
            state.heartbeat.version,
            state.heartbeat.sequence,
            drive_state_name(state.heartbeat.state),
            state.heartbeat.state,
            drive_reason_name(state.heartbeat.reason),
            state.heartbeat.reason,
            state.heartbeat.source,
            (state.heartbeat.flags &
             VCU_HEARTBEAT_FLAG_RUN_REQUESTED) != 0U,
            (state.heartbeat.flags &
             VCU_HEARTBEAT_FLAG_CRSF_LINK) != 0U,
            (state.heartbeat.flags &
             VCU_HEARTBEAT_FLAG_LEFT_ONLINE) != 0U,
            (state.heartbeat.flags &
             VCU_HEARTBEAT_FLAG_RIGHT_ONLINE) != 0U,
            (state.heartbeat.flags &
             VCU_HEARTBEAT_FLAG_OUTPUT_ENABLED) != 0U,
            state.heartbeat.fault_flags);

    if (state.has_rc_input) {
        commands_printf(
                "RC input\n"
                "throttle/steer/arm: %u/%u/%u\n"
                "link/arm/reset     : %d/%d/%d",
                state.rc_input.throttle_raw,
                state.rc_input.steering_raw,
                state.rc_input.arm_raw,
                (state.rc_input.flags & VCU_RC_FLAG_LINK) != 0U,
                (state.rc_input.flags & VCU_RC_FLAG_ARM) != 0U,
                (state.rc_input.flags & VCU_RC_FLAG_RESET) != 0U);
    }

    if (state.has_targets) {
        commands_printf(
                "targets rpm\n"
                "base left/right : %d/%d\n"
                "final left/right: %d/%d",
                state.targets.left_base_rpm,
                state.targets.right_base_rpm,
                state.targets.left_target_rpm,
                state.targets.right_target_rpm);
    }

    if (state.has_motor_left && state.has_motor_right) {
        commands_printf(
                "motors\n"
                "left : rpm=%d voltage=%u.%u temp=%u fault=%u age=%u0ms\n"
                "right: rpm=%d voltage=%u.%u temp=%u fault=%u age=%u0ms",
                state.motor_left.speed_rpm,
                state.motor_left.voltage_decivolts / 10U,
                state.motor_left.voltage_decivolts % 10U,
                state.motor_left.temperature_c,
                state.motor_left.fault_code,
                state.motor_left.age_10ms,
                state.motor_right.speed_rpm,
                state.motor_right.voltage_decivolts / 10U,
                state.motor_right.voltage_decivolts % 10U,
                state.motor_right.temperature_c,
                state.motor_right.fault_code,
                state.motor_right.age_10ms);
    }

    if (state.has_diagnostics) {
        commands_printf(
                "diagnostics\n"
                "CRSF valid/crc : %u/%u\n"
                "CAN overflows  : %u\n"
                "flags          : 0x%02x",
                state.diagnostics.crsf_valid_frames,
                state.diagnostics.crsf_crc_errors,
                state.diagnostics.can_rx_overflows,
                state.diagnostics.flags);
    }
}

bool vcu_stm32_gateway_process_can_frame(uint32_t id,
                                         const uint8_t *data,
                                         uint8_t len,
                                         bool is_ext)
{
    const int index = telemetry_index_from_id(id);
    const uint32_t timestamp_ms = now_ms();
    bool initialized;

    if (index < 0) {
        return false;
    }

    portENTER_CRITICAL(&m_ctx.lock);
    initialized = m_ctx.public_state.initialized;
    portEXIT_CRITICAL(&m_ctx.lock);
    if (!initialized) {
        return false;
    }

    if (is_ext || data == NULL || len != 8U) {
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.invalid_frame_count++;
        portEXIT_CRITICAL(&m_ctx.lock);
        return false;
    }

    switch ((vcu_stm32_gateway_telemetry_index_t)index) {
    case VCU_STM32_GATEWAY_TELEMETRY_HEARTBEAT:
    {
        vcu_stm32_gateway_heartbeat_t message;
        vcu_gateway_unpack_heartbeat(data, &message);
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.heartbeat = message;
        m_ctx.public_state.has_heartbeat = true;
        m_ctx.public_state.protocol_compatible =
                message.version ==
                VCU_STM32_GATEWAY_PROTOCOL_VERSION;
        if (!m_ctx.public_state.protocol_compatible) {
            m_ctx.public_state.protocol_mismatch_count++;
        }
        portEXIT_CRITICAL(&m_ctx.lock);
        break;
    }
    case VCU_STM32_GATEWAY_TELEMETRY_RC_INPUT:
    {
        vcu_stm32_gateway_rc_input_t message;
        vcu_gateway_unpack_rc_input(data, &message);
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.rc_input = message;
        m_ctx.public_state.has_rc_input = true;
        portEXIT_CRITICAL(&m_ctx.lock);
        break;
    }
    case VCU_STM32_GATEWAY_TELEMETRY_TARGETS:
    {
        vcu_stm32_gateway_targets_t message;
        vcu_gateway_unpack_targets(data, &message);
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.targets = message;
        m_ctx.public_state.has_targets = true;
        portEXIT_CRITICAL(&m_ctx.lock);
        break;
    }
    case VCU_STM32_GATEWAY_TELEMETRY_MOTOR_LEFT:
    {
        vcu_stm32_gateway_motor_status_t message;
        vcu_gateway_unpack_motor_status(data, &message);
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.motor_left = message;
        m_ctx.public_state.has_motor_left = true;
        portEXIT_CRITICAL(&m_ctx.lock);
        break;
    }
    case VCU_STM32_GATEWAY_TELEMETRY_MOTOR_RIGHT:
    {
        vcu_stm32_gateway_motor_status_t message;
        vcu_gateway_unpack_motor_status(data, &message);
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.motor_right = message;
        m_ctx.public_state.has_motor_right = true;
        portEXIT_CRITICAL(&m_ctx.lock);
        break;
    }
    case VCU_STM32_GATEWAY_TELEMETRY_DIAGNOSTICS:
    {
        vcu_stm32_gateway_diagnostics_t message;
        vcu_gateway_unpack_diagnostics(data, &message);
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.diagnostics = message;
        m_ctx.public_state.has_diagnostics = true;
        portEXIT_CRITICAL(&m_ctx.lock);
        break;
    }
    default:
        return false;
    }

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.public_state.last_any_rx_ms = timestamp_ms;
    m_ctx.public_state.frame_rx_count[index]++;
    m_ctx.public_state.frame_last_rx_ms[index] = timestamp_ms;
    portEXIT_CRITICAL(&m_ctx.lock);
    return true;
}

bool vcu_stm32_gateway_get_state(vcu_stm32_gateway_state_t *state)
{
    uint32_t timestamp_ms;

    if (state == NULL) {
        return false;
    }

    timestamp_ms = now_ms();
    portENTER_CRITICAL(&m_ctx.lock);
    *state = m_ctx.public_state;
    portEXIT_CRITICAL(&m_ctx.lock);

    if (!state->initialized) {
        return false;
    }

    if (state->has_heartbeat) {
        state->heartbeat_age_ms =
                (uint32_t)(timestamp_ms -
                    state->frame_last_rx_ms[
                        VCU_STM32_GATEWAY_TELEMETRY_HEARTBEAT]);
        state->online = state->protocol_compatible &&
                state->heartbeat_age_ms <=
                VCU_STM32_GATEWAY_ONLINE_TIMEOUT_MS;
    } else {
        state->heartbeat_age_ms = UINT32_MAX;
        state->online = false;
    }

    return true;
}

void vcu_stm32_gateway_init(void)
{
    bool already_initialized;

    portENTER_CRITICAL(&m_ctx.lock);
    already_initialized = m_ctx.public_state.initialized;
    if (!already_initialized) {
        memset(&m_ctx.public_state, 0,
               sizeof(m_ctx.public_state));
        m_ctx.next_command_sequence = 0U;
        m_ctx.public_state.heartbeat_age_ms = UINT32_MAX;
        m_ctx.public_state.initialized = true;
    }
    portEXIT_CRITICAL(&m_ctx.lock);

    if (already_initialized) {
        return;
    }

    terminal_register_command_callback(
            "vcu_drive_start",
            "Request guarded STM32 RC drive.",
            "[period_ms] [accel_rpm_s]",
            terminal_drive_start);
    terminal_register_command_callback(
            "vcu_drive_stop",
            "Request STM32 drive stop.",
            0,
            terminal_drive_stop);
    terminal_register_command_callback(
            "vcu_drive_reset",
            "Request STM32 motor-controller reset.",
            0,
            terminal_drive_reset);
    terminal_register_command_callback(
            "vcu_drive_status",
            "Print cached STM32 VCU status.",
            0,
            terminal_drive_status);
}
