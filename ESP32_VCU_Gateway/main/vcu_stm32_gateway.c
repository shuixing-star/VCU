#include "vcu_stm32_gateway.h"
#include "vcu_fault_log.h"
#include "vcu_parking_sample.h"

#include <limits.h>
#include <string.h>
#include <stdlib.h>

#include "comm_can.h"
#include "commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "terminal.h"

#define VCU_HEARTBEAT_FLAG_RUN_REQUESTED (1U << 0)
#define VCU_HEARTBEAT_FLAG_CRSF_LINK (1U << 1)
#define VCU_HEARTBEAT_FLAG_LEFT_ONLINE (1U << 2)
#define VCU_HEARTBEAT_FLAG_RIGHT_ONLINE (1U << 3)
#define VCU_HEARTBEAT_FLAG_OUTPUT_ENABLED (1U << 4)

#define VCU_RC_FLAG_LINK (1U << 0)
#define VCU_RC_FLAG_ARM (1U << 1)
#define VCU_RC_FLAG_RESET (1U << 2)

#define VCU_MOTOR_RAW_FRAME_COUNT 6U
#define VCU_MOTOR_RAW_DATA_SIZE 8U
#define VCU_CAN_ERROR_LOG_SIZE 64U

typedef struct {
    bool valid;
    uint8_t len;
    uint32_t count;
    uint32_t last_rx_ms;
    uint8_t data[VCU_MOTOR_RAW_DATA_SIZE];
} vcu_motor_raw_frame_t;

typedef struct {
    vcu_gateway_can_error_t event;
    uint32_t received_ms;
    uint8_t probe_mask;
    uint32_t probe[VCU_GATEWAY_TX_PROBE_PARTS];
} vcu_can_error_record_t;

static const uint32_t s_motor_raw_ids[VCU_MOTOR_RAW_FRAME_COUNT] = {
    0x0C01EFD0UL, 0x1801D0EFUL, 0x1802D0EFUL,
    0x0C01F0D0UL, 0x1801D0F0UL, 0x1802D0F0UL
};

static const char *s_motor_raw_names[VCU_MOTOR_RAW_FRAME_COUNT] = {
    "left command", "left electrical", "left state",
    "right command", "right electrical", "right state"
};

typedef struct {
    vcu_stm32_gateway_state_t public_state;
    vcu_parking_observer_t parking[2];
    vcu_k2_diag_assembler_t k2_diag[2];
    vcu_motor_raw_frame_t motor_raw[VCU_MOTOR_RAW_FRAME_COUNT];
    vcu_can_error_record_t can_error_log[VCU_CAN_ERROR_LOG_SIZE];
    uint8_t can_error_next;
    uint8_t last_can_error_sequence;
    bool has_can_error_sequence;
    portMUX_TYPE lock;
} vcu_stm32_gateway_context_t;

static vcu_stm32_gateway_context_t m_ctx = {
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static uint8_t s_command_sequence;
static uint32_t s_command_tx_attempts;
static uint32_t s_command_tx_successes;
static uint32_t s_command_tx_failures;
static uint8_t s_last_command_type;
static uint8_t s_last_command_sequence;
static bool s_last_command_tx_ok;
static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static const char *can_error_type_name(uint8_t type);
static const char *can_lec_name(uint8_t flags);


static int telemetry_index_from_id(uint32_t id)
{
    if (id < VCU_STM32_GATEWAY_ID_HEARTBEAT ||
        id > VCU_STM32_GATEWAY_ID_DIAGNOSTICS) {
        return -1;
    }
    return (int)(id - VCU_STM32_GATEWAY_ID_HEARTBEAT);
}

static int motor_raw_index_from_id(uint32_t id)
{
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_RAW_FRAME_COUNT; index++) {
        if (s_motor_raw_ids[index] == id) {
            return (int)index;
        }
    }
    return -1;
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
            "CAN errors   : %lu stored=%u gaps=%lu\n"
            "command TX   : %lu/%lu/%lu\n"
            "last command : type=%u seq=%u tx=%s",
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
            (unsigned long)state.can_error_event_count,
            state.can_error_stored_count,
            (unsigned long)state.can_error_sequence_gap_count,
            (unsigned long)s_command_tx_attempts,
            (unsigned long)s_command_tx_successes,
            (unsigned long)s_command_tx_failures,
            s_last_command_type,
            s_last_command_sequence,
            s_command_tx_attempts == 0U ? "never" :
                    (s_last_command_tx_ok ? "ok" : "failed"));

    if (state.has_can_error) {
        commands_printf(
                "last CAN err : #%u %s lec=%s flags=0x%02x "
                "TEC=%u REC=%u hal=%u age=%lums",
                state.last_can_error.sequence,
                can_error_type_name(state.last_can_error.type),
                can_lec_name(state.last_can_error.flags),
                state.last_can_error.flags,
                state.last_can_error.tx_error_count,
                state.last_can_error.rx_error_count,
                state.last_can_error.hal_state,
                (unsigned long)(now_ms() - state.last_can_error_rx_ms));
    }

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

static void terminal_vcu_can_raw(int argc, const char **argv)
{
    static const char hex[] = "0123456789abcdef";
    vcu_motor_raw_frame_t raw[VCU_MOTOR_RAW_FRAME_COUNT];
    const uint32_t timestamp_ms = now_ms();
    unsigned int frame_index;
    unsigned int byte_index;
    (void)argc;
    (void)argv;

    portENTER_CRITICAL(&m_ctx.lock);
    memcpy(raw, m_ctx.motor_raw, sizeof(raw));
    portEXIT_CRITICAL(&m_ctx.lock);

    commands_printf("VCU <-> motor CAN raw frames");
    for (frame_index = 0U;
         frame_index < VCU_MOTOR_RAW_FRAME_COUNT;
         frame_index++) {
        char data_text[VCU_MOTOR_RAW_DATA_SIZE * 3U];
        unsigned int text_index = 0U;
        if (!raw[frame_index].valid) {
            commands_printf("%s id=0x%08lx never",
                            s_motor_raw_names[frame_index],
                            (unsigned long)s_motor_raw_ids[frame_index]);
            continue;
        }
        commands_printf("%s id=0x%08lx count=%lu age=%lu ms len=%u",
                        s_motor_raw_names[frame_index],
                        (unsigned long)s_motor_raw_ids[frame_index],
                        (unsigned long)raw[frame_index].count,
                        (unsigned long)(timestamp_ms -
                                        raw[frame_index].last_rx_ms),
                        raw[frame_index].len);
        for (byte_index = 0U; byte_index < raw[frame_index].len;
             byte_index++) {
            const uint8_t value = raw[frame_index].data[byte_index];
            data_text[text_index++] = hex[value >> 4U];
            data_text[text_index++] = hex[value & 0x0FU];
            if (byte_index + 1U < raw[frame_index].len) {
                data_text[text_index++] = ' ';
            }
        }
        data_text[text_index] = '\0';
        commands_printf("data: %s", data_text);
    }
}

/* A missing fragment remains visibly missing. Match seq AND STM32 timestamp
 * so an 8-bit sequence wrap cannot attach a snapshot to an older event. */
static bool process_tx_probe(const uint8_t data[8])
{
    uint8_t sequence, part, i;
    uint16_t timestamp;
    uint32_t value;
    bool found = false;
    if (!vcu_gateway_unpack_tx_probe(data, &sequence, &timestamp, &part, &value)) {
        return false;
    }
    portENTER_CRITICAL(&m_ctx.lock);
    for (i = 0U; i < m_ctx.public_state.can_error_stored_count; ++i) {
        const unsigned int index = (m_ctx.can_error_next + VCU_CAN_ERROR_LOG_SIZE -
                                     1U - i) % VCU_CAN_ERROR_LOG_SIZE;
        vcu_can_error_record_t *record = &m_ctx.can_error_log[index];
        if (record->event.sequence == sequence &&
            record->event.timestamp_10ms == timestamp) {
            record->probe[part] = value;
            record->probe_mask |= (uint8_t)(1U << part);
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&m_ctx.lock);
    return found;
}

bool vcu_stm32_gateway_process_can_frame(uint32_t id,
                                         const uint8_t *data,
                                         uint8_t len,
                                         bool is_ext)
{
    int index;
    const uint32_t timestamp_ms = now_ms();
    if (vcu_fault_log_receive(id,data,len,is_ext,timestamp_ms)) return true;
    const int raw_index = is_ext ? motor_raw_index_from_id(id) : -1;
    bool initialized;
    unsigned diag_side, diag_part;

    if (vcu_k2_diag_route(id, is_ext, &diag_side, &diag_part)) {
        portENTER_CRITICAL(&m_ctx.lock);
        if (m_ctx.public_state.initialized)
            (void)vcu_k2_diag_accept(&m_ctx.k2_diag[diag_side],
                    &m_ctx.public_state.k2_diag[diag_side],
                    diag_part, data, len, timestamp_ms);
        portEXIT_CRITICAL(&m_ctx.lock);
        return true;
    }

    if (raw_index >= 0) {
        if (data != NULL && len <= VCU_MOTOR_RAW_DATA_SIZE) {
            portENTER_CRITICAL(&m_ctx.lock);
            if (m_ctx.public_state.initialized) {
                vcu_parking_observe(&m_ctx.parking[raw_index / 3],
                        (unsigned)(raw_index % 3), data, len, timestamp_ms);
                vcu_motor_raw_frame_t *raw = &m_ctx.motor_raw[raw_index];
                raw->valid = true;
                raw->len = len;
                raw->count++;
                raw->last_rx_ms = timestamp_ms;
                memset(raw->data, 0, sizeof(raw->data));
                memcpy(raw->data, data, len);
            }
            portEXIT_CRITICAL(&m_ctx.lock);
        }
        return false;
    }

    portENTER_CRITICAL(&m_ctx.lock);
    initialized = m_ctx.public_state.initialized;
    portEXIT_CRITICAL(&m_ctx.lock);
    if (!initialized) {
        return false;
    }

    /* Other protocols are not malformed VCU telemetry. */
    if (is_ext || (telemetry_index_from_id(id) < 0 &&
                   id != VCU_GATEWAY_ID_CAN_ERROR &&
                   id != VCU_GATEWAY_ID_CAN_TX_PROBE &&
                   id != VCU_BRAKE_DIAG_ID)) {
        return false;
    }
    if (data == NULL || len != 8U) {
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.invalid_frame_count++;
        portEXIT_CRITICAL(&m_ctx.lock);
        return false;
    }

    if (id == VCU_BRAKE_DIAG_ID) {
        vcu_brake_diag_t message;
        if (!vcu_brake_diag_unpack(data, &message)) {
            portENTER_CRITICAL(&m_ctx.lock);
            m_ctx.public_state.invalid_frame_count++;
            portEXIT_CRITICAL(&m_ctx.lock);
            return false;
        }
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public_state.brake_diag = message;
        m_ctx.public_state.brake_diag_rx_ms = timestamp_ms;
        m_ctx.public_state.has_brake_diag = true;
        m_ctx.public_state.last_any_rx_ms = timestamp_ms;
        portEXIT_CRITICAL(&m_ctx.lock);
        return true;
    }

    if (id == VCU_GATEWAY_ID_CAN_TX_PROBE) {
        return process_tx_probe(data);
    }
    if (id == VCU_STM32_GATEWAY_ID_CAN_ERROR) {
        vcu_gateway_can_error_t event;
        uint8_t delta;

        vcu_gateway_unpack_can_error(data, &event);
        if (event.type < VCU_GATEWAY_CAN_ERROR_HAL_IRQ ||
            event.type > VCU_GATEWAY_CAN_ERROR_HARD_RECOVERY_FAILED) {
            portENTER_CRITICAL(&m_ctx.lock);
            m_ctx.public_state.invalid_frame_count++;
            portEXIT_CRITICAL(&m_ctx.lock);
            return false;
        }

        portENTER_CRITICAL(&m_ctx.lock);
        if (m_ctx.has_can_error_sequence) {
            delta = (uint8_t)(event.sequence - m_ctx.last_can_error_sequence);
            if (delta > 1U) {
                m_ctx.public_state.can_error_sequence_gap_count +=
                        (uint32_t)(delta - 1U);
            }
        }
        m_ctx.has_can_error_sequence = true;
        m_ctx.last_can_error_sequence = event.sequence;
        memset(&m_ctx.can_error_log[m_ctx.can_error_next], 0,
               sizeof(m_ctx.can_error_log[m_ctx.can_error_next]));
        m_ctx.can_error_log[m_ctx.can_error_next].event = event;
        m_ctx.can_error_log[m_ctx.can_error_next].received_ms = timestamp_ms;
        m_ctx.can_error_next = (uint8_t)((m_ctx.can_error_next + 1U) %
                                         VCU_CAN_ERROR_LOG_SIZE);
        if (m_ctx.public_state.can_error_stored_count < VCU_CAN_ERROR_LOG_SIZE) {
            m_ctx.public_state.can_error_stored_count++;
        }
        m_ctx.public_state.can_error_event_count++;
        m_ctx.public_state.has_can_error = true;
        m_ctx.public_state.last_can_error = event;
        m_ctx.public_state.last_can_error_rx_ms = timestamp_ms;
        m_ctx.public_state.last_any_rx_ms = timestamp_ms;
        portEXIT_CRITICAL(&m_ctx.lock);
        return true;
    }

    index = telemetry_index_from_id(id);
    if (index < 0) {
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

bool vcu_stm32_gateway_get_parking_snapshot(vcu_stm32_gateway_state_t *state,
        vcu_parking_observer_t observers[2])
{
    uint32_t timestamp_ms;

    if (state == NULL) {
        return false;
    }

    timestamp_ms = now_ms();
    portENTER_CRITICAL(&m_ctx.lock);
    *state = m_ctx.public_state;
    if (observers) memcpy(observers, m_ctx.parking, sizeof(m_ctx.parking));
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

bool vcu_stm32_gateway_get_state(vcu_stm32_gateway_state_t *state)
{
    return vcu_stm32_gateway_get_parking_snapshot(state, NULL);
}

static void terminal_request_stop(int argc, const char **argv)
{
    bool ok;
    (void)argc;
    (void)argv;

    ok = vcu_stm32_gateway_request_stop();
    commands_printf("STM32 STOP queued: %s; run vcu_drive_status after 100 ms",
                    ok ? "yes" : "no");
}

static const char *can_error_type_name(uint8_t type)
{
    switch (type) {
    case VCU_GATEWAY_CAN_ERROR_HAL_IRQ: return "HAL_IRQ";
    case VCU_GATEWAY_CAN_ERROR_MAILBOX_TIMEOUT: return "MAILBOX_TIMEOUT";
    case VCU_GATEWAY_CAN_ERROR_TX_TIMEOUT: return "TX_TIMEOUT";
    case VCU_GATEWAY_CAN_ERROR_TX_ADD_FAILED: return "TX_ADD_FAILED";
    case VCU_GATEWAY_CAN_ERROR_SOFT_RECOVERY_FAILED: return "SOFT_RECOVERY_FAILED";
    case VCU_GATEWAY_CAN_ERROR_HARD_RECOVERY_FAILED: return "HARD_RECOVERY_FAILED";
    default: return "UNKNOWN";
    }
}

static const char *can_lec_name(uint8_t flags)
{
    switch (flags & VCU_GATEWAY_CAN_ERROR_LEC_MASK) {
    case 0U: return "NONE";
    case 1U: return "STUFF";
    case 2U: return "FORM";
    case 3U: return "ACK";
    case 4U: return "BIT_RECESSIVE";
    case 5U: return "BIT_DOMINANT";
    case 6U: return "CRC";
    default: return "SOFTWARE";
    }
}

static void print_tx_probe(const vcu_can_error_record_t *record)
{
    uint32_t tsr, esr, meta, expected, free_mask, ok, alst, terr, pending;
    const char *source;
    const char *trigger;
    const char *observation;
    if (record->probe_mask != VCU_GATEWAY_TX_PROBE_COMPLETE) {
        if (record->event.type != VCU_GATEWAY_CAN_ERROR_HAL_IRQ) {
            commands_printf("  tx_probe=%s parts=0x%02x/0x7f (pending/lost or older STM32)",
                    record->probe_mask ? "PARTIAL" : "MISSING", record->probe_mask);
        }
        return;
    }
    tsr = record->probe[0];
    esr = record->probe[1];
    meta = record->probe[6];
    source = (meta & 3U) == 1U ? "CONTROL_PAIR" : "BACKGROUND";
    switch (meta >> 24U) {
    case 1U: trigger = "DEADLINE"; break;
    case 2U: trigger = "NO_TXOK"; break;
    case 3U: trigger = "MAILBOX_WAIT"; break;
    default: trigger = "ADD_FAILED"; break;
    }
    expected = ((meta >> 8U) | (meta >> 16U)) & 7U;
    free_mask = (tsr >> 26U) & 7U;
    pending = expected & ~free_mask;
    ok = ((tsr >> 1U) & 1U) | ((tsr >> 8U) & 2U) | ((tsr >> 15U) & 4U);
    alst = ((tsr >> 2U) & 1U) | ((tsr >> 9U) & 2U) | ((tsr >> 16U) & 4U);
    terr = ((tsr >> 3U) & 1U) | ((tsr >> 10U) & 2U) | ((tsr >> 17U) & 4U);
    if (!expected) observation = "NO_MAILBOX_ASSIGNED";
    else if (!pending && (ok & expected) == expected) observation = "COMPLETE_AT_SNAPSHOT";
    else if (pending && (alst & expected)) observation = "PENDING_ARBITRATION_SEEN";
    else if (pending) observation = "PENDING";
    else observation = "NOT_PENDING_NO_TXOK";
    commands_printf("  tx_probe=v1 source=%s trigger=%s elapsed_us=%lu max_check_gap_us=%lu task_switches=%u",
            source, trigger, (unsigned long)record->probe[4],
            (unsigned long)record->probe[5], (unsigned int)((meta >> 2U) & 63U));
    commands_printf("  id1=0x%08lx mb1=0x%x id2=0x%08lx mb2=0x%x",
            (unsigned long)record->probe[2], (unsigned int)((meta >> 8U) & 7U),
            (unsigned long)record->probe[3], (unsigned int)((meta >> 16U) & 7U));
    commands_printf("  TSR=0x%08lx ESR=0x%08lx pending=0x%lx txok=0x%lx alst=0x%lx terr=0x%lx",
            (unsigned long)tsr, (unsigned long)esr, (unsigned long)pending,
            (unsigned long)ok, (unsigned long)alst, (unsigned long)terr);
    commands_printf("  observation=%s (snapshot before abort; gap is not proof of RTOS delay)",
            observation);
}

static void terminal_can_errors(int argc, const char **argv)
{
    vcu_can_error_record_t *snapshot;
    uint32_t total;
    uint32_t gaps;
    uint8_t count;
    uint8_t start;
    uint8_t i;
    const uint32_t timestamp_ms = now_ms();

    /* terminal_process_string includes the command name in argv[0]. */
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        portENTER_CRITICAL(&m_ctx.lock);
        memset(m_ctx.can_error_log, 0, sizeof(m_ctx.can_error_log));
        m_ctx.can_error_next = 0U;
        m_ctx.has_can_error_sequence = false;
        m_ctx.public_state.can_error_event_count = 0U;
        m_ctx.public_state.can_error_sequence_gap_count = 0U;
        m_ctx.public_state.can_error_stored_count = 0U;
        m_ctx.public_state.has_can_error = false;
        portEXIT_CRITICAL(&m_ctx.lock);
        commands_printf("STM32 CAN error log cleared");
        return;
    }
    if (argc != 1) {
        commands_printf("Usage: vcu_can_errors [clear]");
        return;
    }

    snapshot = malloc(sizeof(m_ctx.can_error_log));
    if (snapshot == NULL) {
        commands_printf("CAN log snapshot allocation failed");
        return;
    }
    portENTER_CRITICAL(&m_ctx.lock);
    total = m_ctx.public_state.can_error_event_count;
    gaps = m_ctx.public_state.can_error_sequence_gap_count;
    count = m_ctx.public_state.can_error_stored_count;
    start = (uint8_t)((m_ctx.can_error_next + VCU_CAN_ERROR_LOG_SIZE - count) %
                      VCU_CAN_ERROR_LOG_SIZE);
    for (i = 0U; i < count; ++i) {
        snapshot[i] = m_ctx.can_error_log[(start + i) % VCU_CAN_ERROR_LOG_SIZE];
    }
    portEXIT_CRITICAL(&m_ctx.lock);

    commands_printf("STM32 CAN error log: total=%lu stored=%u sequence_gaps=%lu",
                    (unsigned long)total, count, (unsigned long)gaps);
    commands_printf("TX probe decoder=v1 ESP uptime_ms=%lu", (unsigned long)timestamp_ms);
    for (i = 0U; i < count; ++i) {
        const vcu_gateway_can_error_t *event = &snapshot[i].event;
        commands_printf(
                "#%u type=%s(%u) lec=%s(%u) flags=0x%02x "
                "TEC=%u REC=%u hal=%u stm32=%u0ms rx_age=%lums",
                event->sequence,
                can_error_type_name(event->type), event->type,
                can_lec_name(event->flags),
                event->flags & VCU_GATEWAY_CAN_ERROR_LEC_MASK,
                event->flags, event->tx_error_count, event->rx_error_count,
                event->hal_state, event->timestamp_10ms,
                (unsigned long)(timestamp_ms - snapshot[i].received_ms));
        print_tx_probe(&snapshot[i]);
    }
    free(snapshot);
}

static void terminal_request_reset(int argc, const char **argv)
{
    bool ok;
    (void)argc;
    (void)argv;

    ok = vcu_stm32_gateway_request_reset();
    commands_printf("STM32 RESET queued: %s; run vcu_drive_status after 100 ms",
                    ok ? "yes" : "no");
}

static bool send_drive_command(vcu_gateway_command_type_t type,
                               uint16_t period_ms,
                               uint16_t acceleration_rpm_s)
{
    vcu_gateway_command_t command;
    uint8_t data[8];

    command.type = type;
    command.period_ms = period_ms;
    command.acceleration_rpm_s = acceleration_rpm_s;
    command.sequence = ++s_command_sequence;
    if (!vcu_gateway_pack_command(&command, data)) {
        return false;
    }
    s_command_tx_attempts++;
    s_last_command_type = (uint8_t)type;
    s_last_command_sequence = command.sequence;
    s_last_command_tx_ok = comm_can_transmit_sid_checked(
            VCU_GATEWAY_ID_COMMAND, data, sizeof(data));
    if (s_last_command_tx_ok) {
        s_command_tx_successes++;
    } else {
        s_command_tx_failures++;
    }
    return s_last_command_tx_ok;
}

bool vcu_stm32_gateway_request_stop(void)
{
    return send_drive_command(VCU_GATEWAY_COMMAND_STOP, 0U, 0U);
}
bool vcu_stm32_gateway_request_reset(void)
{
    return send_drive_command(VCU_GATEWAY_COMMAND_RESET, 0U, 0U);
}


bool vcu_stm32_gateway_request_run(uint16_t period_ms,
                                    uint16_t acceleration_rpm_s)
{
    return send_drive_command(VCU_GATEWAY_COMMAND_RUN,
                              period_ms, acceleration_rpm_s);
}

void vcu_stm32_gateway_init(void)
{
    bool already_initialized;

    portENTER_CRITICAL(&m_ctx.lock);
    already_initialized = m_ctx.public_state.initialized;
    if (!already_initialized) {
        memset(&m_ctx.public_state, 0,
               sizeof(m_ctx.public_state));
        memset(m_ctx.can_error_log, 0, sizeof(m_ctx.can_error_log));
        m_ctx.can_error_next = 0U;
        m_ctx.last_can_error_sequence = 0U;
        m_ctx.has_can_error_sequence = false;
        m_ctx.public_state.heartbeat_age_ms = UINT32_MAX;
        m_ctx.public_state.initialized = true;
        s_command_sequence = 0U;
        s_command_tx_attempts = 0U;
        memset(m_ctx.motor_raw, 0, sizeof(m_ctx.motor_raw));
        memset(m_ctx.k2_diag, 0, sizeof(m_ctx.k2_diag));
        s_command_tx_successes = 0U;
        s_command_tx_failures = 0U;
        s_last_command_type = 0U;
        s_last_command_sequence = 0U;
        s_last_command_tx_ok = false;
    }
    portEXIT_CRITICAL(&m_ctx.lock);

    if (already_initialized) {
        return;
    }

    terminal_register_command_callback(
            "vcu_drive_status",
            "Print cached STM32 VCU status.",
            0,
            terminal_drive_status);
    terminal_register_command_callback(
            "vcu_can_errors",
            "Print STM32 CAN error events; add 'clear' to clear.",
            "[clear]",
            terminal_can_errors);
    terminal_register_command_callback(
            "vcu_stm32_stop",
            "Send one guarded STOP command to STM32 VCU.",
            0,
            terminal_request_stop);
    terminal_register_command_callback(
            "vcu_stm32_reset",
            "Request a guarded motor-controller reset through STM32 VCU.",
            0,
            terminal_request_reset);
    terminal_register_command_callback(
            "vcu_can_raw",
            "Print last raw VCU/motor CAN frames.",
            0,
            terminal_vcu_can_raw);
}
