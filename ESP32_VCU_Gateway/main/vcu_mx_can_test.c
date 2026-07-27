#include "vcu_mx_can_test.h"

#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "comm_can.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "terminal.h"

#define VCU_MX_CAN_STATUS_TIMEOUT_MS 500U
#define VCU_RPM_FILTER_SHIFT 2
#define VCU_RPM_FILTER_ALPHA_DESCRIPTION "0.25"

typedef struct {
    bool initialized;
    uint32_t ignored_frame_count;
    vcu_mx_can_controller_state_t controller[VCU_MX_CAN_CONTROLLER_COUNT];
    bool last_tx_valid[VCU_MX_CAN_CONTROLLER_COUNT];
    uint32_t last_tx_count[VCU_MX_CAN_CONTROLLER_COUNT];
    uint32_t last_tx_id[VCU_MX_CAN_CONTROLLER_COUNT];
    uint8_t last_tx_data[VCU_MX_CAN_CONTROLLER_COUNT][MX_CAN_FRAME_SIZE];
    bool last_rx_valid[VCU_MX_CAN_CONTROLLER_COUNT];
    uint32_t last_rx_id[VCU_MX_CAN_CONTROLLER_COUNT];
    uint8_t last_rx_data[VCU_MX_CAN_CONTROLLER_COUNT][MX_CAN_FRAME_SIZE];
    bool rpm_filter_initialized[VCU_MX_CAN_CONTROLLER_COUNT];
    int16_t rpm_history_1[VCU_MX_CAN_CONTROLLER_COUNT];
    int16_t rpm_history_2[VCU_MX_CAN_CONTROLLER_COUNT];
    int32_t filtered_rpm[VCU_MX_CAN_CONTROLLER_COUNT];
    portMUX_TYPE lock;
} vcu_mx_can_test_context_t;

static vcu_mx_can_test_context_t m_ctx = {
    .controller = {
        {
            .configured = true,
            .status_id = MX_CAN_STATUS_ID + 1U,
            .command_id = MX_CAN_COMMAND_ID + 1U,
            .direction_sign = -1,
        },
        {
            .configured = true,
            .status_id = MX_CAN_STATUS_ID,
            .command_id = MX_CAN_COMMAND_ID,
            .direction_sign = 1,
        },
    },
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint32_t parse_can_id_arg(const char *arg) {
    return (uint32_t)strtoul(arg, NULL, 16);
}

static const char *controller_name(unsigned int index) {
    return index == VCU_MX_CAN_LEFT ? "left" : "right";
}

static int16_t apply_direction_sign(int16_t value, int8_t direction_sign) {
    if (direction_sign >= 0) return value;
    if (value == INT16_MIN) return INT16_MAX;
    return (int16_t)-value;
}

static int16_t normalize_vehicle_rpm(int16_t controller_rpm, int8_t direction_sign) {
    const int16_t vehicle_rpm = apply_direction_sign(controller_rpm, direction_sign);
    if (vehicle_rpm > -VCU_VEHICLE_RPM_DEADZONE && vehicle_rpm < VCU_VEHICLE_RPM_DEADZONE) return 0;
    return vehicle_rpm;
}

static int16_t median3(int16_t a, int16_t b, int16_t c) {
    int16_t temp;
    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }
    if (b > c) {
        temp = b;
        b = c;
        c = temp;
    }
    if (a > b) {
        temp = a;
        a = b;
        b = temp;
    }
    return b;
}

static int16_t filter_vehicle_rpm(unsigned int index, int16_t vehicle_rpm) {
    if (!m_ctx.rpm_filter_initialized[index]) {
        m_ctx.rpm_history_1[index] = vehicle_rpm;
        m_ctx.rpm_history_2[index] = vehicle_rpm;
        m_ctx.filtered_rpm[index] = vehicle_rpm;
        m_ctx.rpm_filter_initialized[index] = true;
        return vehicle_rpm;
    }

    const int16_t median = median3(m_ctx.rpm_history_2[index], m_ctx.rpm_history_1[index], vehicle_rpm);
    m_ctx.rpm_history_2[index] = m_ctx.rpm_history_1[index];
    m_ctx.rpm_history_1[index] = vehicle_rpm;
    m_ctx.filtered_rpm[index] += ((int32_t)median - m_ctx.filtered_rpm[index]) >> VCU_RPM_FILTER_SHIFT;
    if (vehicle_rpm == 0 && abs(m_ctx.filtered_rpm[index]) < VCU_VEHICLE_RPM_DEADZONE) {
        m_ctx.filtered_rpm[index] = 0;
    }
    return (int16_t)m_ctx.filtered_rpm[index];
}

static int16_t controller_target(unsigned int index, int16_t vehicle_target) {
    return apply_direction_sign(vehicle_target, m_ctx.controller[index].direction_sign);
}

static bool controller_is_configured(const vcu_mx_can_controller_state_t *controller) {
    return controller->configured && (controller->status_id != 0U || controller->command_id != 0U);
}

static void store_last_tx(unsigned int index, uint32_t id, const uint8_t data[MX_CAN_FRAME_SIZE]) {
    if (index >= VCU_MX_CAN_CONTROLLER_COUNT || data == NULL) return;

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.last_tx_valid[index] = true;
    m_ctx.last_tx_count[index]++;
    m_ctx.last_tx_id[index] = id;
    memcpy(m_ctx.last_tx_data[index], data, MX_CAN_FRAME_SIZE);
    portEXIT_CRITICAL(&m_ctx.lock);
}

static void store_last_rx(unsigned int index, uint32_t id, const uint8_t data[MX_CAN_FRAME_SIZE]) {
    if (index >= VCU_MX_CAN_CONTROLLER_COUNT || data == NULL) return;

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.last_rx_valid[index] = true;
    m_ctx.last_rx_id[index] = id;
    memcpy(m_ctx.last_rx_data[index], data, MX_CAN_FRAME_SIZE);
    portEXIT_CRITICAL(&m_ctx.lock);
}

static void set_controller_ids(unsigned int index, uint32_t status_id, uint32_t command_id) {
    if (index >= VCU_MX_CAN_CONTROLLER_COUNT) return;

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.controller[index].configured = status_id != 0U || command_id != 0U;
    m_ctx.controller[index].status_id = status_id;
    m_ctx.controller[index].command_id = command_id;
    m_ctx.controller[index].has_status = false;
    m_ctx.controller[index].status_frame_count = 0U;
    m_ctx.controller[index].last_status_ms = 0U;
    memset(&m_ctx.controller[index].status, 0, sizeof(m_ctx.controller[index].status));
    m_ctx.last_rx_valid[index] = false;
    memset(m_ctx.last_rx_data[index], 0, MX_CAN_FRAME_SIZE);
    m_ctx.last_tx_valid[index] = false;
    m_ctx.last_tx_count[index] = 0U;
    memset(m_ctx.last_tx_data[index], 0, MX_CAN_FRAME_SIZE);
    m_ctx.rpm_filter_initialized[index] = false;
    m_ctx.rpm_history_1[index] = 0;
    m_ctx.rpm_history_2[index] = 0;
    m_ctx.filtered_rpm[index] = 0;
    portEXIT_CRITICAL(&m_ctx.lock);
}

bool vcu_mx_can_test_process_can_frame(uint32_t id, const uint8_t *data, uint8_t len, bool is_ext) {
    if (!is_ext) return false;

    int matched_index = -1;
    portENTER_CRITICAL(&m_ctx.lock);
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        if (m_ctx.controller[i].configured && m_ctx.controller[i].status_id == id) {
            matched_index = (int)i;
            break;
        }
    }
    portEXIT_CRITICAL(&m_ctx.lock);

    if (matched_index < 0) return false;

    if (len != MX_CAN_FRAME_SIZE || data == NULL) {
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.ignored_frame_count++;
        portEXIT_CRITICAL(&m_ctx.lock);
        return false;
    }

    mx_can_status_t status;
    if (!mx_can_unpack_status(data, &status)) return false;
    const int8_t direction_sign = m_ctx.controller[matched_index].direction_sign;
    status.axis1_speed_rpm = normalize_vehicle_rpm(status.axis1_speed_rpm, direction_sign);
    const int16_t vehicle_rpm = normalize_vehicle_rpm(status.axis2_speed_rpm, direction_sign);
    status.axis2_speed_rpm = filter_vehicle_rpm((unsigned int)matched_index, vehicle_rpm);
    store_last_rx((unsigned int)matched_index, id, data);

    portENTER_CRITICAL(&m_ctx.lock);
    vcu_mx_can_controller_state_t *controller = &m_ctx.controller[matched_index];
    controller->status = status;
    controller->has_status = true;
    controller->status_frame_count++;
    controller->last_status_ms = now_ms();
    portEXIT_CRITICAL(&m_ctx.lock);
    return true;
}

bool vcu_mx_can_test_get_state(vcu_mx_can_test_state_t *state) {
    if (state == NULL) return false;

    portENTER_CRITICAL(&m_ctx.lock);
    memset(state, 0, sizeof(*state));
    state->initialized = m_ctx.initialized;
    state->ignored_frame_count = m_ctx.ignored_frame_count;
    memcpy(state->controller, m_ctx.controller, sizeof(state->controller));
    memcpy(state->last_tx_valid, m_ctx.last_tx_valid, sizeof(state->last_tx_valid));
    memcpy(state->last_tx_count, m_ctx.last_tx_count, sizeof(state->last_tx_count));
    memcpy(state->last_tx_id, m_ctx.last_tx_id, sizeof(state->last_tx_id));
    memcpy(state->last_tx_data, m_ctx.last_tx_data, sizeof(state->last_tx_data));
    memcpy(state->last_rx_valid, m_ctx.last_rx_valid, sizeof(state->last_rx_valid));
    memcpy(state->last_rx_id, m_ctx.last_rx_id, sizeof(state->last_rx_id));
    memcpy(state->last_rx_data, m_ctx.last_rx_data, sizeof(state->last_rx_data));
    portEXIT_CRITICAL(&m_ctx.lock);
    return true;
}

static bool controller_is_online(const vcu_mx_can_controller_state_t *controller, uint32_t age_ms) {
    return controller_is_configured(controller) && controller->has_status && age_ms <= VCU_MX_CAN_STATUS_TIMEOUT_MS;
}

static void print_controller_status(const vcu_mx_can_controller_state_t *controller,
        unsigned int index, uint32_t age_ms) {
    const bool online = controller_is_online(controller, age_ms);
    commands_printf(
            "%s:\n"
            "CAN      : %s, %lu ms\n"
            "Fault    : %s\n"
            "RPM      : %d rpm\n"
            "Temp     : %u C\n"
            "Voltage  : %u.%u V",
            index == VCU_MX_CAN_LEFT ? "Left" : "Right",
            online ? "Online" : "Offline",
            (unsigned long)age_ms,
            mx_can_fault_name(controller->status.fault_code),
            controller->status.axis2_speed_rpm,
            controller->status.controller_temperature_c,
            controller->status.supply_voltage_decivolts / 10U,
            controller->status.supply_voltage_decivolts % 10U);
}


bool vcu_mx_can_test_commands_configured(void) {
    bool configured = false;
    portENTER_CRITICAL(&m_ctx.lock);
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        configured = configured || (m_ctx.controller[i].configured && m_ctx.controller[i].command_id != 0U);
    }
    portEXIT_CRITICAL(&m_ctx.lock);
    return configured;
}

bool vcu_mx_can_test_send_axis2_targets(int16_t left_target, int16_t right_target, uint8_t ramp, bool enable) {
    uint32_t command_id[VCU_MX_CAN_CONTROLLER_COUNT];
    int16_t target[VCU_MX_CAN_CONTROLLER_COUNT] = {
        controller_target(VCU_MX_CAN_LEFT, left_target),
        controller_target(VCU_MX_CAN_RIGHT, right_target),
    };
    portENTER_CRITICAL(&m_ctx.lock);
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        command_id[i] = m_ctx.controller[i].configured ? m_ctx.controller[i].command_id : 0U;
    }
    portEXIT_CRITICAL(&m_ctx.lock);

    bool sent = false;
    uint8_t data[MX_CAN_FRAME_SIZE];
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        if (command_id[i] == 0U) continue;
        if (!mx_can_pack_axis2_speed_command(target[i], ramp, enable, data)) return false;
        comm_can_transmit_eid(command_id[i], data, MX_CAN_FRAME_SIZE);
        store_last_tx(i, command_id[i], data);
        sent = true;
    }
    return sent;
}

bool vcu_mx_can_test_send_reset(void) {
    uint32_t command_id[VCU_MX_CAN_CONTROLLER_COUNT];
    portENTER_CRITICAL(&m_ctx.lock);
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        command_id[i] = m_ctx.controller[i].configured ? m_ctx.controller[i].command_id : 0U;
    }
    portEXIT_CRITICAL(&m_ctx.lock);

    const mx_can_command_t command = {
        .axis1_mode = MX_CAN_CONTROL_TORQUE,
        .axis2_mode = MX_CAN_CONTROL_TORQUE,
        .axis1_enable = false,
        .axis2_enable = false,
        .reset = true,
        .coast = false,
        .axis1_target = 0,
        .axis2_target = 0,
        .axis1_ramp = 0,
        .axis2_ramp = 0,
    };

    uint8_t data[MX_CAN_FRAME_SIZE];
    if (!mx_can_pack_command(&command, data)) return false;

    bool sent = false;
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        if (command_id[i] == 0U) continue;
        comm_can_transmit_eid(command_id[i], data, MX_CAN_FRAME_SIZE);
        store_last_tx(i, command_id[i], data);
        sent = true;
    }
    return sent;
}

static void terminal_mx_can_status(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    vcu_mx_can_test_state_t state;
    vcu_mx_can_test_get_state(&state);

    const uint32_t current_ms = now_ms();
    uint32_t age_ms[VCU_MX_CAN_CONTROLLER_COUNT];
    bool normal = state.initialized;
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        age_ms[i] = state.controller[i].has_status ?
                current_ms - state.controller[i].last_status_ms : 0U;
        normal = normal && controller_is_online(&state.controller[i], age_ms[i]) &&
                state.controller[i].status.fault_code == 0U;
    }

    const int32_t rpm_difference = abs((int32_t)state.controller[VCU_MX_CAN_LEFT].status.axis2_speed_rpm -
            (int32_t)state.controller[VCU_MX_CAN_RIGHT].status.axis2_speed_rpm);
    commands_printf(
            "MX CAN: %s\n"
            "Ignored RX: %lu\n"
            "RPM difference : %ld rpm",
            normal ? "Normal" : "Warning",
            (unsigned long)state.ignored_frame_count,
            (long)rpm_difference);

    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        print_controller_status(&state.controller[i], i, age_ms[i]);
    }
}

static void terminal_mx_can_raw(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    vcu_mx_can_test_state_t state;
    vcu_mx_can_test_get_state(&state);

    commands_printf("MX CAN last raw frames");
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        if (state.last_tx_valid[i]) {
            commands_printf(
                    "%s tx\n"
                    "valid : 1\n"
                    "count : %lu\n"
                    "id    : 0x%08lx\n"
                    "data  : %02x %02x %02x %02x %02x %02x %02x %02x",
                    controller_name(i),
                    (unsigned long)state.last_tx_count[i],
                    (unsigned long)state.last_tx_id[i],
                    state.last_tx_data[i][0], state.last_tx_data[i][1],
                    state.last_tx_data[i][2], state.last_tx_data[i][3],
                    state.last_tx_data[i][4], state.last_tx_data[i][5],
                    state.last_tx_data[i][6], state.last_tx_data[i][7]);
        } else {
            commands_printf("%s tx\nvalid : 0", controller_name(i));
        }

        if (state.last_rx_valid[i]) {
            commands_printf(
                    "%s rx\n"
                    "valid : 1\n"
                    "id    : 0x%08lx\n"
                    "data  : %02x %02x %02x %02x %02x %02x %02x %02x",
                    controller_name(i),
                    (unsigned long)state.last_rx_id[i],
                    state.last_rx_data[i][0], state.last_rx_data[i][1],
                    state.last_rx_data[i][2], state.last_rx_data[i][3],
                    state.last_rx_data[i][4], state.last_rx_data[i][5],
                    state.last_rx_data[i][6], state.last_rx_data[i][7]);
        } else {
            commands_printf("%s rx\nvalid : 0", controller_name(i));
        }
    }
}

static void terminal_mx_can_ids(int argc, const char **argv) {
    if (argc >= 5) {
        set_controller_ids(0U, parse_can_id_arg(argv[1]), parse_can_id_arg(argv[2]));
        set_controller_ids(1U, parse_can_id_arg(argv[3]), parse_can_id_arg(argv[4]));
    } else if (argc >= 3) {
        set_controller_ids(0U, parse_can_id_arg(argv[1]), parse_can_id_arg(argv[2]));
    }

    vcu_mx_can_test_state_t state;
    vcu_mx_can_test_get_state(&state);
    commands_printf(
            "MX CAN IDs\n"
            "left  status=0x%08lx command=0x%08lx\n"
            "right status=0x%08lx command=0x%08lx\n"
            "usage: mx_can_ids <left_status> <left_command> [right_status right_command]",
            (unsigned long)state.controller[0].status_id,
            (unsigned long)state.controller[0].command_id,
            (unsigned long)state.controller[1].status_id,
            (unsigned long)state.controller[1].command_id);
}

void vcu_mx_can_test_init(void) {
    if (m_ctx.initialized) return;

    terminal_register_command_callback("mx_can_status", "Print MX controller CAN status.", 0,
            terminal_mx_can_status);
    terminal_register_command_callback("mx_can_ids", "Print or set MX left/right CAN IDs.", 0,
            terminal_mx_can_ids);
    terminal_register_command_callback("mx_can_raw", "Print last raw MX CAN RX/TX frames.", 0,
            terminal_mx_can_raw);

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.initialized = true;
    portEXIT_CRITICAL(&m_ctx.lock);
}
