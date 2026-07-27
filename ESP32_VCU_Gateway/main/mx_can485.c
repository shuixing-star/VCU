/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include "mx_can485.h"

#include "buffer.h"
#include "commands.h"
#include "comm_can.h"
#include "mx_rs485_protocol.h"
#include "terminal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MX_CAN485_RX_BUF_LEN 32
#define MX_CAN485_DEFAULT_TX_ID 0x101
#define MX_CAN485_DEFAULT_RX_ID 0x102
#define MX_CAN485_DEFAULT_PERIOD_MS 200
#define MX_CAN485_MIN_PERIOD_MS 50
#define MX_CAN485_RESPONSE_TIMEOUT_MS 120
#define MX_CAN485_DEFAULT_STREAM_HZ 10
#define MX_CAN485_MIN_STREAM_HZ 1
#define MX_CAN485_MAX_STREAM_HZ 50
#define MX_CAN485_APP_STATUS_MAX_LEN 64

typedef struct {
    uint32_t tx_id;
    uint32_t rx_id;
    bool use_ext_id;
    uint8_t mx_address;

    uint8_t rx_buf[MX_CAN485_RX_BUF_LEN];
    size_t rx_len;
    size_t expected_len;

    mx_rs485_sample sample;
    mx_rs485_result last_result;
    TickType_t last_rx_time;
    TickType_t last_poll_time;
    uint32_t rx_frames;
    uint32_t rx_ok;
    uint32_t rx_bad;
    uint32_t rx_overflow;
    uint32_t rx_timeout;
    uint32_t tx_count;
    bool has_sample;
    bool online;
    bool pending_response;
    bool auto_poll_enabled;
    bool auto_task_running;
    bool stream_enabled;
    bool stream_task_running;
    uint32_t period_ms;
    uint32_t stream_period_ms;
    uint32_t stream_tx;
} mx_can485_state;

static mx_can485_state m_state = {
    .tx_id = MX_CAN485_DEFAULT_TX_ID,
    .rx_id = MX_CAN485_DEFAULT_RX_ID,
    .use_ext_id = false,
    .mx_address = 1,
    .last_result = MX_RS485_RESULT_BAD_LEN,
    .period_ms = MX_CAN485_DEFAULT_PERIOD_MS,
    .stream_period_ms = 1000 / MX_CAN485_DEFAULT_STREAM_HZ,
};

bool mx_can485_get_state(mx_can485_public_state *state) {
    if (!state) {
        return false;
    }

    state->tx_id = m_state.tx_id;
    state->rx_id = m_state.rx_id;
    state->use_ext_id = m_state.use_ext_id;
    state->mx_address = m_state.mx_address;
    state->last_result = m_state.last_result;
    state->sample = m_state.sample;
    state->rx_frames = m_state.rx_frames;
    state->rx_ok = m_state.rx_ok;
    state->rx_bad = m_state.rx_bad;
    state->rx_overflow = m_state.rx_overflow;
    state->rx_timeout = m_state.rx_timeout;
    state->tx_count = m_state.tx_count;
    state->stream_tx = m_state.stream_tx;
    state->has_sample = m_state.has_sample;
    state->online = m_state.online;
    state->pending_response = m_state.pending_response;
    state->auto_poll_enabled = m_state.auto_poll_enabled;
    state->stream_enabled = m_state.stream_enabled;
    state->period_ms = m_state.period_ms;
    state->stream_period_ms = m_state.stream_period_ms;

    return true;
}

static unsigned int parse_u32_arg(const char *arg) {
    return (unsigned int)strtoul(arg, 0, 0);
}

static uint8_t parse_hex_byte_arg(const char *arg) {
    if (arg[0] == '0' && (arg[1] == 'x' || arg[1] == 'X')) {
        arg += 2;
    }

    return (uint8_t)(strtoul(arg, 0, 16) & 0xFF);
}

static void print_bytes(const uint8_t *data, size_t len) {
    char line[100];
    size_t pos = 0;

    for (size_t i = 0;i < len && pos < (sizeof(line) - 4);i++) {
        int written = snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i]);
        if (written <= 0) {
            break;
        }
        pos += (size_t)written;
    }

    if (pos > 0) {
        line[pos - 1] = '\0';
    } else {
        line[0] = '\0';
    }

    commands_printf("%s", line);
}

static void print_sample(const mx_rs485_sample *s) {
    commands_printf(
            "MX addr %u\n"
            "fault     : %u\n"
            "temp      : %d C\n"
            "voltage   : %.1f V\n"
            "m1 current: %.1f A\n"
            "m2 current: %.1f A\n"
            "m1 rpm    : %d\n"
            "m2 rpm    : %d",
            s->address,
            s->fault_code,
            s->temperature_c,
            (double)s->voltage_v,
            (double)s->motor1_current_a,
            (double)s->motor2_current_a,
            s->motor1_speed_rpm,
            s->motor2_speed_rpm);
}

static void send_app_status(void) {
    uint8_t buffer[MX_CAN485_APP_STATUS_MAX_LEN];
    int32_t ind = 0;

    buffer[ind++] = MX_CAN485_APP_PACKET_STATUS;
    buffer[ind++] = MX_CAN485_APP_PACKET_VERSION;
    buffer[ind++] = m_state.online ? 1 : 0;
    buffer[ind++] = m_state.has_sample ? 1 : 0;
    buffer[ind++] = m_state.mx_address;
    buffer_append_uint16(buffer, m_state.sample.fault_code, &ind);
    buffer_append_int16(buffer, m_state.sample.temperature_c, &ind);
    buffer_append_float32_auto(buffer, m_state.sample.voltage_v, &ind);
    buffer_append_float32_auto(buffer, m_state.sample.motor1_current_a, &ind);
    buffer_append_float32_auto(buffer, m_state.sample.motor2_current_a, &ind);
    buffer_append_int32(buffer, m_state.sample.motor1_speed_rpm, &ind);
    buffer_append_int32(buffer, m_state.sample.motor2_speed_rpm, &ind);
    buffer_append_uint32(buffer, m_state.tx_count, &ind);
    buffer_append_uint32(buffer, m_state.rx_ok, &ind);
    buffer_append_uint32(buffer, m_state.rx_timeout, &ind);
    buffer_append_uint32(buffer, m_state.rx_bad, &ind);
    buffer_append_uint32(buffer, m_state.rx_overflow, &ind);

    commands_send_app_data(buffer, ind);
    m_state.stream_tx++;
}

static void reset_rx_buffer(void) {
    m_state.rx_len = 0;
    m_state.expected_len = 0;
}

static void parse_rx_if_complete(void) {
    if (m_state.expected_len == 0) {
        m_state.expected_len = mx_rs485_expected_response_len(m_state.rx_buf, m_state.rx_len);
    }

    if (m_state.expected_len == 0 || m_state.rx_len < m_state.expected_len) {
        return;
    }

    mx_rs485_sample sample;
    mx_rs485_result result = mx_rs485_parse_read_response(
            m_state.mx_address,
            m_state.rx_buf,
            m_state.expected_len,
            &sample);

    m_state.last_result = result;
    m_state.last_rx_time = xTaskGetTickCount();

    if (result == MX_RS485_RESULT_OK) {
        m_state.sample = sample;
        m_state.has_sample = true;
        m_state.online = true;
        m_state.rx_ok++;
    } else {
        m_state.online = false;
        m_state.rx_bad++;
    }

    m_state.pending_response = false;
    reset_rx_buffer();
}

bool mx_can485_process_can_frame(uint32_t id, const uint8_t *data, uint8_t len, bool is_ext) {
    if (!data || len == 0 || id != m_state.rx_id || is_ext != m_state.use_ext_id) {
        return false;
    }

    m_state.rx_frames++;

    if ((m_state.rx_len + len) > sizeof(m_state.rx_buf)) {
        reset_rx_buffer();
        m_state.rx_overflow++;
        m_state.last_result = MX_RS485_RESULT_BAD_LEN;
        return true;
    }

    memcpy(&m_state.rx_buf[m_state.rx_len], data, len);
    m_state.rx_len += len;
    parse_rx_if_complete();

    return true;
}

static bool send_poll(uint8_t address, bool print) {
    uint8_t req[MX_RS485_READ_REQUEST_LEN];

    if (!mx_rs485_build_read_request(address, req, sizeof(req))) {
        return false;
    }

    m_state.mx_address = address;
    reset_rx_buffer();

    if (m_state.use_ext_id) {
        comm_can_transmit_eid(m_state.tx_id, req, sizeof(req));
    } else {
        comm_can_transmit_sid(m_state.tx_id, req, sizeof(req));
    }

    m_state.tx_count++;
    m_state.last_poll_time = xTaskGetTickCount();
    m_state.pending_response = true;

    if (print) {
        commands_printf("MX poll sent to CAN %s 0x%lx, MX addr %u, bytes:",
                m_state.use_ext_id ? "EID" : "SID",
                (unsigned long)m_state.tx_id,
                address);
        print_bytes(req, sizeof(req));
    }

    return true;
}

static void auto_poll_task(void *arg) {
    (void)arg;

    m_state.auto_task_running = true;

    while (m_state.auto_poll_enabled) {
        send_poll(m_state.mx_address, false);

        uint32_t waited_ms = 0;
        while (m_state.auto_poll_enabled &&
                m_state.pending_response &&
                waited_ms < MX_CAN485_RESPONSE_TIMEOUT_MS) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            waited_ms += 10;
        }

        if (m_state.pending_response) {
            m_state.pending_response = false;
            m_state.online = false;
            m_state.rx_timeout++;
            reset_rx_buffer();
        }

        uint32_t elapsed_ms = waited_ms;
        while (m_state.auto_poll_enabled && elapsed_ms < m_state.period_ms) {
            uint32_t step_ms = m_state.period_ms - elapsed_ms;
            if (step_ms > 20) {
                step_ms = 20;
            }
            vTaskDelay(step_ms / portTICK_PERIOD_MS);
            elapsed_ms += step_ms;
        }
    }

    m_state.auto_task_running = false;
    vTaskDelete(NULL);
}

static void stream_task(void *arg) {
    (void)arg;

    m_state.stream_task_running = true;

    while (m_state.stream_enabled) {
        send_app_status();
        vTaskDelay(m_state.stream_period_ms / portTICK_PERIOD_MS);
    }

    m_state.stream_task_running = false;
    vTaskDelete(NULL);
}

static void terminal_mx_cfg(int argc, const char **argv) {
    if (argc >= 3) {
        m_state.tx_id = parse_u32_arg(argv[1]);
        m_state.rx_id = parse_u32_arg(argv[2]);
    }

    if (argc >= 4) {
        m_state.use_ext_id = strcmp(argv[3], "eid") == 0 || strcmp(argv[3], "ext") == 0;
    }

    if (argc >= 5) {
        unsigned int addr = parse_u32_arg(argv[4]);
        if (addr >= 1 && addr <= 0x7F) {
            m_state.mx_address = addr;
        }
    }

    commands_printf(
            "MX CAN485 cfg\n"
            "tx_id   : 0x%lx\n"
            "rx_id   : 0x%lx\n"
            "id type : %s\n"
            "mx addr : %u",
            (unsigned long)m_state.tx_id,
            (unsigned long)m_state.rx_id,
            m_state.use_ext_id ? "EID" : "SID",
            m_state.mx_address);
}

static void terminal_mx_poll(int argc, const char **argv) {
    uint8_t addr = m_state.mx_address;

    if (argc >= 2) {
        unsigned int parsed = parse_u32_arg(argv[1]);
        if (parsed >= 1 && parsed <= 0x7F) {
            addr = parsed;
        }
    }

    if (!send_poll(addr, true)) {
        commands_printf("Invalid MX address: %u", addr);
    }
}

bool mx_can485_start_poll(uint32_t period_ms) {
    if (period_ms < MX_CAN485_MIN_PERIOD_MS) {
        period_ms = MX_CAN485_MIN_PERIOD_MS;
    }

    m_state.period_ms = period_ms;

    m_state.auto_poll_enabled = true;
    if (!m_state.auto_task_running) {
        BaseType_t res = xTaskCreatePinnedToCore(
                auto_poll_task, "mx_poll", 2048, NULL, 8, NULL, tskNO_AFFINITY);
        if (res != pdPASS) {
            m_state.auto_poll_enabled = false;
            return false;
        }
    }

    return true;
}

void mx_can485_stop_poll(void) {
    m_state.auto_poll_enabled = false;
}

bool mx_can485_start_stream(uint32_t hz) {
    if (hz < MX_CAN485_MIN_STREAM_HZ) {
        hz = MX_CAN485_MIN_STREAM_HZ;
    } else if (hz > MX_CAN485_MAX_STREAM_HZ) {
        hz = MX_CAN485_MAX_STREAM_HZ;
    }

    m_state.stream_period_ms = 1000 / hz;
    m_state.stream_enabled = true;

    if (!m_state.stream_task_running) {
        m_state.stream_task_running = true;
        BaseType_t res = xTaskCreatePinnedToCore(
                stream_task, "mx_stream", 2048, NULL, 6, NULL, tskNO_AFFINITY);
        if (res != pdPASS) {
            m_state.stream_task_running = false;
            m_state.stream_enabled = false;
            return false;
        }
    }

    return true;
}

void mx_can485_stop_stream(void) {
    m_state.stream_enabled = false;
}

static void terminal_mx_start(int argc, const char **argv) {
    uint32_t period_ms = m_state.period_ms;

    if (argc >= 2) {
        period_ms = parse_u32_arg(argv[1]);
    }

    if (!mx_can485_start_poll(period_ms)) {
        commands_printf("Failed to start MX auto polling task.");
        return;
    }

    commands_printf("MX auto polling started, period: %lu ms", (unsigned long)m_state.period_ms);
}

static void terminal_mx_stop(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    mx_can485_stop_poll();
    commands_printf("MX auto polling stopped.");
}

static void terminal_mx_stream_start(int argc, const char **argv) {
    uint32_t hz = MX_CAN485_DEFAULT_STREAM_HZ;

    if (argc >= 2) {
        hz = parse_u32_arg(argv[1]);
    }

    if (!mx_can485_start_stream(hz)) {
        commands_printf("Failed to start MX app stream task.");
        return;
    }

    commands_printf("MX app stream started, period: %lu ms",
            (unsigned long)m_state.stream_period_ms);
}

static void terminal_mx_stream_stop(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    mx_can485_stop_stream();
    commands_printf("MX app stream stopped.");
}

static void terminal_mx_stream_once(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    send_app_status();
    commands_printf("MX app status packet sent.");
}

static void terminal_mx_status(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    commands_printf(
            "MX CAN485 status\n"
            "tx_id       : 0x%lx\n"
            "rx_id       : 0x%lx\n"
            "id type     : %s\n"
            "mx addr     : %u\n"
            "last result : %s\n"
            "online      : %d\n"
            "auto poll   : %d\n"
            "period ms   : %lu\n"
            "app stream  : %d\n"
            "stream ms   : %lu\n"
            "stream tx   : %lu\n"
            "tx count    : %lu\n"
            "rx frames   : %lu\n"
            "rx ok       : %lu\n"
            "rx bad      : %lu\n"
            "rx overflow : %lu\n"
            "rx timeout  : %lu\n"
            "pending rx  : %u",
            (unsigned long)m_state.tx_id,
            (unsigned long)m_state.rx_id,
            m_state.use_ext_id ? "EID" : "SID",
            m_state.mx_address,
            mx_rs485_result_name(m_state.last_result),
            m_state.online,
            m_state.auto_poll_enabled,
            (unsigned long)m_state.period_ms,
            m_state.stream_enabled,
            (unsigned long)m_state.stream_period_ms,
            (unsigned long)m_state.stream_tx,
            (unsigned long)m_state.tx_count,
            (unsigned long)m_state.rx_frames,
            (unsigned long)m_state.rx_ok,
            (unsigned long)m_state.rx_bad,
            (unsigned long)m_state.rx_overflow,
            (unsigned long)m_state.rx_timeout,
            (unsigned int)m_state.rx_len);

    if (m_state.has_sample) {
        print_sample(&m_state.sample);
    } else {
        commands_printf("No valid MX sample yet.");
    }
}

static void terminal_mx_parse(int argc, const char **argv) {
    uint8_t frame[MX_CAN485_RX_BUF_LEN];
    int len = 0;

    for (int i = 1;i < argc && len < (int)sizeof(frame);i++) {
        frame[len++] = parse_hex_byte_arg(argv[i]);
    }

    if (len == 0) {
        commands_printf("Usage: mx_parse [response bytes]");
        return;
    }

    mx_rs485_sample sample;
    mx_rs485_result result = mx_rs485_parse_read_response(
            m_state.mx_address,
            frame,
            len,
            &sample);

    commands_printf("mx_parse result: %s", mx_rs485_result_name(result));
    if (result == MX_RS485_RESULT_OK) {
        print_sample(&sample);
    }
}

static void terminal_mx_reset(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    reset_rx_buffer();
    m_state.rx_frames = 0;
    m_state.rx_ok = 0;
    m_state.rx_bad = 0;
    m_state.rx_overflow = 0;
    m_state.rx_timeout = 0;
    m_state.tx_count = 0;
    m_state.stream_tx = 0;
    m_state.has_sample = false;
    m_state.online = false;
    m_state.pending_response = false;
    m_state.last_result = MX_RS485_RESULT_BAD_LEN;
    commands_printf("MX CAN485 counters reset.");
}

void mx_can485_init(void) {
    terminal_register_command_callback(
            "mx_cfg",
            "Configure MX CAN485 bridge: mx_cfg [tx_id] [rx_id] [sid|eid] [mx_addr]",
            "[tx_id] [rx_id] [sid|eid] [mx_addr]",
            terminal_mx_cfg);

    terminal_register_command_callback(
            "mx_poll",
            "Send one MX read request through the CAN485 bridge.",
            "[mx_addr]",
            terminal_mx_poll);

    terminal_register_command_callback(
            "mx_start",
            "Start automatic MX polling.",
            "[period_ms]",
            terminal_mx_start);

    terminal_register_command_callback(
            "mx_stop",
            "Stop automatic MX polling.",
            0,
            terminal_mx_stop);

    terminal_register_command_callback(
            "mx_stream_start",
            "Start streaming MX status as COMM_CUSTOM_APP_DATA.",
            "[hz]",
            terminal_mx_stream_start);

    terminal_register_command_callback(
            "mx_stream_stop",
            "Stop streaming MX app status packets.",
            0,
            terminal_mx_stream_stop);

    terminal_register_command_callback(
            "mx_stream_once",
            "Send one MX app status packet.",
            0,
            terminal_mx_stream_once);

    terminal_register_command_callback(
            "mx_status",
            "Print MX CAN485 communication status and last parsed sample.",
            0,
            terminal_mx_status);

    terminal_register_command_callback(
            "mx_parse",
            "Parse MX response bytes locally for protocol verification.",
            "[bytes...]",
            terminal_mx_parse);

    terminal_register_command_callback(
            "mx_reset",
            "Reset MX CAN485 counters and pending RX buffer.",
            0,
            terminal_mx_reset);
}
