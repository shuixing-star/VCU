/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include "mx_log.h"

#include "commands.h"
#include "conf_general.h"
#include "log.h"
#include "terminal.h"
#include "utils.h"
#include "vcu_stm32_gateway.h"
#include "vcu_log_sample.h"
#ifdef HW_VCU_STM32_GATEWAY
#include "vcu_parking_sample.h"
#include "vcu_fault_log.h"
#endif
#include "vcu_crsf_uart.h"
#include "vcu_drive_runtime.h"
#include "vcu_mx_can_test.h"
#include "vcu_rc_drive.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/stat.h>

#define MX_LOG_DEFAULT_HZ 20
#define MX_LOG_MIN_HZ 1
#define MX_LOG_MAX_HZ 50
#define MX_LOG_DIR_NAME "mx_log"
#define MX_LOG_FILE_BUFFER_SIZE 4096U
#define MX_LOG_FLUSH_INTERVAL_MS 1000U
#define MX_LOG_SLOW_FLUSH_MS 50U

typedef struct {
    bool enabled;
    bool task_running;
    uint32_t period_ms;
    uint32_t samples_written;
    uint32_t file_errors;
    uint32_t flush_count;
    uint32_t slow_flush_count;
    uint32_t max_sample_write_ms;
    uint32_t max_flush_ms;
    int last_error_errno;
    char last_error_operation[32];
    char active_path[160];
} mx_log_state;

static mx_log_state m_log = {
    .period_ms = 1000 / MX_LOG_DEFAULT_HZ,
};

static char m_file_buffer[MX_LOG_FILE_BUFFER_SIZE];

static void record_file_error(const char *operation) {
    m_log.file_errors++;
    m_log.last_error_errno = errno != 0 ? errno : EIO;
    snprintf(m_log.last_error_operation, sizeof(m_log.last_error_operation), "%s", operation);
}

static unsigned int parse_u32_arg(const char *arg) {
    return (unsigned int)strtoul(arg, 0, 0);
}

bool mx_log_get_state(mx_log_public_state *state) {
    if (!state) {
        return false;
    }

    state->enabled = m_log.enabled;
    state->task_running = m_log.task_running;
    state->period_ms = m_log.period_ms;
    state->samples_written = m_log.samples_written;
    state->duplicate_skips = 0U;
    state->file_errors = m_log.file_errors;
    state->flush_count = m_log.flush_count;
    state->slow_flush_count = m_log.slow_flush_count;
    state->max_sample_write_ms = m_log.max_sample_write_ms;
    state->max_flush_ms = m_log.max_flush_ms;
    state->last_error_errno = m_log.last_error_errno;
    state->last_error_operation = m_log.last_error_operation[0] ? m_log.last_error_operation : "-";
    state->active_path = m_log.active_path[0] ? m_log.active_path : "-";

    return true;
}

static void make_log_dir(char *out, size_t out_len) {
    snprintf(out, out_len, "%s%s", file_basepath, MX_LOG_DIR_NAME);
}

static int find_next_log_index(const char *dir_path) {
    int highest = 0;
    DIR *dir = opendir(dir_path);

    if (!dir) {
        return 1;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        int idx = 0;
        if (sscanf(entry->d_name, "vcu_%d.csv", &idx) == 1 && idx > highest) {
            highest = idx;
        }
    }

    closedir(dir);
    return highest + 1;
}

static bool open_log_file(FILE **file) {
#ifndef HW_VCU_STM32_GATEWAY
    const char *header =
            "t_day:Time:s:3:0:1;throttle_raw:Throttle raw::0:0:0;steering_raw:Steering raw::0:0:0;"
            "left_target_rpm:Left target RPM:rpm:0:0:0;right_target_rpm:Right target RPM:rpm:0:0:0;"
            "left_axis2_rpm:Left measured RPM:rpm:0:0:0;right_axis2_rpm:Right measured RPM:rpm:0:0:0;"
            "left_fault:Left fault::0:0:0;right_fault:Right fault::0:0:0;"
            "left_voltage_v:Left voltage:V:1:0:0;right_voltage_v:Right voltage:V:1:0:0\n";
#endif
    char dir_path[120];
    make_log_dir(dir_path, sizeof(dir_path));
    mkdir(dir_path, 0775);

    int index = find_next_log_index(dir_path);
    snprintf(m_log.active_path, sizeof(m_log.active_path), "%s/vcu_%03d.csv", dir_path, index);

    errno = 0;
    FILE *f = fopen(m_log.active_path, "w");
    if (!f) {
        record_file_error("fopen");
        return false;
    }

    errno = 0;
    if (setvbuf(f, m_file_buffer, _IOFBF, sizeof(m_file_buffer)) != 0) {
        record_file_error("setvbuf");
        fclose(f);
        return false;
    }
#ifdef HW_VCU_STM32_GATEWAY
    bool header_ok = fprintf(f, "t_day:Time:s:3:0:1") >= 0;
    for (unsigned int i = 0; i < VCU_PARKING_CSV_FIELD_COUNT && header_ok; i++) {
        const vcu_log_field_t *h = vcu_parking_field(vcu_parking_csv_fields[i]);
        header_ok = fprintf(f, ";%s:%s:%s:%u:0:0",
                            h->key, h->name, h->unit, h->precision) >= 0;
    }
    header_ok = header_ok && fprintf(f, "\n") >= 0;
#else
    bool header_ok = fprintf(f, "%s", header) >= 0;
#endif
    if (!header_ok) {
        record_file_error("write header");
        fclose(f);
        return false;
    }
    *file = f;

    return true;
}



static bool write_sample(FILE *file) {
#ifdef HW_VCU_STM32_GATEWAY
    static vcu_stm32_gateway_state_t state; /* Only the SD task uses this buffer. */
    static vcu_parking_observer_t observers[2];
    if (!vcu_stm32_gateway_get_parking_snapshot(&state, observers)) {
        return false;
    }
    static double values[VCU_PARKING_FIELD_COUNT];
    const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    vcu_parking_sample_from_state(&state, observers, now, values);
    if (fprintf(file, "%.3f", (double)now / 1000.0) < 0) {
        return false;
    }
    for (unsigned int i = 0; i < VCU_PARKING_CSV_FIELD_COUNT; i++) {
        const unsigned int field = vcu_parking_csv_fields[i];
        if (fprintf(file, ";%.*f", (int)vcu_parking_field(field)->precision,
                    (double)values[field]) < 0) {
            return false;
        }
    }
    return fprintf(file, "\n") >= 0;
#else
    vcu_crsf_uart_state_t crsf;
    vcu_rc_drive_config_t config;
    vcu_rc_drive_preview_t preview;
    vcu_drive_runtime_state_t drive;
    vcu_mx_can_test_state_t mx;

    vcu_crsf_uart_get_state(&crsf);
    vcu_rc_drive_get_default_config(&config);
    vcu_rc_drive_preview_from_channels(crsf.channels, crsf.link_ok, &config, &preview);
    vcu_drive_runtime_get_state(&drive);
    vcu_mx_can_test_get_state(&mx);

    const uint32_t now = (uint32_t)utils_ms_tot();

    return fprintf(file,
            "%.3f;%u;%u;%d;%d;%d;%d;%u;%u;%u.%u;%u.%u\n",
            (double)now / 1000.0,
            preview.throttle_raw,
            preview.steering_raw,
            drive.latest_left_target,
            drive.latest_right_target,
            mx.controller[0].status.axis2_speed_rpm,
            mx.controller[1].status.axis2_speed_rpm,
            mx.controller[0].status.fault_code,
            mx.controller[1].status.fault_code,
            mx.controller[0].status.supply_voltage_decivolts / 10U,
            mx.controller[0].status.supply_voltage_decivolts % 10U,
            mx.controller[1].status.supply_voltage_decivolts / 10U,
            mx.controller[1].status.supply_voltage_decivolts % 10U) >= 0;
#endif
}
static bool flush_log_file(FILE *file) {
    const uint32_t start_ms = (uint32_t)utils_ms_tot();
    errno = 0;
    const bool ok = fflush(file) == 0 && !ferror(file);
    const uint32_t elapsed_ms = (uint32_t)utils_ms_tot() - start_ms;

    m_log.flush_count++;
    if (elapsed_ms > m_log.max_flush_ms) {
        m_log.max_flush_ms = elapsed_ms;
    }
    if (elapsed_ms >= MX_LOG_SLOW_FLUSH_MS) {
        m_log.slow_flush_count++;
    }
    if (!ok) {
        record_file_error("fflush");
    }
    return ok;
}

static void mx_log_task(void *arg) {
    (void)arg;

    m_log.task_running = true;
    FILE *file = NULL;

    if (!open_log_file(&file)) {
        m_log.enabled = false;
    }

#ifdef HW_VCU_STM32_GATEWAY
    /* Same session number for the main CSV and both event companions. The
     * existing CAN callback assembles records; only this task writes SD. */
    if (file && !vcu_fault_log_open(m_log.active_path)) {
        record_file_error("open fault/trace");
        m_log.enabled = false;
    }
#endif

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_flush = last_wake;
    while (m_log.enabled) {
        if (file) {
            const uint32_t start_ms = (uint32_t)utils_ms_tot();
            errno = 0;
            if (!write_sample(file) || ferror(file)) {
                record_file_error("write sample");
                m_log.enabled = false;
                break;
            }
            const uint32_t elapsed_ms = (uint32_t)utils_ms_tot() - start_ms;
            if (elapsed_ms > m_log.max_sample_write_ms) {
                m_log.max_sample_write_ms = elapsed_ms;
            }
            m_log.samples_written++;
#ifdef HW_VCU_STM32_GATEWAY
            if (!vcu_fault_log_drain()) {
                record_file_error("write fault/trace");
                m_log.enabled = false;
                break;
            }
#endif

            if ((xTaskGetTickCount() - last_flush) >= pdMS_TO_TICKS(MX_LOG_FLUSH_INTERVAL_MS)) {
                if (!flush_log_file(file)) {
                    m_log.enabled = false;
                    break;
                }
                last_flush = xTaskGetTickCount();
            }
        } else {
            errno = EIO;
            record_file_error("file unavailable");
            m_log.enabled = false;
            break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(m_log.period_ms));
    }

#ifdef HW_VCU_STM32_GATEWAY
    /* Stop fixes the event boundary; finish waits for packets already in
     * flight. Keep task_running true until all three files are closed. */
    vcu_fault_log_stop((uint32_t)utils_ms_tot());
    if (!vcu_fault_log_finish()) record_file_error("close fault/trace");
#endif
    if (file) {
        if (!flush_log_file(file)) {
            clearerr(file);
        }
        errno = 0;
        if (fclose(file) != 0) {
            record_file_error("fclose");
        }
    }

    m_log.task_running = false;
    vTaskDelete(NULL);
}

bool mx_log_start(uint32_t hz) {
    if (hz < MX_LOG_MIN_HZ) {
        hz = MX_LOG_MIN_HZ;
    } else if (hz > MX_LOG_MAX_HZ) {
        hz = MX_LOG_MAX_HZ;
    }

    m_log.period_ms = 1000 / hz;

    if (m_log.task_running) {
        return true;
    }

    m_log.samples_written = 0;
    m_log.file_errors = 0;
    m_log.flush_count = 0;
    m_log.slow_flush_count = 0;
    m_log.max_sample_write_ms = 0;
    m_log.max_flush_ms = 0;
    m_log.last_error_errno = 0;
    m_log.last_error_operation[0] = '\0';
    m_log.active_path[0] = '\0';
    m_log.enabled = true;
    m_log.task_running = true;
#ifdef HW_VCU_STM32_GATEWAY
    vcu_fault_log_begin((uint32_t)utils_ms_tot());
#endif
    BaseType_t res = xTaskCreatePinnedToCore(
            mx_log_task, "mx_log", 4096, NULL, 6, NULL, tskNO_AFFINITY);
    if (res != pdPASS) {
#ifdef HW_VCU_STM32_GATEWAY
        vcu_fault_log_stop((uint32_t)utils_ms_tot());
        if (!vcu_fault_log_finish()) record_file_error("close fault/trace");
#endif
        m_log.task_running = false;
        m_log.enabled = false;
        return false;
    }

    return true;
}

void mx_log_stop(void) {
#ifdef HW_VCU_STM32_GATEWAY
    vcu_fault_log_stop((uint32_t)utils_ms_tot());
#endif
    m_log.enabled = false;
}

static void terminal_mx_log_start(int argc, const char **argv) {
    uint32_t hz = MX_LOG_DEFAULT_HZ;

    if (argc >= 2) {
        hz = parse_u32_arg(argv[1]);
    }

    bool was_running = m_log.task_running;

    if (!mx_log_start(hz)) {
        commands_printf("Failed to start VCU log task.");
        return;
    }

    if (was_running) {
        commands_printf("VCU logging already running, rate updated to %lu Hz",
                (unsigned long)hz);
        return;
    }

    commands_printf("VCU logging started, rate: %lu Hz. Use mx_log_status to check file open result.",
            (unsigned long)hz);
}

static void terminal_mx_log_stop(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    mx_log_stop();
    commands_printf("VCU logging stopped.");
}

static void terminal_mx_log_status(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    commands_printf(
            /* Recording hosts match this first line exactly. Keep profile in CSV. */
            "VCU log status\n"
            "enabled       : %d\n"
            "task running  : %d\n"
            "period ms     : %lu\n"
            "samples       : %lu\n"
            "file errors   : %lu\n"
            "flushes       : %lu\n"
            "slow flushes  : %lu (>= %u ms)\n"
            "max sample ms : %lu\n"
            "max flush ms  : %lu\n"
            "last error    : %s: errno %d (%s)\n"
            "active path   : %s",
            m_log.enabled,
            m_log.task_running,
            (unsigned long)m_log.period_ms,
            (unsigned long)m_log.samples_written,
            (unsigned long)m_log.file_errors,
            (unsigned long)m_log.flush_count,
            (unsigned long)m_log.slow_flush_count,
            MX_LOG_SLOW_FLUSH_MS,
            (unsigned long)m_log.max_sample_write_ms,
            (unsigned long)m_log.max_flush_ms,
            m_log.last_error_operation[0] ? m_log.last_error_operation : "-",
            m_log.last_error_errno,
            m_log.last_error_errno ? strerror(m_log.last_error_errno) : "none",
            m_log.active_path[0] ? m_log.active_path : "-");
}

void mx_log_init(void) {
    terminal_register_command_callback(
            "mx_log_start",
            "Start parking/brake CSV: mx_log_start [hz]",
            "[hz]",
            terminal_mx_log_start);

    terminal_register_command_callback(
            "mx_log_stop",
            "Stop VCU CSV logging.",
            0,
            terminal_mx_log_stop);

    terminal_register_command_callback(
            "mx_log_status",
            "Print VCU CSV logging status.",
            0,
            terminal_mx_log_status);
}
