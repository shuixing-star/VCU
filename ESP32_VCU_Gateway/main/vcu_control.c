/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include "vcu_control.h"

#include "commands.h"
#include "mx_can485.h"
#include "mx_log.h"
#include "terminal.h"
#include "vcu_gps.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define VCU_DEFAULT_POLL_MS     200
#define VCU_DEFAULT_STREAM_HZ   10
#define VCU_DEFAULT_LOG_HZ      5

static unsigned int parse_u32_arg(const char *arg) {
    return (unsigned int)strtoul(arg, 0, 0);
}

static void terminal_vcu_start(int argc, const char **argv) {
    uint32_t poll_ms = VCU_DEFAULT_POLL_MS;
    uint32_t stream_hz = VCU_DEFAULT_STREAM_HZ;
    uint32_t log_hz = VCU_DEFAULT_LOG_HZ;

    if (argc >= 2) {
        poll_ms = parse_u32_arg(argv[1]);
    }

    if (argc >= 3) {
        stream_hz = parse_u32_arg(argv[2]);
    }

    if (argc >= 4) {
        log_hz = parse_u32_arg(argv[3]);
    }

    if (!mx_can485_start_poll(poll_ms)) {
        commands_printf("VCU start failed: MX poll task.");
        return;
    }

    if (!mx_can485_start_stream(stream_hz)) {
        commands_printf("VCU start failed: MX app stream task.");
        return;
    }

    if (!mx_log_start(log_hz)) {
        commands_printf("VCU start failed: MX log task.");
        return;
    }

    commands_printf(
            "VCU started\n"
            "poll ms   : %lu\n"
            "stream hz : %lu\n"
            "log hz    : %lu",
            (unsigned long)poll_ms,
            (unsigned long)stream_hz,
            (unsigned long)log_hz);
}

static void terminal_vcu_stop(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    mx_log_stop();
    mx_can485_stop_stream();
    mx_can485_stop_poll();
    commands_printf("VCU stopped.");
}

static void terminal_vcu_status(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    mx_can485_public_state mx;
    mx_log_public_state log;
    vcu_gps_public_state gps;
    bool mx_ok = mx_can485_get_state(&mx);
    bool log_ok = mx_log_get_state(&log);
    bool gps_ok = vcu_gps_get_state(&gps);

    if (!mx_ok || !log_ok || !gps_ok) {
        commands_printf("VCU status unavailable.");
        return;
    }

    commands_printf(
            "VCU status\n"
            "mx online   : %d\n"
            "auto poll   : %d\n"
            "poll ms     : %lu\n"
            "app stream  : %d\n"
            "stream ms   : %lu\n"
            "log enabled : %d\n"
            "log ms      : %lu\n"
            "log samples : %lu\n"
            "dup skips   : %lu\n"
            "file errors : %lu\n"
            "log path    : %s\n"
            "gps ready   : %d\n"
            "gps fix     : %d\n"
            "gps tod     : %d\n"
            "gps date    : %d\n"
            "gps sats    : %d\n"
            "gps hdop    : %.2f\n"
            "gps lat     : %.8f\n"
            "gps lon     : %.8f\n"
            "rx ok       : %lu\n"
            "rx timeout  : %lu\n"
            "rx bad      : %lu",
            mx.online,
            mx.auto_poll_enabled,
            (unsigned long)mx.period_ms,
            mx.stream_enabled,
            (unsigned long)mx.stream_period_ms,
            log.enabled,
            (unsigned long)log.period_ms,
            (unsigned long)log.samples_written,
            (unsigned long)log.duplicate_skips,
            (unsigned long)log.file_errors,
            log.active_path,
            gps.ublox_ready ? 1 : 0,
            gps.fix_valid ? 1 : 0,
            gps.tod_valid ? 1 : 0,
            gps.date_valid ? 1 : 0,
            gps.sats,
            (double)gps.hdop,
            gps.lat,
            gps.lon,
            (unsigned long)mx.rx_ok,
            (unsigned long)mx.rx_timeout,
            (unsigned long)mx.rx_bad);
}

void vcu_control_init(void) {
    terminal_register_command_callback(
            "vcu_start",
            "Start MX polling, app stream and SD logging: vcu_start [poll_ms] [stream_hz] [log_hz]",
            "[poll_ms] [stream_hz] [log_hz]",
            terminal_vcu_start);

    terminal_register_command_callback(
            "vcu_stop",
            "Stop MX logging, app stream and polling.",
            0,
            terminal_vcu_stop);

    terminal_register_command_callback(
            "vcu_status",
            "Print VCU acquisition status.",
            0,
            terminal_vcu_status);
}
