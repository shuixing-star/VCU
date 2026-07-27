/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include "vcu_gps.h"

#include "commands.h"
#include "conf_general.h"
#include "nmea.h"
#include "terminal.h"
#include "ublox.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static uint32_t age_ms(uint32_t update_time) {
    if (update_time == 0) {
        return 0;
    }

    return (uint32_t)((xTaskGetTickCount() - update_time) * portTICK_PERIOD_MS);
}

static bool date_time_valid(const nmea_state_t *nmea) {
    return nmea->rmc.yy >= 0 && nmea->rmc.mo >= 0 && nmea->rmc.dd >= 0 &&
            nmea->rmc.hh >= 0 && nmea->rmc.mm >= 0 && nmea->rmc.ss >= 0;
}

static bool tod_valid(const nmea_state_t *nmea) {
    return nmea->rmc.hh >= 0 && nmea->rmc.mm >= 0 && nmea->rmc.ss >= 0;
}

static bool date_valid(const nmea_state_t *nmea) {
    return nmea->rmc.yy >= 0 && nmea->rmc.mo >= 0 && nmea->rmc.dd >= 0;
}

bool vcu_gps_get_state(vcu_gps_public_state *state) {
    if (!state) {
        return false;
    }

    memset(state, 0, sizeof(*state));

    nmea_state_t *nmea = nmea_get_state();
    state->ublox_ready = ublox_init_ok();
    state->has_gga = nmea->gga_cnt > 0;
    state->has_rmc = nmea->rmc_cnt > 0;
    state->fix_valid = state->has_gga && nmea->gga.fix_type > 0;
    state->tod_valid = state->has_rmc && tod_valid(nmea);
    state->date_valid = state->has_rmc && date_valid(nmea);
    state->time_valid = state->has_rmc && date_time_valid(nmea);
    state->gga_age_ms = age_ms(nmea->gga.update_time);
    state->rmc_age_ms = age_ms(nmea->rmc.update_time);
    state->gga_count = nmea->gga_cnt;
    state->rmc_count = nmea->rmc_cnt;
    state->fix_type = nmea->gga.fix_type;
    state->sats = nmea->gga.n_sat;
    state->hdop = nmea->gga.h_dop;
    state->lat = nmea->gga.lat;
    state->lon = nmea->gga.lon;
    state->height_m = nmea->gga.height;
    state->speed_mps = nmea->rmc.speed;
    state->yy = nmea->rmc.yy;
    state->mo = nmea->rmc.mo;
    state->dd = nmea->rmc.dd;
    state->hh = nmea->rmc.hh;
    state->mm = nmea->rmc.mm;
    state->ss = nmea->rmc.ss;

    return true;
}

static void terminal_gps_status(int argc, const char **argv) {
    (void)argc;
    (void)argv;

    vcu_gps_public_state gps;
    if (!vcu_gps_get_state(&gps)) {
        commands_printf("GPS status unavailable.");
        return;
    }

    commands_printf(
            "VCU GPS status\n"
            "ublox ready : %d\n"
            "uart num    : %d\n"
            "uart rx     : %d\n"
            "uart tx     : %d\n"
            "gga count   : %d\n"
            "rmc count   : %d\n"
            "fix valid   : %d\n"
            "fix type    : %s (%d)\n"
            "satellites  : %d\n"
            "hdop        : %.2f\n"
            "lat         : %.8f\n"
            "lon         : %.8f\n"
            "height      : %.2f m\n"
            "speed       : %.2f m/s\n"
            "gga age     : %lu ms\n"
            "rmc age     : %lu ms\n"
            "tod valid   : %d\n"
            "date valid  : %d\n"
            "time valid  : %d\n"
            "time        : %02d-%02d-%02d %02d:%02d:%02d",
            gps.ublox_ready ? 1 : 0,
            UART_NUM,
            UART_RX,
            UART_TX,
            gps.gga_count,
            gps.rmc_count,
            gps.fix_valid ? 1 : 0,
            nmea_fix_type(),
            gps.fix_type,
            gps.sats,
            (double)gps.hdop,
            gps.lat,
            gps.lon,
            gps.height_m,
            (double)gps.speed_mps,
            (unsigned long)gps.gga_age_ms,
            (unsigned long)gps.rmc_age_ms,
            gps.tod_valid ? 1 : 0,
            gps.date_valid ? 1 : 0,
            gps.time_valid ? 1 : 0,
            gps.yy, gps.mo, gps.dd, gps.hh, gps.mm, gps.ss);
}

void vcu_gps_init(void) {
    terminal_register_command_callback(
            "gps_status",
            "Print VCU GPS/u-blox status.",
            0,
            terminal_gps_status);
}
