/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#ifndef VCU_GPS_H_
#define VCU_GPS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool ublox_ready;
    bool has_gga;
    bool has_rmc;
    bool fix_valid;
    bool tod_valid;
    bool date_valid;
    bool time_valid;
    uint32_t gga_age_ms;
    uint32_t rmc_age_ms;
    int gga_count;
    int rmc_count;
    int fix_type;
    int sats;
    float hdop;
    double lat;
    double lon;
    double height_m;
    float speed_mps;
    int yy;
    int mo;
    int dd;
    int hh;
    int mm;
    int ss;
} vcu_gps_public_state;

void vcu_gps_init(void);
bool vcu_gps_get_state(vcu_gps_public_state *state);

#endif /* VCU_GPS_H_ */
