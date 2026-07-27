/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#ifndef MX_CAN485_H_
#define MX_CAN485_H_

#include <stdbool.h>
#include <stdint.h>

#include "mx_rs485_protocol.h"

#define MX_CAN485_APP_PACKET_STATUS      1
#define MX_CAN485_APP_PACKET_VERSION     1

typedef struct {
    uint32_t tx_id;
    uint32_t rx_id;
    bool use_ext_id;
    uint8_t mx_address;
    mx_rs485_result last_result;
    mx_rs485_sample sample;
    uint32_t rx_frames;
    uint32_t rx_ok;
    uint32_t rx_bad;
    uint32_t rx_overflow;
    uint32_t rx_timeout;
    uint32_t tx_count;
    uint32_t stream_tx;
    bool has_sample;
    bool online;
    bool pending_response;
    bool auto_poll_enabled;
    bool stream_enabled;
    uint32_t period_ms;
    uint32_t stream_period_ms;
} mx_can485_public_state;

void mx_can485_init(void);
bool mx_can485_process_can_frame(uint32_t id, const uint8_t *data, uint8_t len, bool is_ext);
bool mx_can485_get_state(mx_can485_public_state *state);
bool mx_can485_start_poll(uint32_t period_ms);
void mx_can485_stop_poll(void);
bool mx_can485_start_stream(uint32_t hz);
void mx_can485_stop_stream(void);

#endif /* MX_CAN485_H_ */
