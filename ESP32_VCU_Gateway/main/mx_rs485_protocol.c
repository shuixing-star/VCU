/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#include "mx_rs485_protocol.h"

#include <string.h>

#define MX_RS485_FUNC_READ_HOLDING_REGS  0x03
#define MX_RS485_EXCEPTION_MASK          0x80

static uint16_t get_be_u16(const uint8_t *data) {
    return ((uint16_t)data[0] << 8) | data[1];
}

static int16_t get_be_i16(const uint8_t *data) {
    return (int16_t)get_be_u16(data);
}

static void put_be_u16(uint8_t *data, uint16_t value) {
    data[0] = value >> 8;
    data[1] = value & 0xFF;
}

uint16_t mx_rs485_crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0;i < len;i++) {
        crc ^= data[i];
        for (int bit = 0;bit < 8;bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

bool mx_rs485_build_read_request(uint8_t address, uint8_t *out, size_t out_len) {
    if (!out || out_len < MX_RS485_READ_REQUEST_LEN || address < 0x01 || address > 0x7F) {
        return false;
    }

    out[0] = address;
    out[1] = MX_RS485_FUNC_READ_HOLDING_REGS;
    put_be_u16(&out[2], MX_RS485_READ_START_REG);
    put_be_u16(&out[4], MX_RS485_READ_REG_COUNT);

    uint16_t crc = mx_rs485_crc16_modbus(out, MX_RS485_READ_REQUEST_LEN - 2);
    out[6] = crc & 0xFF;
    out[7] = crc >> 8;

    return true;
}

size_t mx_rs485_expected_response_len(const uint8_t *frame, size_t frame_len) {
    if (!frame || frame_len < 2) {
        return 0;
    }

    if (frame[1] & MX_RS485_EXCEPTION_MASK) {
        return 5;
    }

    if (frame_len < 3) {
        return 0;
    }

    return (size_t)frame[2] + 5;
}

mx_rs485_result mx_rs485_parse_read_response(
        uint8_t expected_address,
        const uint8_t *frame,
        size_t frame_len,
        mx_rs485_sample *sample) {
    if (!frame || !sample || frame_len < 5) {
        return MX_RS485_RESULT_BAD_LEN;
    }

    if (frame[0] != expected_address) {
        return MX_RS485_RESULT_BAD_ADDRESS;
    }

    uint16_t rx_crc = (uint16_t)frame[frame_len - 2] | ((uint16_t)frame[frame_len - 1] << 8);
    uint16_t calc_crc = mx_rs485_crc16_modbus(frame, frame_len - 2);
    if (rx_crc != calc_crc) {
        return MX_RS485_RESULT_BAD_CRC;
    }

    if (frame[1] == (MX_RS485_FUNC_READ_HOLDING_REGS | MX_RS485_EXCEPTION_MASK)) {
        return MX_RS485_RESULT_EXCEPTION;
    }

    if (frame[1] != MX_RS485_FUNC_READ_HOLDING_REGS) {
        return MX_RS485_RESULT_BAD_FUNCTION;
    }

    if (frame_len != MX_RS485_READ_RESPONSE_LEN) {
        return MX_RS485_RESULT_BAD_LEN;
    }

    if (frame[2] != MX_RS485_READ_RESPONSE_DATA_LEN) {
        return MX_RS485_RESULT_BAD_BYTE_COUNT;
    }

    memset(sample, 0, sizeof(*sample));
    sample->address = frame[0];

    const uint8_t *data = &frame[3];
    sample->fault_code = get_be_u16(&data[0]);
    sample->temperature_c = get_be_i16(&data[2]);
    sample->voltage_v = (float)get_be_i16(&data[4]) * 0.1;
    sample->motor2_current_a = (float)get_be_i16(&data[6]) * 0.1;
    sample->motor1_current_a = (float)get_be_i16(&data[8]) * 0.1;
    sample->motor2_speed_rpm = get_be_i16(&data[10]);
    sample->motor1_speed_rpm = get_be_i16(&data[12]);

    return MX_RS485_RESULT_OK;
}

const char *mx_rs485_result_name(mx_rs485_result result) {
    switch (result) {
    case MX_RS485_RESULT_OK: return "ok";
    case MX_RS485_RESULT_BAD_LEN: return "bad_len";
    case MX_RS485_RESULT_BAD_ADDRESS: return "bad_address";
    case MX_RS485_RESULT_BAD_FUNCTION: return "bad_function";
    case MX_RS485_RESULT_EXCEPTION: return "exception";
    case MX_RS485_RESULT_BAD_BYTE_COUNT: return "bad_byte_count";
    case MX_RS485_RESULT_BAD_CRC: return "bad_crc";
    default: return "unknown";
    }
}
