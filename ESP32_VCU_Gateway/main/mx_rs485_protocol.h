/*
    Copyright 2026

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#ifndef MX_RS485_PROTOCOL_H_
#define MX_RS485_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MX_RS485_READ_START_REG          0x0000
#define MX_RS485_READ_REG_COUNT          7
#define MX_RS485_READ_REQUEST_LEN        8
#define MX_RS485_READ_RESPONSE_DATA_LEN  14
#define MX_RS485_READ_RESPONSE_LEN       19

typedef enum {
    MX_RS485_RESULT_OK = 0,
    MX_RS485_RESULT_BAD_LEN,
    MX_RS485_RESULT_BAD_ADDRESS,
    MX_RS485_RESULT_BAD_FUNCTION,
    MX_RS485_RESULT_EXCEPTION,
    MX_RS485_RESULT_BAD_BYTE_COUNT,
    MX_RS485_RESULT_BAD_CRC
} mx_rs485_result;

typedef struct {
    uint8_t address;
    uint16_t fault_code;
    int16_t temperature_c;
    float voltage_v;
    float motor2_current_a;
    float motor1_current_a;
    int16_t motor2_speed_rpm;
    int16_t motor1_speed_rpm;
} mx_rs485_sample;

uint16_t mx_rs485_crc16_modbus(const uint8_t *data, size_t len);
bool mx_rs485_build_read_request(uint8_t address, uint8_t *out, size_t out_len);
size_t mx_rs485_expected_response_len(const uint8_t *frame, size_t frame_len);
mx_rs485_result mx_rs485_parse_read_response(
        uint8_t expected_address,
        const uint8_t *frame,
        size_t frame_len,
        mx_rs485_sample *sample);
const char *mx_rs485_result_name(mx_rs485_result result);

#endif /* MX_RS485_PROTOCOL_H_ */
