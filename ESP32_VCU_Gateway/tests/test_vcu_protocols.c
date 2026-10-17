#include "mx_can_protocol.h"
#include "vcu_crsf.h"
#include "vcu_gateway_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t crc8(const uint8_t *p, size_t n) {
    uint8_t c = 0;
    while (n--) { c ^= *p++; for (uint8_t b = 0; b < 8; b++) c = (c & 0x80U) ? (uint8_t)((c << 1U) ^ 0xD5U) : (uint8_t)(c << 1U); }
    return c;
}
static void pack(const uint16_t *ch, uint8_t *p) {
    memset(p, 0, 22); size_t n = 0;
    for (size_t i = 0; i < 16; i++) for (uint8_t b = 0; b < 11; b++, n++) if (ch[i] & (1U << b)) p[n / 8] |= (uint8_t)(1U << (n % 8));
}
int main(void) {
    const uint16_t ch[16] = {172,992,1811,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700};
    uint8_t frame[26] = {0xC8,24,0x16}; pack(ch, &frame[3]); frame[25] = crc8(&frame[2], 23);
    vcu_crsf_t c; vcu_crsf_init(&c, 992); vcu_crsf_feed(&c, frame, 7, 100); assert(!vcu_crsf_link_ok(&c, 100, 200));
    vcu_crsf_feed(&c, &frame[7], 19, 110); assert(vcu_crsf_link_ok(&c, 300, 200)); assert(!vcu_crsf_link_ok(&c, 311, 200));
    for (uint8_t i = 0; i < 16; i++) assert(vcu_crsf_channel(&c, i) == ch[i]);
    frame[25] ^= 1U; vcu_crsf_feed(&c, frame, 26, 400); assert(c.crc_error_count == 1);

    mx_can_command_t cmd = {MX_CAN_CONTROL_SPEED,MX_CAN_CONTROL_SPEED,true,true,false,false,50,50,100,100};
    const uint8_t expected[8] = {0xC3,0,0x32,0x64,0,0x32,0x64,0}; uint8_t data[8];
    assert(mx_can_pack_command(&cmd, data)); assert(memcmp(data, expected, 8) == 0);
    const uint8_t raw[8] = {5,42,0xFF,0x9C,0x04,0xD2,0x03,0xC0}; mx_can_status_t s;
    assert(mx_can_unpack_status(raw, &s)); assert(s.fault_code == 5 && s.axis2_speed_rpm == -100);
    assert(s.axis1_speed_rpm == 1234 && s.supply_voltage_decivolts == 960);

    const vcu_gateway_command_t gateway_command = {
        VCU_GATEWAY_COMMAND_RUN, 5U, 2550U, 0x42U
    };
    const uint8_t gateway_expected[8] = {
        1U, 1U, 0U, 5U, 0x09U, 0xF6U, 0x42U, 0U
    };
    vcu_gateway_command_t gateway_decoded;
    assert(vcu_gateway_pack_command(&gateway_command, data));
    assert(memcmp(data, gateway_expected, sizeof(data)) == 0);
    assert(vcu_gateway_unpack_command(data, &gateway_decoded));
    assert(gateway_decoded.type == VCU_GATEWAY_COMMAND_RUN);
    assert(gateway_decoded.period_ms == 5U);
    assert(gateway_decoded.acceleration_rpm_s == 2550U);
    assert(gateway_decoded.sequence == 0x42U);

    {
        const vcu_gateway_can_error_t can_error = {
            0xA5U,
            VCU_GATEWAY_CAN_ERROR_HAL_IRQ,
            3U | VCU_GATEWAY_CAN_ERROR_WARNING |
                    VCU_GATEWAY_CAN_ERROR_TX,
            2U,
            129U,
            3U,
            0x1234U
        };
        const uint8_t can_error_expected[8] = {
            0xA5U, 1U, 0x4BU, 2U, 129U, 3U, 0x12U, 0x34U
        };
        vcu_gateway_can_error_t can_error_decoded;

        vcu_gateway_pack_can_error(&can_error, data);
        assert(memcmp(data, can_error_expected, sizeof(data)) == 0);
        vcu_gateway_unpack_can_error(data, &can_error_decoded);
        assert(can_error_decoded.sequence == can_error.sequence);
        assert(can_error_decoded.type == can_error.type);
        assert(can_error_decoded.flags == can_error.flags);
        assert(can_error_decoded.hal_state == can_error.hal_state);
        assert(can_error_decoded.tx_error_count == can_error.tx_error_count);
        assert(can_error_decoded.rx_error_count == can_error.rx_error_count);
        assert(can_error_decoded.timestamp_10ms == can_error.timestamp_10ms);
    }

    puts("VCU protocol tests passed"); return 0;
}
