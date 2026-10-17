#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "vcu_log_sample.h"

int main(int argc, char **argv) {
    double v[VCU_LOG_FIELD_COUNT];
    vcu_stm32_gateway_state_t s = {0};
    vcu_log_sample_from_state(&s, 1000, v);
    assert(v[0] == 0 && v[19] == 0);
    assert(v[20] == -1 && v[21] == -1 && v[22] == -1);

    s.online = s.initialized = s.protocol_compatible = true;
    s.has_heartbeat = s.has_rc_input = s.has_targets = true;
    s.has_motor_left = s.has_motor_right = s.has_diagnostics = true;
    for (unsigned i = 0; i < VCU_STM32_GATEWAY_TELEMETRY_COUNT; i++)
        s.frame_last_rx_ms[i] = 950;
    s.heartbeat.state = 3; s.heartbeat.reason = 10; s.heartbeat.flags = 16;
    s.rc_input.flags = 1; s.rc_input.throttle_raw = 1500; s.rc_input.steering_raw = 992;
    s.targets.left_base_rpm = -110; s.targets.right_base_rpm = 220;
    s.targets.left_target_rpm = -100; s.targets.right_target_rpm = 200;
    s.motor_left.speed_rpm = -98; s.motor_right.speed_rpm = 199;
    s.motor_left.voltage_decivolts = 721; s.motor_right.voltage_decivolts = 719;
    s.motor_left.fault_code = 7; s.motor_right.fault_code = 9;
    s.motor_left.temperature_c = 45; s.motor_right.temperature_c = 46;
    s.motor_left.age_10ms = 3; s.motor_right.age_10ms = 255;
    s.diagnostics.can_rx_overflows = 17;
    vcu_log_sample_from_state(&s, 1000, v);
    assert(v[0] == 1 && v[1] == 1 && v[2] == 3 && v[3] == 10);
    assert(v[4] == 1500 && v[5] == 992);
    assert(v[6] == -110 && v[7] == 220 && v[8] == -100 && v[9] == 200);
    assert(v[10] == -98 && v[11] == 199 && v[12] == 7 && v[13] == 9);
    assert(fabsf(v[14] - 72.1f) < 0.001f && fabsf(v[15] - 71.9f) < 0.001f);
    assert(v[16] == 17 && v[17] == 45 && v[18] == 46);
    assert(v[19] == 63 && v[20] == 50 && v[21] == 80 && v[22] == 2600 && v[23] == 1);

    /* A fresh heartbeat must not hide stale/missing individual streams. */
    s.frame_last_rx_ms[1] = 499;
    s.has_motor_right = false;
    vcu_log_sample_from_state(&s, 1000, v);
    assert(v[1] == 0 && (unsigned)v[19] == (63U & ~2U & ~16U));
    assert(v[11] == 0 && v[22] == -1);
    s.frame_last_rx_ms[1] = 500;
    vcu_log_sample_from_state(&s, 1000, v);
    assert(v[1] == 1); /* 500 ms is the inclusive boundary. */

    /* Disconnects keep useful last values but explicitly invalidate them. */
    s.online = false;
    vcu_log_sample_from_state(&s, 1100, v);
    assert(v[0] == 0 && v[1] == 0 && v[19] == 0 && v[23] == 0);
    assert(v[10] == -98 && v[21] == 180);

    /* The 32-bit gateway timestamp wraps without false stale detection. */
    s.online = true; s.has_motor_right = true;
    for (unsigned i = 0; i < VCU_STM32_GATEWAY_TELEMETRY_COUNT; i++)
        s.frame_last_rx_ms[i] = UINT32_MAX - 49;
    vcu_log_sample_from_state(&s, 50, v);
    assert(v[19] == 63 && v[20] == 100 && v[21] == 130);

    assert(v[35] == -1 && v[47] == -1 && v[48] == 0);
    s.k2_diag[0].valid = true; s.k2_diag[1].valid = true;
    s.k2_diag[0].sample_ticks = UINT32_MAX;
    s.k2_diag[0].rx_ms = UINT32_MAX - 49;
    s.k2_diag[1].rx_ms = UINT32_MAX - 50;
    s.k2_diag[0].iq_reference_dA = -123;
    s.k2_diag[0].iq_actual_dA = -117;
    s.k2_diag[0].integral_dA = 87;
    s.k2_diag[0].state_flags = 65535;
    s.k2_diag[0].startup = 255;
    vcu_log_sample_from_state(&s, 50, v);
    assert(fabs(v[24] - 429496.7295) < .000001);
    assert(v[28] == -12.3 && v[29] == -11.7 && v[30] == 8.7);
    assert(v[32] == 16777215 && v[35] == 100 && v[47] == 101 && v[48] == 1);
    s.online = false; /* Direct K2 data and STM32 telemetry have separate validity. */
    vcu_log_sample_from_state(&s, 50, v);
    assert(v[19] == 0 && v[48] == 1);
    for (unsigned i = 0; i < VCU_LOG_CSV_FIELD_COUNT; i++) {
        assert(vcu_log_csv_fields[i] < VCU_LOG_FIELD_COUNT);
        for (unsigned j = 0; j < i; j++) assert(vcu_log_csv_fields[i] != vcu_log_csv_fields[j]);
    }

    for (unsigned i = 0; i < VCU_LOG_FIELD_COUNT; i++) {
        assert(strlen(vcu_log_fields[i].key) < 25);
        assert(strlen(vcu_log_fields[i].name) < 30);
        assert(strlen(vcu_log_fields[i].unit) < 10);
        for (unsigned j = 0; j < i; j++)
            assert(strcmp(vcu_log_fields[i].key, vcu_log_fields[j].key));
    }
    if (argc == 2) {
        FILE *f = fopen(argv[1], "w");
        assert(f);
        fprintf(f, "t_day:Time:s:3:0:1");
        for (unsigned i = 0; i < VCU_LOG_FIELD_COUNT; i++) {
            const vcu_log_field_t *h = &vcu_log_fields[i];
            fprintf(f, ";%s:%s:%s:%u:0:0", h->key, h->name, h->unit, h->precision);
        }
        fprintf(f, "\n0.050");
        for (unsigned i = 0; i < VCU_LOG_FIELD_COUNT; i++)
            fprintf(f, ";%.*f", (int)vcu_log_fields[i].precision, (double)v[i]);
        fprintf(f, "\n");
        assert(fclose(f) == 0);
    }
    puts("PASS: VCU logging values, units, dropout, freshness boundary, missing streams, tick wrap.");
    return 0;
}
