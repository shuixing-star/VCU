#include "vcu_log_sample.h"
#include <math.h>

const vcu_log_field_t vcu_log_fields[VCU_LOG_FIELD_COUNT] = {
    {"stm32_online", "STM32 online", "", 0},
    {"crsf_link", "RC link", "", 0},
    {"drive_state", "Drive state", "", 0},
    {"drive_reason", "Drive reason", "", 0},
    {"throttle_raw", "Throttle raw", "", 0},
    {"steering_raw", "Steering raw", "", 0},
    {"left_base_rpm", "Left base RPM", "rpm", 0},
    {"right_base_rpm", "Right base RPM", "rpm", 0},
    {"left_target_rpm", "Left target RPM", "rpm", 0},
    {"right_target_rpm", "Right target RPM", "rpm", 0},
    {"left_speed_rpm", "Left measured RPM", "rpm", 0},
    {"right_speed_rpm", "Right measured RPM", "rpm", 0},
    {"left_fault", "Left fault", "", 0},
    {"right_fault", "Right fault", "", 0},
    {"left_voltage_v", "Left voltage", "V", 1},
    {"right_voltage_v", "Right voltage", "V", 1},
    {"can_rx_overflows", "CAN RX overflows", "", 0},
    {"left_temp_c", "Left temperature", "degC", 0},
    {"right_temp_c", "Right temperature", "degC", 0},
    {"fresh_mask", "Fresh telemetry mask", "", 0},
    {"heartbeat_age_ms", "Heartbeat age", "ms", 0},
    {"left_age_ms", "Left feedback age", "ms", 0},
    {"right_age_ms", "Right feedback age", "ms", 0},
    {"output_enabled", "Output enabled", "", 0},
    {"left_sample_s", "Left K2 time", "s", 4},
    {"left_effective_rpm", "Left Accepted target", "rpm", 0},
    {"left_reference_rpm", "Left Speed reference", "rpm", 0},
    {"left_actual_rpm", "Left Actual speed", "rpm", 0},
    {"left_iq_ref_a", "Left Iq reference", "A", 1},
    {"left_iq_actual_a", "Left Iq actual", "A", 1},
    {"left_speed_integral_a", "Left Speed integral", "A", 1},
    {"left_limit_flags", "Left Limit flags", "", 0},
    {"left_control_flags", "Left Control flags", "", 0},
    {"left_motor_fault", "Left Raw K2 fault", "", 0},
    {"left_vbus_v", "Left K2 bus voltage", "V", 1},
    {"left_diag_age_ms", "Left K2 RX age", "ms", 0},
    {"right_sample_s", "Right K2 time", "s", 4},
    {"right_effective_rpm", "Right Accepted target", "rpm", 0},
    {"right_reference_rpm", "Right Speed reference", "rpm", 0},
    {"right_actual_rpm", "Right Actual speed", "rpm", 0},
    {"right_iq_ref_a", "Right Iq reference", "A", 1},
    {"right_iq_actual_a", "Right Iq actual", "A", 1},
    {"right_speed_integral_a", "Right Speed integral", "A", 1},
    {"right_limit_flags", "Right Limit flags", "", 0},
    {"right_control_flags", "Right Control flags", "", 0},
    {"right_motor_fault", "Right Raw K2 fault", "", 0},
    {"right_vbus_v", "Right K2 bus voltage", "V", 1},
    {"right_diag_age_ms", "Right K2 RX age", "ms", 0},
    {"diag_fresh_mask", "Fresh K2 snapshot mask", "", 0},
    {"target_age_ms", "VCU target RX age", "ms", 0},
    {"left_hall_raw_rpm", "Left Hall raw", "rpm", 1},
    {"left_hall_avg_rpm", "Left Hall average", "rpm", 1},
    {"left_hall_filtered_rpm", "Left Hall filtered", "rpm", 1},
    {"left_hall_period_ms", "Left Hall period", "ms", 1},
    {"left_hall_edge_age_ms", "Left Hall edge age", "ms", 1},
    {"left_hall_flags", "Left Hall flags", "", 0},
    {"left_hall_sample_s", "Left Hall sample", "s", 4},
    {"left_hall_rx_age_ms", "Left Hall RX age", "ms", 0},
    {"left_hall_fresh", "Left Hall fresh", "", 0},
    {"left_hall_pole_pairs", "Left Pole pairs", "", 2},
    {"right_hall_raw_rpm", "Right Hall raw", "rpm", 1},
    {"right_hall_avg_rpm", "Right Hall average", "rpm", 1},
    {"right_hall_filtered_rpm", "Right Hall filtered", "rpm", 1},
    {"right_hall_period_ms", "Right Hall period", "ms", 1},
    {"right_hall_edge_age_ms", "Right Hall edge age", "ms", 1},
    {"right_hall_flags", "Right Hall flags", "", 0},
    {"right_hall_sample_s", "Right Hall sample", "s", 4},
    {"right_hall_rx_age_ms", "Right Hall RX age", "ms", 0},
    {"right_hall_fresh", "Right Hall fresh", "", 0},
    {"right_hall_pole_pairs", "Right Pole pairs", "", 2},
    {"left_win_end_s", "Left Window end", "s", 4},
    {"left_win_n", "Left Window samples", "", 0},
    {"left_win_flags", "Left Window flags", "", 0},
    {"left_iq_ref_mean_a", "Left Iq ref mean", "A", 1},
    {"left_iq_mean_a", "Left Iq mean", "A", 1},
    {"left_id_ref_mean_a", "Left Id ref mean", "A", 1},
    {"left_id_mean_a", "Left Id mean", "A", 1},
    {"left_iq_error_ma_a", "Left Iq mean abs error", "A", 1},
    {"left_q_clip_pct", "Left Q voltage clip", "%", 1},
    {"left_v_clip_pct", "Left Vector voltage clip", "%", 1},
    {"left_win_age_ms", "Left Window age", "ms", 1},
    {"left_win_fresh", "Left Window fresh", "", 0},
    {"right_win_end_s", "Right Window end", "s", 4},
    {"right_win_n", "Right Window samples", "", 0},
    {"right_win_flags", "Right Window flags", "", 0},
    {"right_iq_ref_mean_a", "Right Iq ref mean", "A", 1},
    {"right_iq_mean_a", "Right Iq mean", "A", 1},
    {"right_id_ref_mean_a", "Right Id ref mean", "A", 1},
    {"right_id_mean_a", "Right Id mean", "A", 1},
    {"right_iq_error_ma_a", "Right Iq mean abs error", "A", 1},
    {"right_q_clip_pct", "Right Q voltage clip", "%", 1},
    {"right_v_clip_pct", "Right Vector voltage clip", "%", 1},
    {"right_win_age_ms", "Right Window age", "ms", 1},
    {"right_win_fresh", "Right Window fresh", "", 0},
};

/* SD schema: original VCU context plus coherent per-controller snapshots. */
const unsigned int vcu_log_csv_fields[VCU_LOG_CSV_FIELD_COUNT] = {
    2, 3, 8, 9, 23, 19, 24, 25, 26, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93
};

void vcu_log_sample_from_state(const vcu_stm32_gateway_state_t *s,
                              uint32_t now_ms,
                              double v[VCU_LOG_FIELD_COUNT]) {
    const bool present[VCU_STM32_GATEWAY_TELEMETRY_COUNT] = {
        s->has_heartbeat, s->has_rc_input, s->has_targets,
        s->has_motor_left, s->has_motor_right, s->has_diagnostics
    };
    uint32_t fresh_mask = 0U;
    for (unsigned int i = 0; i < VCU_STM32_GATEWAY_TELEMETRY_COUNT; i++) {
        if (s->online && present[i] &&
                (uint32_t)(now_ms - s->frame_last_rx_ms[i]) <=
                    VCU_STM32_GATEWAY_ONLINE_TIMEOUT_MS) {
            fresh_mask |= 1U << i;
        }
    }
    v[0] = s->online;
    v[1] = (fresh_mask & (1U << VCU_STM32_GATEWAY_TELEMETRY_RC_INPUT)) &&
            (s->rc_input.flags & 1U);
    v[2] = s->has_heartbeat ? s->heartbeat.state : 0;
    v[3] = s->has_heartbeat ? s->heartbeat.reason : 0;
    v[4] = s->has_rc_input ? s->rc_input.throttle_raw : 0;
    v[5] = s->has_rc_input ? s->rc_input.steering_raw : 0;
    v[6] = s->has_targets ? s->targets.left_base_rpm : 0;
    v[7] = s->has_targets ? s->targets.right_base_rpm : 0;
    v[8] = s->has_targets ? s->targets.left_target_rpm : 0;
    v[9] = s->has_targets ? s->targets.right_target_rpm : 0;
    v[10] = s->has_motor_left ? s->motor_left.speed_rpm : 0;
    v[11] = s->has_motor_right ? s->motor_right.speed_rpm : 0;
    v[12] = s->has_motor_left ? s->motor_left.fault_code : 0;
    v[13] = s->has_motor_right ? s->motor_right.fault_code : 0;
    v[14] = s->has_motor_left ? s->motor_left.voltage_decivolts / 10.0f : 0;
    v[15] = s->has_motor_right ? s->motor_right.voltage_decivolts / 10.0f : 0;
    v[16] = s->has_diagnostics ? s->diagnostics.can_rx_overflows : 0;
    v[17] = s->has_motor_left ? s->motor_left.temperature_c : 0;
    v[18] = s->has_motor_right ? s->motor_right.temperature_c : 0;
    v[19] = fresh_mask;
    v[20] = s->has_heartbeat ?
            (float)(uint32_t)(now_ms - s->frame_last_rx_ms[0]) : -1.0f;
    v[21] = s->has_motor_left ?
            (float)(uint32_t)(now_ms - s->frame_last_rx_ms[3]) +
                s->motor_left.age_10ms * 10.0f : -1.0f;
    v[22] = s->has_motor_right ?
            (float)(uint32_t)(now_ms - s->frame_last_rx_ms[4]) +
                s->motor_right.age_10ms * 10.0f : -1.0f;
    v[23] = s->online && s->has_heartbeat &&
            (s->heartbeat.flags & (1U << 4));
    v[48] = 0;
    for (unsigned int side = 0; side < 2; side++) {
        const vcu_k2_diag_t *d = &s->k2_diag[side];
        const unsigned int base = 24U + side * 12U;
        for (unsigned int i = 0; i < 12U; i++) v[base+i] = 0;
        v[base+11] = -1;
        if (!d->valid) continue; /* Zero placeholders; invalid mask and age=-1. */
        const uint32_t age = now_ms - d->rx_ms;
        v[base] = d->sample_ticks / 10000.0;
        v[base+1] = d->target_rpm; v[base+2] = d->reference_rpm;
        v[base+3] = d->actual_rpm;
        v[base+4] = d->iq_reference_dA / 10.0;
        v[base+5] = d->iq_actual_dA / 10.0;
        v[base+6] = d->integral_dA / 10.0;
        v[base+7] = d->limit_flags;
        v[base+8] = (uint32_t)d->state_flags | ((uint32_t)d->startup << 16);
        v[base+9] = d->fault; v[base+10] = d->voltage_dV / 10.0;
        v[base+11] = age;
        /* K2 freshness is independent of STM32 heartbeat; each source is explicit. */
        if (age <= VCU_K2_DIAG_TIMEOUT_MS) v[48] += (1U << side);
    }
    for (unsigned side=0;side<2;side++) {
        const vcu_k2_diag_t *d=&s->k2_diag[side];
        unsigned b=50U+side*10U;
        for(unsigned j=0;j<10;j++)v[b+j]=NAN;
        v[b+7]=-1;v[b+8]=0;
        if (!d->hall_log_valid) continue;
        uint32_t age=now_ms-d->hall_rx_ms;
        if (!(d->hall_flags&128U)) {
            if (d->hall_flags&2U)v[b]=d->hall_raw_drpm/10.0;
            if (d->hall_flags&4U)v[b+1]=d->hall_avg_drpm/10.0;
            v[b+2]=d->hall_filtered_drpm/10.0;
        }
        v[b+3]=d->hall_period_ticks/10.0;v[b+4]=d->hall_age_ticks/10.0;
        v[b+5]=d->hall_flags;v[b+6]=d->hall_sample_ticks/10000.0;
        v[b+7]=age;v[b+8]=age<=VCU_K2_DIAG_TIMEOUT_MS;
        v[b+9]=d->hall_pole_pairs_centi/100.0;
    }
    for (unsigned side=0;side<2;side++) {
        const vcu_k2_diag_t *d=&s->k2_diag[side];unsigned b=70U+side*12U;
        for(unsigned j=0;j<12;j++)v[b+j]=NAN;
        v[b+10]=-1;v[b+11]=0;
        if (!d->window_present) continue;
        uint32_t ticks=d->window_diag_ticks-d->window_end_ticks;
        double age=(uint32_t)(now_ms-d->window_rx_ms)+ticks/10.0;
        v[b]=d->window_end_ticks/10000.0;v[b+1]=d->window_count;v[b+2]=d->window_flags;
        v[b+10]=age;
        bool valid=d->window_schema==1U && d->window_count==200U &&
            (d->window_flags&1U) && !(d->window_flags&2U) &&
            d->q_clip_count<=d->window_count && d->v_clip_count<=d->window_count && d->iq_mae_dA>=0;
        bool fresh=valid && !(d->window_flags&4U) && age<=VCU_K2_DIAG_TIMEOUT_MS &&
            (uint32_t)(d->sample_ticks-d->window_end_ticks)<=1000U;
        v[b+11]=fresh;
        if (!valid) continue;
        v[b+3]=d->iq_ref_mean_dA/10.0;v[b+4]=d->iq_mean_dA/10.0;
        v[b+5]=d->id_ref_mean_dA/10.0;v[b+6]=d->id_mean_dA/10.0;v[b+7]=d->iq_mae_dA/10.0;
        v[b+8]=100.0*d->q_clip_count/d->window_count;
        v[b+9]=100.0*d->v_clip_count/d->window_count;
    }
    v[49] = s->has_targets ?
            (double)(uint32_t)(now_ms - s->frame_last_rx_ms[2]) : -1.0;
}
