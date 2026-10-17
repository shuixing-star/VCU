#include "vcu_k2_diag.h"
#include <string.h>
static uint16_t read16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
bool vcu_k2_diag_route(uint32_t id, bool extended, unsigned *side, unsigned *part) {
    if (!extended) return false;
    for (unsigned p = 0; p < 12; p++) {
        for (unsigned s = 0; s < 2; s++) {
            if (id == (0x18A0D000UL | ((uint32_t)p << 16) | (239U + s))) {
                *side = s; *part = p; return true;
            }
        }
    }
    return false;
}
bool vcu_k2_diag_accept(vcu_k2_diag_assembler_t *a, vcu_k2_diag_t *out,
                       unsigned part, const uint8_t *data, uint8_t len, uint32_t now_ms) {
    if (!data || len != 8U || part >= 12U) return false;
    const uint16_t token = read16(data);
    /* Replayed parts must not refresh the age of the last good snapshot. */
    if (a->has_completed && token == a->last_completed_token &&
        (!a->active || (a->mask & (1U << part)))) return false;
    if (!a->active || (uint32_t)(now_ms - a->first_rx_ms) > VCU_K2_DIAG_TIMEOUT_MS) {
        a->active = true; a->token = token; a->first_rx_ms = now_ms; a->mask = 0; a->completed_groups = 0;
    } else if (token != a->token) {
        if ((uint16_t)(token - a->token) >= 0x8000U) return false;
        a->token = token; a->first_rx_ms = now_ms; a->mask = 0; a->completed_groups = 0;
    }
    if (a->mask & (1U << part)) return false;
    memcpy(a->bytes[part], data, 8);
    a->mask |= (uint16_t)(1U << part);
    if ((a->mask & 15U) != 15U) return false;
    bool changed = false;
    if ((a->mask & 255U) == 255U && !(a->completed_groups & 2U)) {
        uint16_t w[12];
        for (unsigned p=0;p<4;p++) for(unsigned j=0;j<3;j++)
            w[p*3+j]=read16(a->bytes[p+4]+2+j*2);
        out->hall_raw_drpm=(int32_t)((uint32_t)w[0]|((uint32_t)w[1]<<16));
        out->hall_avg_drpm=(int32_t)((uint32_t)w[2]|((uint32_t)w[3]<<16));
        out->hall_filtered_drpm=(int32_t)((uint32_t)w[4]|((uint32_t)w[5]<<16));
        out->hall_period_ticks=(uint32_t)w[6]|((uint32_t)w[7]<<16);
        out->hall_age_ticks=(uint32_t)w[8]|((uint32_t)w[9]<<16);
        out->hall_flags=w[10];out->hall_pole_pairs_centi=w[11];
        out->hall_sample_ticks=(uint32_t)read16(a->bytes[3]+2)|((uint32_t)read16(a->bytes[3]+4)<<16);
        out->hall_rx_ms=a->first_rx_ms;out->hall_log_valid=true;
        a->completed_groups |= 2U; changed = true;
    }
    if ((a->mask & 0xF0FU) == 0xF0FU && !(a->completed_groups & 4U)) {
        uint16_t w[12];
        for (unsigned p=0;p<4;p++) for(unsigned j=0;j<3;j++)
            w[p*3+j]=read16(a->bytes[p+8]+2+j*2);
        out->window_end_ticks=(uint32_t)w[0]|((uint32_t)w[1]<<16);
        out->window_count=w[2];out->window_flags=w[3];
        out->iq_ref_mean_dA=(int16_t)w[4];out->iq_mean_dA=(int16_t)w[5];
        out->id_ref_mean_dA=(int16_t)w[6];out->id_mean_dA=(int16_t)w[7];
        out->iq_mae_dA=(int16_t)w[8];out->q_clip_count=w[9];out->v_clip_count=w[10];
        out->window_schema=w[11];out->window_rx_ms=a->first_rx_ms;
        out->window_diag_ticks=(uint32_t)read16(a->bytes[3]+2)|((uint32_t)read16(a->bytes[3]+4)<<16);
        out->window_present=true;
        a->completed_groups |= 4U; changed = true;
    }
    if (a->completed_groups & 1U) return changed;
    a->completed_groups |= 1U;
    out->target_rpm = (int16_t)read16(a->bytes[0] + 2);
    out->reference_rpm = (int16_t)read16(a->bytes[0] + 4);
    out->actual_rpm = (int16_t)read16(a->bytes[0] + 6);
    out->iq_reference_dA = (int16_t)read16(a->bytes[1] + 2);
    out->iq_actual_dA = (int16_t)read16(a->bytes[1] + 4);
    out->integral_dA = (int16_t)read16(a->bytes[1] + 6);
    out->voltage_dV = read16(a->bytes[2] + 2);
    out->limit_flags = read16(a->bytes[2] + 4);
    out->state_flags = read16(a->bytes[2] + 6);
    out->sample_ticks = (uint32_t)read16(a->bytes[3] + 2) |
                       ((uint32_t)read16(a->bytes[3] + 4) << 16);
    out->fault = a->bytes[3][6]; out->startup = a->bytes[3][7];
    out->token = token; out->rx_ms = a->first_rx_ms; out->valid = true;
    a->last_completed_token = token; a->has_completed = true;
    return true;
}
