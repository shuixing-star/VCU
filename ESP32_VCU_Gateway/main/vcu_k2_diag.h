#ifndef VCU_K2_DIAG_H
#define VCU_K2_DIAG_H
#include <stdbool.h>
#include <stdint.h>
#define VCU_K2_DIAG_TIMEOUT_MS 100U
typedef struct {
    bool valid, hall_log_valid, window_present;
    uint32_t window_end_ticks, window_rx_ms, window_diag_ticks;
    uint16_t window_count, window_flags, window_schema, q_clip_count, v_clip_count;
    int16_t iq_ref_mean_dA, iq_mean_dA, id_ref_mean_dA, id_mean_dA, iq_mae_dA;
    int32_t hall_raw_drpm, hall_avg_drpm, hall_filtered_drpm;
    uint32_t hall_period_ticks, hall_age_ticks, hall_sample_ticks, hall_rx_ms;
    uint16_t hall_flags, hall_pole_pairs_centi;
    uint16_t token;
    uint32_t rx_ms, sample_ticks; /* ESP first-part RX time, K2 nominal 10 kHz clock */
    int16_t target_rpm, reference_rpm, actual_rpm;
    int16_t iq_reference_dA, iq_actual_dA, integral_dA;
    uint16_t voltage_dV, limit_flags, state_flags;
    uint8_t fault, startup;
} vcu_k2_diag_t;
typedef struct {
    bool active, has_completed;
    uint16_t token, last_completed_token;
    uint32_t first_rx_ms;
    uint16_t mask;
    uint8_t completed_groups, bytes[12][8];
} vcu_k2_diag_assembler_t;
bool vcu_k2_diag_route(uint32_t id, bool extended, unsigned *side, unsigned *part);
bool vcu_k2_diag_accept(vcu_k2_diag_assembler_t *a, vcu_k2_diag_t *out,
                       unsigned part, const uint8_t *data, uint8_t len, uint32_t now_ms);
#endif
