#ifndef VCU_SPEED_SYNC_H
#define VCU_SPEED_SYNC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    VCU_SYNC_DIAG_NONE = 0,
    VCU_SYNC_DIAG_DISABLED = 1U << 0,
    VCU_SYNC_DIAG_INVALID_CONFIG = 1U << 1,
    VCU_SYNC_DIAG_INVALID_DT = 1U << 2,
    VCU_SYNC_DIAG_LEFT_CAN_INVALID = 1U << 3,
    VCU_SYNC_DIAG_RIGHT_CAN_INVALID = 1U << 4,
    VCU_SYNC_DIAG_LEFT_FAULT = 1U << 5,
    VCU_SYNC_DIAG_RIGHT_FAULT = 1U << 6,
    VCU_SYNC_DIAG_DIRECTION_UNSUPPORTED = 1U << 7,
} vcu_speed_sync_diag_t;

typedef struct {
    float kp;
    int32_t error_deadzone_rpm;
    int32_t low_limit_rpm;
    int32_t medium_limit_rpm;
    int32_t high_limit_rpm;
    int32_t medium_speed_threshold_rpm;
    int32_t high_speed_threshold_rpm;
    int32_t weak_field_threshold_rpm;
    int32_t large_error_threshold_rpm;
    uint32_t large_error_hold_ms;
    int32_t longitudinal_derate_step_rpm;
    int32_t minimum_longitudinal_target_rpm;
    uint32_t max_control_period_ms;
} vcu_speed_sync_config_t;

typedef struct {
    uint32_t large_error_elapsed_ms;
    int32_t accumulated_derate_rpm;
} vcu_speed_sync_state_t;

typedef struct {
    bool enabled;
    bool left_can_valid;
    bool right_can_valid;
    bool left_fault;
    bool right_fault;
    float dt_seconds;
    int32_t left_actual_rpm;
    int32_t right_actual_rpm;
    int32_t left_theoretical_target_rpm;
    int32_t right_theoretical_target_rpm;
} vcu_speed_sync_input_t;

typedef struct {
    uint32_t diagnostic_flags;
    bool sync_active;
    bool weak_field;
    int32_t average_rpm;
    int32_t longitudinal_target_rpm;
    int32_t derated_longitudinal_target_rpm;
    int32_t left_derated_target_rpm;
    int32_t right_derated_target_rpm;
    int32_t theoretical_difference_rpm;
    int32_t actual_difference_rpm;
    int32_t raw_error_rpm;
    int32_t effective_error_rpm;
    float raw_correction_rpm;
    int32_t active_limit_rpm;
    int32_t limited_correction_rpm;
    int32_t left_target_rpm;
    int32_t right_target_rpm;
    uint32_t large_error_elapsed_ms;
    int32_t accumulated_derate_rpm;
} vcu_speed_sync_output_t;

void vcu_speed_sync_default_config(vcu_speed_sync_config_t *config);
void vcu_speed_sync_reset(vcu_speed_sync_state_t *state);
bool vcu_speed_sync_step(vcu_speed_sync_state_t *state,
        const vcu_speed_sync_config_t *config,
        const vcu_speed_sync_input_t *input,
        vcu_speed_sync_output_t *output);

#endif
