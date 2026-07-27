#ifndef VCU_DRIVE_RUNTIME_H
#define VCU_DRIVE_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    VCU_DRIVE_STATE_STOPPED = 0,
    VCU_DRIVE_STATE_DISARMED,
    VCU_DRIVE_STATE_ARMING_NEUTRAL,
    VCU_DRIVE_STATE_ARMED,
    VCU_DRIVE_STATE_FAULT
} vcu_drive_state_t;

typedef struct {
    bool initialized;
    bool task_running;
    bool requested_run;
    vcu_drive_state_t state;
    uint32_t period_ms;
    int16_t max_target;
    uint32_t accel_rpm_s;
    uint8_t can_ramp;
    int16_t target_deadzone_rpm;
    int16_t latest_left_target;
    int16_t latest_right_target;
    int16_t latest_left_base_target;
    int16_t latest_right_base_target;
    int16_t left_pre_slew_target;
    int16_t right_pre_slew_target;
    bool sync_enabled;
    float sync_kp;
    int32_t sync_deadzone_rpm;
    int32_t sync_low_limit_rpm;
    int32_t sync_medium_limit_rpm;
    int32_t sync_high_limit_rpm;
    int32_t sync_medium_threshold_rpm;
    int32_t sync_high_threshold_rpm;
    int32_t sync_weak_threshold_rpm;
    int32_t sync_target_slew_rpm_s;
    int32_t sync_large_error_threshold_rpm;
    uint32_t sync_large_error_hold_ms;
    int32_t sync_derate_step_rpm;
    int32_t sync_minimum_target_rpm;
    uint32_t sync_diagnostic_flags;
    bool sync_active;
    bool weak_field;
    bool left_can_valid;
    bool right_can_valid;
    int32_t average_rpm;
    int32_t longitudinal_target_rpm;
    int32_t derated_longitudinal_target_rpm;
    int32_t theoretical_difference_rpm;
    int32_t actual_difference_rpm;
    int32_t sync_raw_error_rpm;
    int32_t sync_effective_error_rpm;
    float sync_raw_correction_rpm;
    int32_t sync_active_limit_rpm;
    int32_t sync_limited_correction_rpm;
    uint32_t sync_large_error_elapsed_ms;
    int32_t sync_accumulated_derate_rpm;
    bool reset_requested;
    uint32_t loop_count;
    uint32_t tx_count;
    uint32_t zero_tx_count;
    uint32_t reset_tx_count;
    char reason[64];
} vcu_drive_runtime_state_t;

void vcu_drive_runtime_init(void);
bool vcu_drive_runtime_get_state(vcu_drive_runtime_state_t *state);
const char *vcu_drive_runtime_state_name(vcu_drive_state_t state);

#endif
