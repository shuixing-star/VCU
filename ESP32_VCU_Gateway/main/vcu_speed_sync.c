#include "vcu_speed_sync.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static int32_t abs_i32(int32_t value) {
    if (value == INT32_MIN) return INT32_MAX;
    return value < 0 ? -value : value;
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t round_i32(float value) {
    if (value >= (float)INT32_MAX) return INT32_MAX;
    if (value <= (float)INT32_MIN) return INT32_MIN;
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static int32_t reduce_magnitude(int32_t target, int32_t reduction) {
    const int32_t magnitude = abs_i32(target);
    const int32_t reduced = magnitude > reduction ? magnitude - reduction : 0;
    return target < 0 ? -reduced : reduced;
}

static bool config_valid(const vcu_speed_sync_config_t *config) {
    return config != NULL && isfinite(config->kp) && config->kp >= 0.0f &&
            config->error_deadzone_rpm >= 0 &&
            config->low_limit_rpm > 0 && config->medium_limit_rpm > 0 &&
            config->high_limit_rpm > 0 &&
            config->medium_speed_threshold_rpm > 0 &&
            config->high_speed_threshold_rpm > config->medium_speed_threshold_rpm &&
            config->weak_field_threshold_rpm > 0 &&
            config->large_error_threshold_rpm > 0 &&
            config->large_error_hold_ms > 0U &&
            config->longitudinal_derate_step_rpm > 0 &&
            config->minimum_longitudinal_target_rpm >= 0 &&
            config->max_control_period_ms > 0U;
}

void vcu_speed_sync_default_config(vcu_speed_sync_config_t *config) {
    if (config == NULL) return;
    *config = (vcu_speed_sync_config_t) {
        .kp = 0.20f,
        .error_deadzone_rpm = 40,
        .low_limit_rpm = 150,
        .medium_limit_rpm = 200,
        .high_limit_rpm = 250,
        .medium_speed_threshold_rpm = 4000,
        .high_speed_threshold_rpm = 4400,
        .weak_field_threshold_rpm = 4000,
        .large_error_threshold_rpm = 250,
        .large_error_hold_ms = 800U,
        .longitudinal_derate_step_rpm = 100,
        .minimum_longitudinal_target_rpm = 200,
        .max_control_period_ms = 100U,
    };
}

void vcu_speed_sync_reset(vcu_speed_sync_state_t *state) {
    if (state != NULL) memset(state, 0, sizeof(*state));
}

bool vcu_speed_sync_step(vcu_speed_sync_state_t *state,
        const vcu_speed_sync_config_t *config,
        const vcu_speed_sync_input_t *input,
        vcu_speed_sync_output_t *output) {
    if (state == NULL || input == NULL || output == NULL) return false;

    memset(output, 0, sizeof(*output));
    output->left_target_rpm = input->left_theoretical_target_rpm;
    output->right_target_rpm = input->right_theoretical_target_rpm;
    output->left_derated_target_rpm = input->left_theoretical_target_rpm;
    output->right_derated_target_rpm = input->right_theoretical_target_rpm;

    if (!config_valid(config)) output->diagnostic_flags |= VCU_SYNC_DIAG_INVALID_CONFIG;
    if (!isfinite(input->dt_seconds) || input->dt_seconds <= 0.0f ||
            (config_valid(config) && input->dt_seconds * 1000.0f > (float)config->max_control_period_ms)) {
        output->diagnostic_flags |= VCU_SYNC_DIAG_INVALID_DT;
    }
    if (!input->enabled) output->diagnostic_flags |= VCU_SYNC_DIAG_DISABLED;
    if (!input->left_can_valid) output->diagnostic_flags |= VCU_SYNC_DIAG_LEFT_CAN_INVALID;
    if (!input->right_can_valid) output->diagnostic_flags |= VCU_SYNC_DIAG_RIGHT_CAN_INVALID;
    if (input->left_fault) output->diagnostic_flags |= VCU_SYNC_DIAG_LEFT_FAULT;
    if (input->right_fault) output->diagnostic_flags |= VCU_SYNC_DIAG_RIGHT_FAULT;

    const uint32_t fatal_flags = VCU_SYNC_DIAG_INVALID_CONFIG | VCU_SYNC_DIAG_INVALID_DT |
            VCU_SYNC_DIAG_LEFT_CAN_INVALID | VCU_SYNC_DIAG_RIGHT_CAN_INVALID |
            VCU_SYNC_DIAG_LEFT_FAULT | VCU_SYNC_DIAG_RIGHT_FAULT;
    if (!input->enabled || (output->diagnostic_flags & fatal_flags) != 0U) {
        vcu_speed_sync_reset(state);
        output->large_error_elapsed_ms = state->large_error_elapsed_ms;
        output->accumulated_derate_rpm = state->accumulated_derate_rpm;
        return true;
    }

    const int32_t left_abs = abs_i32(input->left_actual_rpm);
    const int32_t right_abs = abs_i32(input->right_actual_rpm);
    output->average_rpm = (left_abs + right_abs) / 2;
    output->weak_field = output->average_rpm >= config->weak_field_threshold_rpm;

    const int32_t longitudinal = (input->left_theoretical_target_rpm +
            input->right_theoretical_target_rpm) / 2;
    const int32_t steering = (input->left_theoretical_target_rpm -
            input->right_theoretical_target_rpm) / 2;
    output->longitudinal_target_rpm = longitudinal;

    const int32_t motion_sign = longitudinal > 0 ? 1 : (longitudinal < 0 ? -1 : 0);
    const bool same_direction = motion_sign != 0 &&
            input->left_theoretical_target_rpm * motion_sign >= 0 &&
            input->right_theoretical_target_rpm * motion_sign >= 0;
    if (!same_direction) {
        output->diagnostic_flags |= VCU_SYNC_DIAG_DIRECTION_UNSUPPORTED;
        vcu_speed_sync_reset(state);
        return true;
    }

    output->theoretical_difference_rpm =
            abs_i32(input->left_theoretical_target_rpm) -
            abs_i32(input->right_theoretical_target_rpm);
    output->actual_difference_rpm = motion_sign * input->left_actual_rpm -
            motion_sign * input->right_actual_rpm;
    output->raw_error_rpm = output->actual_difference_rpm -
            output->theoretical_difference_rpm;

    const uint32_t elapsed_ms = (uint32_t)(input->dt_seconds * 1000.0f + 0.5f);
    if (output->weak_field && abs_i32(output->raw_error_rpm) >=
            config->large_error_threshold_rpm) {
        if (UINT32_MAX - state->large_error_elapsed_ms < elapsed_ms) {
            state->large_error_elapsed_ms = UINT32_MAX;
        } else {
            state->large_error_elapsed_ms += elapsed_ms;
        }
        if (state->large_error_elapsed_ms >= config->large_error_hold_ms) {
            if (INT32_MAX - state->accumulated_derate_rpm >=
                    config->longitudinal_derate_step_rpm) {
                state->accumulated_derate_rpm += config->longitudinal_derate_step_rpm;
            }
            state->large_error_elapsed_ms = 0U;
        }
    } else {
        state->large_error_elapsed_ms = 0U;
        if (!output->weak_field) state->accumulated_derate_rpm = 0;
    }

    int32_t longitudinal_magnitude = abs_i32(longitudinal);
    if (state->accumulated_derate_rpm > 0) {
        const int32_t minimum = longitudinal_magnitude >=
                config->minimum_longitudinal_target_rpm ?
                config->minimum_longitudinal_target_rpm : longitudinal_magnitude;
        longitudinal_magnitude = clamp_i32(longitudinal_magnitude -
                state->accumulated_derate_rpm, minimum, longitudinal_magnitude);
    }
    const int32_t derated_longitudinal = motion_sign * longitudinal_magnitude;
    output->derated_longitudinal_target_rpm = derated_longitudinal;
    output->left_derated_target_rpm = derated_longitudinal + steering;
    output->right_derated_target_rpm = derated_longitudinal - steering;

    output->theoretical_difference_rpm =
            abs_i32(output->left_derated_target_rpm) -
            abs_i32(output->right_derated_target_rpm);
    output->raw_error_rpm = output->actual_difference_rpm -
            output->theoretical_difference_rpm;

    const int32_t error_magnitude = abs_i32(output->raw_error_rpm);
    if (error_magnitude > config->error_deadzone_rpm) {
        output->effective_error_rpm = error_magnitude - config->error_deadzone_rpm;
    }
    output->raw_correction_rpm = config->kp * (float)output->effective_error_rpm;

    if (output->average_rpm < config->medium_speed_threshold_rpm) {
        output->active_limit_rpm = config->low_limit_rpm;
    } else if (output->average_rpm < config->high_speed_threshold_rpm) {
        output->active_limit_rpm = config->medium_limit_rpm;
    } else {
        output->active_limit_rpm = config->high_limit_rpm;
    }
    output->limited_correction_rpm = clamp_i32(round_i32(output->raw_correction_rpm),
            0, output->active_limit_rpm);

    output->left_target_rpm = output->left_derated_target_rpm;
    output->right_target_rpm = output->right_derated_target_rpm;
    if (output->raw_error_rpm > config->error_deadzone_rpm) {
        output->left_target_rpm = reduce_magnitude(output->left_derated_target_rpm,
                output->limited_correction_rpm);
    } else if (output->raw_error_rpm < -config->error_deadzone_rpm) {
        output->right_target_rpm = reduce_magnitude(output->right_derated_target_rpm,
                output->limited_correction_rpm);
    }

    output->sync_active = true;
    output->large_error_elapsed_ms = state->large_error_elapsed_ms;
    output->accumulated_derate_rpm = state->accumulated_derate_rpm;
    return true;
}
