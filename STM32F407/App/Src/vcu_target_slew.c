#include "vcu_target_slew.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

static int32_t round_i32(float value) {
    if (value >= (float)INT32_MAX) return INT32_MAX;
    if (value <= (float)INT32_MIN) return INT32_MIN;
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

void vcu_target_slew_reset(vcu_target_slew_state_t *state, int32_t target_rpm) {
    if (state == NULL) return;
    state->initialized = true;
    state->output_rpm = (float)target_rpm;
}

bool vcu_target_slew_step(vcu_target_slew_state_t *state,
        int32_t requested_target_rpm,
        int32_t rate_limit_rpm_s,
        float dt_seconds,
        int32_t *limited_target_rpm) {
    if (state == NULL || limited_target_rpm == NULL || rate_limit_rpm_s <= 0 ||
            !isfinite(dt_seconds) || dt_seconds <= 0.0f || dt_seconds > 0.1f) {
        return false;
    }
    if (!state->initialized) vcu_target_slew_reset(state, 0);

    const float requested = (float)requested_target_rpm;
    const float maximum_change = (float)rate_limit_rpm_s * dt_seconds;
    float change = requested - state->output_rpm;
    if (change > maximum_change) change = maximum_change;
    if (change < -maximum_change) change = -maximum_change;
    state->output_rpm += change;
    if ((change >= 0.0f && state->output_rpm > requested) ||
            (change < 0.0f && state->output_rpm < requested)) {
        state->output_rpm = requested;
    }
    *limited_target_rpm = round_i32(state->output_rpm);
    return true;
}
