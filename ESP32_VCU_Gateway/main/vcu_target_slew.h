#ifndef VCU_TARGET_SLEW_H
#define VCU_TARGET_SLEW_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool initialized;
    float output_rpm;
} vcu_target_slew_state_t;

void vcu_target_slew_reset(vcu_target_slew_state_t *state, int32_t target_rpm);
bool vcu_target_slew_step(vcu_target_slew_state_t *state,
        int32_t requested_target_rpm,
        int32_t rate_limit_rpm_s,
        float dt_seconds,
        int32_t *limited_target_rpm);

#endif
