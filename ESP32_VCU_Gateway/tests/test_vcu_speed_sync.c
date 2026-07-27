#include "vcu_speed_sync.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static vcu_speed_sync_input_t input_forward(void) {
    return (vcu_speed_sync_input_t) {
        .enabled = true,
        .left_can_valid = true,
        .right_can_valid = true,
        .dt_seconds = 0.02f,
        .left_actual_rpm = 1000,
        .right_actual_rpm = 1000,
        .left_theoretical_target_rpm = 1000,
        .right_theoretical_target_rpm = 1000,
    };
}

int main(void) {
    vcu_speed_sync_config_t config;
    vcu_speed_sync_default_config(&config);
    vcu_speed_sync_state_t state = {0};
    vcu_speed_sync_output_t output;
    vcu_speed_sync_input_t input = input_forward();

    assert(vcu_speed_sync_step(&state, &config, &input, &output));
    assert(output.left_target_rpm == 1000 && output.right_target_rpm == 1000);

    input.left_actual_rpm = 1041;
    assert(vcu_speed_sync_step(&state, &config, &input, &output));
    assert(output.effective_error_rpm == 1);
    assert(output.left_target_rpm == 1000);

    input.left_actual_rpm = 1200;
    input.right_actual_rpm = 1000;
    assert(vcu_speed_sync_step(&state, &config, &input, &output));
    assert(output.effective_error_rpm == 160);
    assert(output.limited_correction_rpm == 32);
    assert(output.left_target_rpm == 968 && output.right_target_rpm == 1000);

    input.left_actual_rpm = 1000;
    input.right_actual_rpm = 1200;
    assert(vcu_speed_sync_step(&state, &config, &input, &output));
    assert(output.left_target_rpm == 1000 && output.right_target_rpm == 968);

    input.left_actual_rpm = -1200;
    input.right_actual_rpm = -1000;
    input.left_theoretical_target_rpm = -1000;
    input.right_theoretical_target_rpm = -1000;
    assert(vcu_speed_sync_step(&state, &config, &input, &output));
    assert(output.left_target_rpm == -968 && output.right_target_rpm == -1000);

    input = input_forward();
    input.left_theoretical_target_rpm = 1400;
    input.right_theoretical_target_rpm = 1000;
    input.left_actual_rpm = 1500;
    input.right_actual_rpm = 1000;
    assert(vcu_speed_sync_step(&state, &config, &input, &output));
    assert(output.raw_error_rpm == 100);
    assert(output.left_target_rpm == 1388 && output.right_target_rpm == 1000);

    input = input_forward();
    input.left_actual_rpm = 5000;
    input.right_actual_rpm = 4000;
    assert(vcu_speed_sync_step(&state, &config, &input, &output));
    assert(output.active_limit_rpm == 250);
    assert(output.limited_correction_rpm == 192);

    input.left_actual_rpm = 4300;
    input.right_actual_rpm = 4000;
    for (int i = 0; i < 40; i++) {
        assert(vcu_speed_sync_step(&state, &config, &input, &output));
    }
    assert(output.accumulated_derate_rpm == 100);
    assert(output.derated_longitudinal_target_rpm == 900);

    input.left_can_valid = false;
    assert(!vcu_speed_sync_step(&state, &config, &input, &output));
    assert(!output.sync_active);
    assert(output.left_target_rpm == input.left_theoretical_target_rpm);

    input = input_forward();
    input.dt_seconds = NAN;
    assert(!vcu_speed_sync_step(&state, &config, &input, &output));
    assert((output.diagnostic_flags & VCU_SYNC_DIAG_INVALID_DT) != 0U);

    puts("VCU speed synchronization tests passed");
    return 0;
}
