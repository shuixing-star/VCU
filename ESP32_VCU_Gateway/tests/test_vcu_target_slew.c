#include "vcu_target_slew.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
    vcu_target_slew_state_t state = {0};
    int32_t output = 0;

    assert(vcu_target_slew_step(&state, 1000, 2550, 0.02f, &output));
    assert(output == 51);
    assert(vcu_target_slew_step(&state, 1000, 2550, 0.02f, &output));
    assert(output == 102);

    vcu_target_slew_reset(&state, 1000);
    assert(vcu_target_slew_step(&state, 0, 2550, 0.02f, &output));
    assert(output == 949);

    vcu_target_slew_reset(&state, 0);
    for (int i = 0; i < 4; i++) {
        assert(vcu_target_slew_step(&state, 1000, 2550, 0.005f, &output));
    }
    assert(output == 51);

    assert(!vcu_target_slew_step(&state, 1000, 2550, NAN, &output));
    assert(!vcu_target_slew_step(&state, 1000, 2550, 0.2f, &output));

    puts("VCU target slew tests passed");
    return 0;
}
