#include "vcu_inputs.h"

#include <string.h>

bool vcu_inputs_init(void)
{
    /*
     * Safe stub: final ADC, GPIO, debounce, and remote receiver acquisition
     * will be connected after the CubeMX pin configuration is reviewed.
     */
    return true;
}

bool vcu_inputs_read(uint32_t now_ms, vcu_input_snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return false;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_ms = now_ms;
    snapshot->local_inputs_valid = false;
    return false;
}
