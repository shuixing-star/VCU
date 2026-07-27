#include "vcu_driver_intent.h"

#include <string.h>

void vcu_driver_intent_init(void)
{
}

bool vcu_driver_intent_evaluate(const vcu_input_snapshot_t *inputs,
                                vcu_driver_intent_t *intent)
{
    if (inputs == 0 || intent == 0) {
        return false;
    }

    memset(intent, 0, sizeof(*intent));
    intent->source = VCU_CONTROL_SOURCE_NONE;
    intent->timestamp_ms = inputs->timestamp_ms;

    /* Safe stub: no input source is authorized until its policy is added. */
    return false;
}
