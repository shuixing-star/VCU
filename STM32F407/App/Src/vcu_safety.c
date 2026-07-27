#include "vcu_safety.h"

#include <string.h>

void vcu_safety_init(void)
{
}

bool vcu_safety_evaluate(const vcu_driver_intent_t *intent,
                         const vcu_vehicle_feedback_t *feedback,
                         uint32_t now_ms,
                         vcu_safety_output_t *output)
{
    (void)intent;
    (void)feedback;
    (void)now_ms;

    if (output == 0) {
        return false;
    }

    memset(output, 0, sizeof(*output));
    output->state = VCU_STATE_DISARMED;
    output->fault_flags = VCU_FAULT_INPUT_INVALID;
    output->actuation_allowed = false;

    /* Safe stub: the state machine will be implemented as a separate slice. */
    return true;
}
