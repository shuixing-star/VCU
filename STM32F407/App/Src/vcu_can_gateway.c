#include "vcu_can_gateway.h"

#include <string.h>

static vcu_vehicle_feedback_t s_feedback;

bool vcu_can_gateway_init(void)
{
    memset(&s_feedback, 0, sizeof(s_feedback));
    return true;
}

void vcu_can_gateway_step(uint32_t now_ms)
{
    (void)now_ms;
    /* CAN transmit scheduling will be added after CAN IDs are assigned. */
}

bool vcu_can_gateway_process_rx(const vcu_can_frame_t *frame, uint32_t now_ms)
{
    (void)now_ms;

    if (!vcu_can_protocol_validate_frame(frame)) {
        return false;
    }

    /* No CAN message owns vehicle control in this structure-only slice. */
    return false;
}

bool vcu_can_gateway_get_vehicle_feedback(vcu_vehicle_feedback_t *feedback)
{
    if (feedback == 0) {
        return false;
    }

    *feedback = s_feedback;
    return feedback->left_motor.valid || feedback->right_motor.valid;
}
