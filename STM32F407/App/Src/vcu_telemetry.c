#include "vcu_telemetry.h"

void vcu_telemetry_init(void)
{
}

void vcu_telemetry_update(const vcu_input_snapshot_t *inputs,
                          const vcu_driver_intent_t *intent,
                          const vcu_vehicle_feedback_t *feedback,
                          const vcu_safety_output_t *safety,
                          const vcu_actuator_command_t *command,
                          uint32_t now_ms)
{
    (void)inputs;
    (void)intent;
    (void)feedback;
    (void)safety;
    (void)command;
    (void)now_ms;

    /* Snapshot encoding and publication are intentionally deferred. */
}
