#include "vcu_actuators.h"

bool vcu_actuators_init(void)
{
    vcu_actuators_force_safe();
    return true;
}

bool vcu_actuators_apply(const vcu_actuator_command_t *command,
                         const vcu_safety_output_t *safety)
{
    if (command == 0 || safety == 0 ||
        !safety->actuation_allowed || command->output_enabled) {
        vcu_actuators_force_safe();
        return false;
    }

    /*
     * The structure-only implementation accepts only an explicitly disabled
     * command. Real GPIO/PWM output will be added after the hardware mapping.
     */
    vcu_actuators_force_safe();
    return true;
}

void vcu_actuators_force_safe(void)
{
    /*
     * Intentionally empty until the active levels and PWM signal chain are
     * verified. This function remains the single future owner of safe outputs.
     */
}
