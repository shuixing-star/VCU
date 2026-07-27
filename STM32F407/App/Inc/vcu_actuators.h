#ifndef VCU_ACTUATORS_H
#define VCU_ACTUATORS_H

#include <stdbool.h>

#include "vcu_types.h"

bool vcu_actuators_init(void);
bool vcu_actuators_apply(const vcu_actuator_command_t *command,
                         const vcu_safety_output_t *safety);
void vcu_actuators_force_safe(void);

#endif
