#ifndef VCU_SAFETY_H
#define VCU_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "vcu_types.h"

void vcu_safety_init(void);
bool vcu_safety_evaluate(const vcu_driver_intent_t *intent,
                         const vcu_vehicle_feedback_t *feedback,
                         uint32_t now_ms,
                         vcu_safety_output_t *output);

#endif
