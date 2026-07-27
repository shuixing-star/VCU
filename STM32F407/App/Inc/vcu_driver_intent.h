#ifndef VCU_DRIVER_INTENT_H
#define VCU_DRIVER_INTENT_H

#include <stdbool.h>

#include "vcu_types.h"

void vcu_driver_intent_init(void);
bool vcu_driver_intent_evaluate(const vcu_input_snapshot_t *inputs,
                                vcu_driver_intent_t *intent);

#endif
