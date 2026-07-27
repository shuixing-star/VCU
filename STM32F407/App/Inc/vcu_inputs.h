#ifndef VCU_INPUTS_H
#define VCU_INPUTS_H

#include <stdbool.h>
#include <stdint.h>

#include "vcu_types.h"

bool vcu_inputs_init(void);
bool vcu_inputs_read(uint32_t now_ms, vcu_input_snapshot_t *snapshot);

#endif
