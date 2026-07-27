#ifndef VCU_APP_H
#define VCU_APP_H

#include <stdbool.h>
#include <stdint.h>

bool vcu_app_init(void);
void vcu_app_step(uint32_t now_ms);

#endif
