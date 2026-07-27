#ifndef VCU_CRSF_PORT_H
#define VCU_CRSF_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool vcu_crsf_port_init(uint32_t baudrate);
size_t vcu_crsf_port_read(uint8_t *data,
                          uint32_t *received_ms,
                          size_t capacity);
uint32_t vcu_crsf_port_error_count(void);
uint32_t vcu_crsf_port_overflow_count(void);

#endif
