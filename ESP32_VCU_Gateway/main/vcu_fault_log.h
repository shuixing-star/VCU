#ifndef VCU_FAULT_LOG_H
#define VCU_FAULT_LOG_H
#include <stdbool.h>
#include <stdint.h>
void vcu_fault_log_begin(uint32_t now);
void vcu_fault_log_stop(uint32_t now);
bool vcu_fault_log_open(const char *main_path);
bool vcu_fault_log_drain(void);
bool vcu_fault_log_finish(void);
bool vcu_fault_log_receive(uint32_t id,const uint8_t *data,uint8_t len,bool ext,uint32_t now);
#endif
