#ifndef ESP_VCU_FLIGHT_LOG_H
#define ESP_VCU_FLIGHT_LOG_H
#include <stdbool.h>
#include <stdint.h>
void vcu_flight_log_init(void);
void vcu_flight_log_receive(uint32_t id,const uint8_t *data,uint8_t len,bool ext,bool rtr,uint32_t rx_ms);
void vcu_flight_log_bus(uint32_t state,uint32_t tec,uint32_t rec,uint32_t missed,uint32_t overrun,uint32_t failed,uint32_t bus_errors,uint32_t queue_lost);
#endif
