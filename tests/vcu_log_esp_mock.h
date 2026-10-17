#ifndef VCU_LOG_ESP_MOCK_H
#define VCU_LOG_ESP_MOCK_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/stat.h>
#define HW_VCU_STM32_GATEWAY
#define FW_VERSION_MAJOR 7
#define FW_VERSION_MINOR 0
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define MALLOC_CAP_8BIT 1
#define pdPASS 1
#define pdMS_TO_TICKS(n) (n)
static char *file_basepath;
static uint32_t mock_now;
static size_t mock_free_heap=150000;
static inline int64_t esp_timer_get_time(void){return (int64_t)mock_now*1000;}
static inline uint32_t esp_random(void){return 0x11223344;}
static inline size_t heap_caps_get_free_size(int cap){(void)cap;return mock_free_heap;}
static inline size_t heap_caps_get_minimum_free_size(int cap){(void)cap;return 100000;}
static inline int xTaskCreate(void (*task)(void*),const char *name,unsigned size,void *arg,int pri,void *handle) {
    (void)task;(void)name;(void)size;(void)arg;(void)pri;(void)handle;return pdPASS;
}
static inline void vTaskDelay(unsigned n){(void)n;}
static inline void terminal_register_command_callback(const char *a,const char *b,const char *c,void (*cb)(int,const char**)) {(void)a;(void)b;(void)c;(void)cb;}
#include <stdarg.h>
static inline void commands_printf(const char *fmt,...) { va_list ap;va_start(ap,fmt);vprintf(fmt,ap);va_end(ap); }
#ifdef _WIN32
static inline int mock_mkdir(const char *p,int mode){(void)mode;return mkdir(p);}
#define mkdir(p,mode) mock_mkdir(p,mode)
#endif
#endif
