#ifndef TEST_CRSF_HAL_H
#define TEST_CRSF_HAL_H
#include <stdint.h>
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { volatile uint32_t SR, DR, BRR, CR1, CR2, CR3; } test_usart_t;
extern test_usart_t test_usart;
extern uint32_t test_tick, test_primask;
#define USART3 (&test_usart)
#define GPIOB ((void *)0)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_MODE_AF_PP 2U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF7_USART3 7U
#define USART3_IRQn 39
#define USART_CR1_RE (1U << 2)
#define USART_CR1_RXNEIE (1U << 5)
#define USART_CR1_UE (1U << 13)
#define USART_SR_ORE (1U << 3)
#define USART_SR_NE (1U << 2)
#define USART_SR_FE (1U << 1)
#define USART_SR_PE (1U << 0)
#define USART_SR_RXNE (1U << 5)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART3_FORCE_RESET() ((void)0)
#define __HAL_RCC_USART3_RELEASE_RESET() ((void)0)
#define __DMB() __asm__ volatile ("" ::: "memory")
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1; }
static inline void __set_PRIMASK(uint32_t p) { test_primask = p; }
static inline void HAL_NVIC_DisableIRQ(int n) { (void)n; }
static inline void HAL_NVIC_ClearPendingIRQ(int n) { (void)n; }
static inline void HAL_NVIC_EnableIRQ(int n) { (void)n; }
static inline void HAL_NVIC_SetPriority(int n, uint32_t p, uint32_t s) { (void)n;(void)p;(void)s; }
static inline void HAL_GPIO_Init(void *g, GPIO_InitTypeDef *c) { (void)g;(void)c; }
static inline uint32_t HAL_RCC_GetPCLK1Freq(void) { return 42000000U; }
static inline uint32_t HAL_GetTick(void) { return test_tick; }
#endif
