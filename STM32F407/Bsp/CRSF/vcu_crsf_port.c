#include "vcu_crsf_port.h"

#include "stm32f4xx_hal.h"

#define VCU_CRSF_RX_RING_SIZE 256U
#define VCU_CRSF_RX_RING_MASK (VCU_CRSF_RX_RING_SIZE - 1U)

#if (VCU_CRSF_RX_RING_SIZE & VCU_CRSF_RX_RING_MASK) != 0
#error "VCU_CRSF_RX_RING_SIZE must be a power of two"
#endif

static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_ring[VCU_CRSF_RX_RING_SIZE];
static uint32_t s_rx_time_ring[VCU_CRSF_RX_RING_SIZE];
static volatile uint32_t s_error_count;
static volatile uint32_t s_overflow_count;

/* Latch the first invalid indices; never hide corruption by masking them. */
typedef struct {
    uint32_t latched;
    uint32_t head;
    uint32_t tail;
} vcu_crsf_ring_fault_t;
volatile vcu_crsf_ring_fault_t g_vcu_crsf_ring_fault;

static bool ring_indices_valid(uint16_t head, uint16_t tail)
{
    uint32_t primask;
    if (g_vcu_crsf_ring_fault.latched != 0U) return false;
    if (head < VCU_CRSF_RX_RING_SIZE && tail < VCU_CRSF_RX_RING_SIZE) return true;
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_vcu_crsf_ring_fault.latched == 0U) {
        g_vcu_crsf_ring_fault.head = head;
        g_vcu_crsf_ring_fault.tail = tail;
        s_error_count++;
        __DMB();
        g_vcu_crsf_ring_fault.latched = 1U;
    }
    __set_PRIMASK(primask);
    return false;
}

bool vcu_crsf_port_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t peripheral_clock;
    uint32_t brr;

    if (baudrate == 0U) {
        return false;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();

    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);

    peripheral_clock = HAL_RCC_GetPCLK1Freq();
    brr = (peripheral_clock + (baudrate / 2U)) / baudrate;
    if (brr == 0U || brr > 0xFFFFU) {
        return false;
    }

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_error_count = 0U;
    s_overflow_count = 0U;
    g_vcu_crsf_ring_fault.latched = 0U;
    g_vcu_crsf_ring_fault.head = 0U;
    g_vcu_crsf_ring_fault.tail = 0U;

    USART3->CR1 = 0U;
    USART3->CR2 = 0U;
    USART3->CR3 = 0U;
    USART3->BRR = brr;
    USART3->CR1 = USART_CR1_RE | USART_CR1_RXNEIE;

    HAL_NVIC_SetPriority(USART3_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    USART3->CR1 |= USART_CR1_UE;
    return true;
}

size_t vcu_crsf_port_read(uint8_t *data,
                          uint32_t *received_ms,
                          size_t capacity)
{
    size_t count = 0U;

    if (data == NULL || received_ms == NULL) {
        return 0U;
    }

    while (count < capacity) {
        const uint16_t tail = s_rx_tail;
        const uint16_t head = s_rx_head;
        if (!ring_indices_valid(head, tail) || tail == head) break;
        __DMB();
        data[count] = s_rx_ring[tail];
        received_ms[count] = s_rx_time_ring[tail];
        count++;
        /* Complete both slot reads before allowing the ISR to reuse it. */
        __DMB();
        s_rx_tail = (uint16_t)((tail + 1U) & VCU_CRSF_RX_RING_MASK);
    }
    return count;
}

uint32_t vcu_crsf_port_error_count(void)
{
    return s_error_count;
}

uint32_t vcu_crsf_port_overflow_count(void)
{
    return s_overflow_count;
}

void USART3_IRQHandler(void)
{
    const uint32_t status = USART3->SR;
    const uint32_t error_flags =
            status & (USART_SR_ORE | USART_SR_NE |
                      USART_SR_FE | USART_SR_PE);

    if ((status & USART_SR_RXNE) != 0U || error_flags != 0U) {
        const uint8_t byte = (uint8_t)USART3->DR;

        if (error_flags != 0U) {
            s_error_count++;
        } else {
            const uint16_t head = s_rx_head;
            const uint16_t tail = s_rx_tail;
            uint16_t next;
            /* A corrupt current head is unsafe even if next masks in range. */
            if (!ring_indices_valid(head, tail)) return;
            next = (uint16_t)((head + 1U) & VCU_CRSF_RX_RING_MASK);

            if (next == tail) {
                s_overflow_count++;
            } else {
                s_rx_ring[head] = byte;
                s_rx_time_ring[head] = HAL_GetTick();
                __DMB();
                s_rx_head = next;
            }
        }
    }
}
