#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
test_usart_t test_usart;
uint32_t test_tick, test_primask;
/* Include the port to inject corrupt private indices deliberately. */
#include "../Bsp/CRSF/vcu_crsf_port.c"
static void receive(uint8_t value, uint32_t stamp) {
    test_tick = stamp;
    test_usart.DR = value;
    test_usart.SR = USART_SR_RXNE;
    USART3_IRQHandler();
}
int main(void) {
    uint8_t bytes[256], before[256];
    uint32_t times[256], old_times[256];
    assert(!vcu_crsf_port_init(0));
    assert(vcu_crsf_port_init(420000));
    /* Fill/wrap many times: preserve byte/timestamp pairing and drop newest. */
    for (unsigned round=0; round<10; round++) {
        for (unsigned i=0; i<255; i++) receive((uint8_t)i, round*1000+i);
        receive(99, 99999);
        assert(vcu_crsf_port_read(bytes, times, 256) == 255);
        for (unsigned i=0; i<255; i++) {
            assert(bytes[i] == (uint8_t)i);
            assert(times[i] == round*1000+i);
        }
        assert(vcu_crsf_port_read(bytes, times, 1) == 0);
    }
    assert(s_overflow_count == 10);
    test_usart.SR = USART_SR_RXNE | USART_SR_ORE;
    USART3_IRQHandler();
    assert(s_error_count == 1 && vcu_crsf_port_read(bytes,times,1) == 0);
    /* Bad current head must not be rescued by the next-index mask. */
    memcpy(before, s_rx_ring, sizeof(before));
    memcpy(old_times, s_rx_time_ring, sizeof(old_times));
    s_rx_head = 0x8001;
    receive(42,123);
    assert(g_vcu_crsf_ring_fault.latched == 1);
    assert(g_vcu_crsf_ring_fault.head == 0x8001);
    assert(memcmp(before,s_rx_ring,sizeof(before)) == 0);
    assert(memcmp(old_times,s_rx_time_ring,sizeof(old_times)) == 0);
    assert(test_primask == 0);
    assert(vcu_crsf_port_read(bytes,times,1) == 0);
    s_rx_head = 0;
    receive(43,124);
    assert(g_vcu_crsf_ring_fault.head == 0x8001); /* First fault retained. */
    assert(memcmp(before,s_rx_ring,sizeof(before)) == 0);
    assert(vcu_crsf_port_init(420000));
    receive(44,125);
    assert(vcu_crsf_port_read(bytes,times,1) == 1 && bytes[0] == 44);
    s_rx_tail = 256;
    test_primask = 1;
    assert(vcu_crsf_port_read(bytes,times,1) == 0);
    assert(g_vcu_crsf_ring_fault.latched == 1 && g_vcu_crsf_ring_fault.tail == 256);
    assert(test_primask == 1);
    puts("PASS CRSF wrap/full/timestamps/errors and corrupt-index latch without slot writes");
    return 0;
}
