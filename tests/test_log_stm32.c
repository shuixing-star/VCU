#include "vcu_flight_log.h"
#include "vcu_log_wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t now, slots[1000], decoded;
static bool blocked;
static vcu_log_decoder_t decoder;
static vcu_can_tx_observation_t tx;
void vcu_can_port_get_tx_observation(vcu_can_tx_observation_t *out){*out=tx;}
bool vcu_can_port_take_tx_log(uint8_t out[8]){(void)out;return false;}
uint32_t vcu_can_port_tx_log_dropped(void){return 0;}
uint32_t vcu_can_port_esr(void){return 0x01020004;}
uint32_t vcu_can_port_tx_error_count(void){return 3;}
uint32_t vcu_can_port_rx_overflow_count(void){return 4;}
uint32_t vcu_can_port_rx_error_count(void){return 5;}
uint32_t vcu_can_port_error_event_overflow_count(void){return 6;}
bool vcu_motor_can_controller_recent(const vcu_motor_controller_state_t *c,uint32_t n,uint32_t t){(void)c;(void)n;(void)t;return false;}
bool vcu_can_port_send_log(const vcu_can_port_frame_t *f){
    if(blocked)return false;
    assert(f->data_length==8 && !f->extended_id);
    assert(++slots[(now/10)%1000]<=6);
    if(f->id==VCU_LOG_CAN_ID && vcu_log_decode(&decoder,f->data,now))decoded++;
    return true;
}
int main(void){
    vcu_crsf_uart_state_t rc={0};vcu_drive_runtime_state_t drive={0};vcu_motor_can_state_t motors={0};
    drive.input_valid=true;drive.input_rx_ms=17;drive.preview.throttle_raw=1234;
    drive.preview.throttle_per_mille=456;drive.loop_count=1;
    vcu_flight_log_init(0x12345678);
    for(now=1;now<2000;now++){drive.control_step_ms=now;vcu_flight_log_step(now,&rc,&drive,&motors);}
    assert(decoded>=19);
    assert(vcu_log_u32(decoder.record+12)==0x12345678);
    assert(vcu_log_u32(decoder.record+20)==17 && vcu_log_u16(decoder.record+24)==1234);
    assert(vcu_log_u16(decoder.record+32)==456 && vcu_log_u32(decoder.record+128)==0x01020004);
    memset(slots,0,sizeof(slots));blocked=true;
    for(now=2000;now<5000;now++)vcu_flight_log_step(now,&rc,&drive,&motors);
    blocked=false;
    for(now=5000;now<8000;now++)vcu_flight_log_step(now,&rc,&drive,&motors);
    assert(vcu_log_u32(decoder.record+144)>0);
    assert(vcu_log_u16(decoder.record+150)==8);
    puts("PASS STM32 capture values, timestamp provenance, 600 fps cap and congestion recovery/drop counters");
    return 0;
}
