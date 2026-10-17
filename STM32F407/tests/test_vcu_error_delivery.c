/* Reuse the port's HAL fixture; this entry exercises the real event pipeline. */
#define main can_port_regression_entry
#include "test_vcu_can_port.c"
#undef main
#include "vcu_control_telemetry.h"
#include "vcu_esp_link.h"
bool vcu_motor_can_controller_recent(const vcu_motor_controller_state_t *c,
        uint32_t now, uint32_t timeout) { (void)c; (void)now; (void)timeout; return true; }
int main(void) {
    vcu_crsf_uart_state_t crsf={0};
    vcu_drive_runtime_state_t drive={0};
    vcu_motor_can_state_t motors={0};
    reset();vcu_control_telemetry_init();vcu_esp_link_init();
    bus_stuck=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    for(unsigned t=5;t<=35;t+=5) {
        us=(uint64_t)t*1000;
        vcu_can_port_poll();
        vcu_control_telemetry_step(t,&crsf,&drive,&motors);
    }
    assert(sent==0);
    /* First error transmission failed after admission. Recover the bus. */
    bus_stuck=false;
    for(unsigned t=40;t<=350;t+=5) {
        us=(uint64_t)t*1000;
        bus_stuck=(t==105); /* First snapshot fragment fails; retry next slot. */
        vcu_can_port_poll();
        vcu_control_telemetry_step(t,&crsf,&drive,&motors);
    }
    us+=1000;vcu_can_port_poll();
    unsigned errors=0, probe_mask=0;
    uint16_t event_stamp=0;
    for(unsigned i=0;i<sent;i++) if(sent_id[i]==VCU_GATEWAY_ID_CAN_ERROR) {
        vcu_gateway_can_error_t e;
        vcu_gateway_unpack_can_error(sent_data[i],&e);
        assert(e.sequence==1 && e.type==VCU_GATEWAY_CAN_ERROR_TX_TIMEOUT);
        event_stamp=e.timestamp_10ms;
        errors++;
    }
    assert(errors==1);
    for(unsigned i=0;i<sent;i++) if(sent_id[i]==VCU_GATEWAY_ID_CAN_TX_PROBE) {
        uint8_t seq, part;uint16_t stamp;uint32_t value;
        assert(vcu_gateway_unpack_tx_probe(sent_data[i],&seq,&stamp,&part,&value));
        assert(seq==1 && stamp==event_stamp);
        probe_mask |= 1U<<part;
        if(part==2) assert(value==left.id);
        if(part==3) assert(value==right.id);
    }
    assert(probe_mask==VCU_GATEWAY_TX_PROBE_COMPLETE);
    puts("PASS all snapshot fragments survive a failed TX and retain event identity");
    puts("PASS original CAN error survives failed hardware TX and arrives after recovery");
    return 0;
}
