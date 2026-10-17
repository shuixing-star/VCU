#include "vcu_flight_log.h"
#include "../../../common/vcu_log_wire.h"
#include <string.h>
#define RECORD_QUEUE 8U
static uint8_t records[RECORD_QUEUE][VCU_LOG_SIZE];
static uint8_t raw[2][2][8], raw_valid[2];
static unsigned head, count, part, highwater;
static uint32_t nonce, sequence, dropped, last_sample, slot_ms, frame_ms;
static unsigned tokens;
static uint8_t previous_state, previous_reason, previous_flags, pending_command;
static bool previous_failure, tx_pending, started;
static uint8_t tx_data[8];

static uint16_t age(bool valid, uint32_t now, uint32_t then) {
    uint32_t n = now - then;
    return !valid || n > 65534U ? 65535U : (uint16_t)n;
}
void vcu_flight_log_init(uint32_t boot_nonce) {
    memset(raw, 0, sizeof(raw)); memset(raw_valid, 0, sizeof(raw_valid));
    head = count = part = highwater = 0; nonce = boot_nonce;
    sequence = dropped = last_sample = slot_ms = frame_ms = 0;
    previous_state = previous_reason = previous_flags = 255;
    pending_command = 0; previous_failure = tx_pending = started = false; tokens = 0;
}
void vcu_flight_log_command(uint8_t command) { pending_command = command; }
void vcu_flight_log_rx(const vcu_can_port_frame_t *f) {
    const uint32_t ids[2][2] = {
        { VCU_MOTOR_LEFT_ELECTRICAL_STATUS_ID, VCU_MOTOR_LEFT_STATE_STATUS_ID },
        { VCU_MOTOR_RIGHT_ELECTRICAL_STATUS_ID, VCU_MOTOR_RIGHT_STATE_STATUS_ID }
    };
    unsigned i,j;
    if (!f->extended_id || f->remote_frame || f->data_length != 8) return;
    for (i=0;i<2;i++) for(j=0;j<2;j++) if(f->id == ids[i][j]) {
        memcpy(raw[i][j],f->data,8); raw_valid[i] |= (uint8_t)(1U << j);
    }
}
static void capture(uint32_t now, const vcu_crsf_uart_state_t *rc,
                    const vcu_drive_runtime_state_t *d, const vcu_motor_can_state_t *m,
                    const vcu_can_tx_observation_t *tx, uint8_t flags, bool event) {
    uint8_t *p;
    unsigned i;
    sequence++;
    if (count == RECORD_QUEUE) { dropped++; return; }
    p = records[(head+count) % RECORD_QUEUE]; memset(p,0,VCU_LOG_SIZE);
    p[0]=0xa7; p[1]=VCU_LOG_SCHEMA; p[2]=event ? 2U : 1U; p[3]=pending_command;
    vcu_log_put32(p+4,sequence); vcu_log_put32(p+8,now);
    vcu_log_put32(p+12,nonce); vcu_log_put32(p+16,VCU_LOG_BUILD_ID);
    vcu_log_put32(p+20,d->input_rx_ms);
    vcu_log_put16(p+24,d->preview.throttle_raw); vcu_log_put16(p+26,d->preview.steering_raw);
    vcu_log_put16(p+28,d->preview.arm_raw); vcu_log_put16(p+30,d->preview.reset_raw);
    vcu_log_put16(p+32,(uint16_t)d->preview.throttle_per_mille);
    vcu_log_put16(p+34,(uint16_t)d->preview.steering_per_mille);
    vcu_log_put16(p+36,(uint16_t)d->latest_left_base_target); vcu_log_put16(p+38,(uint16_t)d->latest_right_base_target);
    vcu_log_put16(p+40,(uint16_t)d->latest_left_target); vcu_log_put16(p+42,(uint16_t)d->latest_right_target);
    p[44]=(uint8_t)d->state; p[45]=(uint8_t)d->reason; p[46]=flags;
    p[47]=d->input_valid;
    for(i=0;i<2;i++) {
        const vcu_motor_controller_state_t *c=&m->controller[i];
        uint8_t *q=p+48+i*24;
        memcpy(q,raw[i][0],8); memcpy(q+8,raw[i][1],8);
        vcu_log_put16(q+16,age(c->has_electrical_status,now,c->last_electrical_status_ms));
        vcu_log_put16(q+18,age(c->has_state_status,now,c->last_state_status_ms));
        vcu_log_put16(q+20,age(c->heartbeat_advanced,now,c->last_heartbeat_change_ms));
        q[22]=raw_valid[i] | (vcu_motor_can_controller_recent(c,now,500) ? 4U:0U);
        q[23]=(uint8_t)c->direction_sign;
    }
    vcu_log_put32(p+96,tx->started_ms); vcu_log_put32(p+100,tx->sequence);
    p[104]=tx->queued_mask; p[105]=tx->txok_mask; p[106]=tx->success;
    p[107]=!d->loop_count || (uint32_t)(now-d->control_step_ms)>254U ? 255U : (uint8_t)(now-d->control_step_ms);
    memcpy(p+108,tx->data,16); vcu_log_put32(p+124,tx->completed_ms);
    vcu_log_put32(p+128,vcu_can_port_esr());
    vcu_log_put32(p+132,vcu_can_port_tx_error_count());
    vcu_log_put32(p+136,vcu_can_port_rx_overflow_count());
    vcu_log_put32(p+140,vcu_can_port_rx_error_count());
    vcu_log_put32(p+144,dropped); vcu_log_put16(p+148,(uint16_t)count);
    vcu_log_put16(p+150,(uint16_t)highwater);
    vcu_log_put32(p+152,vcu_can_port_error_event_overflow_count());
    vcu_log_put32(p+156,m->controller[0].command_id); vcu_log_put32(p+160,m->controller[1].command_id);
    vcu_log_put32(p+164,vcu_can_port_tx_log_dropped());
    vcu_log_put32(p+168,rc->uart_overflow_count); p[172]=nonce != 0U;
    vcu_log_put16(p+174,vcu_log_crc(p,174));
    count++; if(count>highwater) highwater=count;
    pending_command=0;
}
void vcu_flight_log_step(uint32_t now, const vcu_crsf_uart_state_t *rc,
                        const vcu_drive_runtime_state_t *d, const vcu_motor_can_state_t *m) {
    vcu_can_tx_observation_t tx;
    vcu_can_port_frame_t frame;
    uint8_t flags=(rc->link_ok?1U:0U) | (d->preview.arm_requested?2U:0U) |
        (d->preview.reset_requested?4U:0U) | (d->requested_run?8U:0U) |
        (((d->state==VCU_DRIVE_STATE_ARMED && d->reason==VCU_DRIVE_REASON_RUNNING) ||
          d->state==VCU_DRIVE_STATE_BRAKING)?16U:0U);
    bool event, failure, periodic;
    vcu_can_port_get_tx_observation(&tx);
    failure=tx.sequence && !tx.success;
    event=previous_state != d->state || previous_reason != d->reason || previous_flags != flags ||
        previous_failure != failure || pending_command != 0U;
    periodic = !started || (uint32_t)(now-last_sample)>=VCU_LOG_SAMPLE_MS;
    if (periodic || event) {
        /* An event coincident with a periodic deadline remains a periodic
         * record, so event storms cannot steal the dedicated sample ring. */
        capture(now,rc,d,m,&tx,flags,event && !periodic);
        /* Events do not move the periodic sample deadline. */
        if(!started || (uint32_t)(now-last_sample)>=VCU_LOG_SAMPLE_MS) last_sample=now;
        started=true; previous_state=(uint8_t)d->state; previous_reason=(uint8_t)d->reason;
        previous_flags=flags; previous_failure=failure;
    }
    /* Six frames / 10 ms; no accumulated burst, no waits. Control and existing
     * telemetry have already run. send_log reserves two motor mailboxes. */
    if ((uint32_t)(now-slot_ms)>=10U) { slot_ms=now; tokens=6; }
    if (!tokens || frame_ms==now) return;
    if(!tx_pending) tx_pending=vcu_can_port_take_tx_log(tx_data);
    memset(&frame,0,sizeof(frame)); frame.data_length=8;
    /* Alternate TX outcomes and snapshot fragments to avoid starvation. */
    if(tx_pending && ((tokens & 1U) || !count)) {
        frame.id=VCU_LOG_TX_ID; memcpy(frame.data,tx_data,8);
        if(vcu_can_port_send_log(&frame)) { tx_pending=false; tokens--; frame_ms=now; }
    } else if(count) {
        frame.id=VCU_LOG_CAN_ID; vcu_log_fragment(records[head],(uint8_t)part,frame.data);
        if(vcu_can_port_send_log(&frame)) {
            tokens--; frame_ms=now;
            if(++part==VCU_LOG_PARTS) { part=0; head=(head+1U)%RECORD_QUEUE; count--; }
        }
    }
}
