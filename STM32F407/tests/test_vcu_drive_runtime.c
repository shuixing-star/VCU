#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vcu_drive_runtime.h"

static vcu_crsf_uart_state_t rc;
static vcu_can_port_frame_t pair[VCU_MOTOR_COUNT];
static uint32_t now;
static uint8_t heartbeat[VCU_MOTOR_COUNT];
static bool tx_ok;
static unsigned int tx_count;

bool vcu_can_port_init(void) { return true; }
bool vcu_can_port_send_pair(const vcu_can_port_frame_t *left,
                            const vcu_can_port_frame_t *right)
{
    tx_count++;
    pair[0] = *left;
    pair[1] = *right;
    return tx_ok;
}
static vcu_drive_runtime_state_t state(void)
{
    vcu_drive_runtime_state_t result;
    assert(vcu_drive_runtime_get_state(&result));
    return result;
}
/* Exercise the real parser and paired command encoder; only the CAN port is mocked. */
static void feedback(unsigned int side, uint8_t f0, uint8_t f1, uint8_t f2)
{
    vcu_can_port_frame_t frame = {
        .id = side == 0 ? VCU_MOTOR_LEFT_ELECTRICAL_STATUS_ID :
                          VCU_MOTOR_RIGHT_ELECTRICAL_STATUS_ID,
        .extended_id = true, .data_length = 8U,
        .data = {0xE8, 0x03, 0x00, 0x7D, 0x00, 0x7D, 0x00, 0x7D}
    };
    assert(vcu_motor_can_process_frame(&frame, now));
    frame.id += VCU_MOTOR_SECOND_STATUS_ID_OFFSET;
    memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = frame.data[1] = 65U;
    frame.data[3] = f0; frame.data[4] = f1; frame.data[5] = f2;
    frame.data[7] = ++heartbeat[side];
    assert(vcu_motor_can_process_frame(&frame, now));
}
static void step(void)
{
    vcu_motor_can_state_t motors;
    assert(vcu_motor_can_get_state(&motors));
    vcu_drive_runtime_step(now, &rc, &motors);
}
static void tick(uint8_t left_fault, uint8_t right_fault)
{
    now += 5U;
    feedback(0, left_fault, 0, 0);
    feedback(1, right_fault, 0, 0);
    step();
}
static void disabled(void)
{
    for (unsigned int side = 0; side < VCU_MOTOR_COUNT; side++) {
        assert((pair[side].data[4] & MX_CAN_COMMAND_RUN_MASK) == 0U);
        assert(pair[side].data[2] == 0x00U && pair[side].data[3] == 0x7DU);
    }
}
static void init_running(void)
{
    now = 100U; tx_ok = true; tx_count = 0U;
    memset(heartbeat, 0, sizeof(heartbeat));
    memset(&rc, 0, sizeof(rc));
    rc.link_ok = true;
    for (unsigned int i = 0; i < VCU_CRSF_CHANNEL_COUNT; i++) rc.channels[i] = 992U;
    rc.channels[6] = 1811U; /* CH7 arm, CH8 mid: no reset */
    assert(vcu_motor_can_init());
    assert(vcu_drive_runtime_init());
    for (unsigned int i = 0; i < 104U; i++) tick(0, 0);
    rc.channels[2] = 1400U;
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_RUNNING);
    for (unsigned int i = 0; i < 4U; i++) tick(0, 0);
    assert(state().latest_left_target > 0);
}
static void recover_and_drive(void)
{
    tick(0, 0);
    assert(!state().fault_latched);
    assert(state().reason == VCU_DRIVE_REASON_WAITING_NEUTRAL);
    disabled();
    rc.channels[2] = 992U;
    tick(0, 0); /* Returning to neutral arms in this very control cycle. */
    assert(state().state == VCU_DRIVE_STATE_ARMED);
    disabled();
    rc.channels[2] = 1400U;
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_RUNNING);
    assert(state().latest_left_target > 13);
    assert(state().latest_left_target == state().sync_output.left_target_rpm);
    for (unsigned int side = 0; side < VCU_MOTOR_COUNT; side++)
        assert((pair[side].data[4] & MX_CAN_COMMAND_RUN_MASK) != 0U);
    assert(state().reset_tx_count == 0U);
}
static void all_motor_faults_recover(void)
{
    const unsigned int bits[] = {8U, 3U, 6U};
    for (unsigned int side = 0; side < 2U; side++) {
        for (unsigned int group = 0; group < 3U; group++) {
            for (unsigned int bit = 0; bit < bits[group]; bit++) {
                uint8_t faults[3] = {0, 0, 0};
                init_running();
                faults[group] = (uint8_t)(1U << bit);
                now += 5U;
                feedback(side, faults[0], faults[1], faults[2]);
                feedback(1U - side, 0, 0, 0);
                step();
                assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
                disabled();
                recover_and_drive();
            }
        }
    }
}
static void persistent_and_dual_faults(void)
{
    init_running();
    for (unsigned int i = 0; i < 120U; i++) {
        unsigned int before = tx_count;
        tick(1, 0x80);
        assert(tx_count > before); /* disabled pairs continue during the fault */
        assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
        disabled();
    }
    tick(0, 0x80);
    assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
    disabled();
    recover_and_drive();
}
static void offline_faulted_side(void)
{
    init_running();
    tick(1, 0);
    for (unsigned int i = 0; i < 140U; i++) {
        now += 5U;
        feedback(1, 0, 0, 0);
        step();
        disabled();
    }
    assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
    recover_and_drive();
}
static void stale_clear_and_frozen_heartbeat(void)
{
    init_running();
    tick(1, 0);
    now += 5U;
    feedback(0, 0, 0, 0);
    now += 501U; /* cached clear expires before the control step */
    feedback(1, 0, 0, 0);
    step();
    assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
    for (unsigned int i = 0; i < 120U; i++) {
        now += 5U;
        heartbeat[0]--; /* same heartbeat, despite new status frames */
        feedback(0, 0, 0, 0);
        feedback(1, 0, 0, 0);
        step();
        assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
        disabled();
    }
    recover_and_drive();
}
static void stop_link_and_arm_gates(void)
{
    init_running();
    tick(1, 0);
    vcu_drive_runtime_request_stop();
    rc.channels[2] = 992U;
    for (unsigned int i = 0; i < 120U; i++) {
        tick(0, 0);
        assert(state().reason == VCU_DRIVE_REASON_STOPPED);
        disabled();
    }
    vcu_drive_runtime_request_run(5U, 2550U);
    rc.link_ok = false;
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_CRSF_LOST);
    disabled();
    rc.link_ok = true; rc.channels[6] = 172U;
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_KILL);
    disabled();
    rc.channels[6] = 1811U; rc.channels[2] = 1400U;
    recover_and_drive();
}
static void single_controller_recovery(void)
{
    init_running();
    /* Restart with no feedback ever received from the right controller. */
    assert(vcu_motor_can_init());
    assert(vcu_drive_runtime_init());
    rc.channels[2] = 992U;
    for (unsigned int i = 0; i < 104U; i++) {
        now += 5U;
        feedback(0, 0, 0, 0);
        step();
    }
    rc.channels[2] = 1400U;
    now += 5U; feedback(0, 1, 0, 0); step();
    assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
    now += 5U; feedback(0, 0, 0, 0); step();
    assert(state().reason == VCU_DRIVE_REASON_WAITING_NEUTRAL);
    rc.channels[2] = 992U;
    for (unsigned int i = 0; i < 102U; i++) {
        now += 5U; feedback(0, 0, 0, 0); step();
    }
    rc.channels[2] = 1400U;
    now += 5U; feedback(0, 0, 0, 0); step();
    assert(state().reason == VCU_DRIVE_REASON_RUNNING);
    assert(state().latest_left_target > 0);
    assert(state().latest_right_target == 0);
}
static void only_throttle_neutral_is_required(void)
{
    init_running();
    tick(1, 0);
    rc.channels[0] = 1400U; /* Steering may remain away from center. */
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_WAITING_NEUTRAL);
    disabled();
    rc.channels[2] = 992U;
    tick(0, 0);
    assert(state().state == VCU_DRIVE_STATE_ARMED);
    disabled(); /* Rearm cycle still sends a disabled zero, not a steering command. */
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_RUNNING);
}
static void internal_faults_auto_recover(void)
{
    init_running();
    tx_ok = false;
    tick(0, 0); /* A failed running command is a CAN failure, not a math failure. */
    assert(state().reason == VCU_DRIVE_REASON_CAN_TX_FAILURE);
    for (unsigned int i = 0; i < 120U; i++) {
        tick(0, 0);
        assert(state().state == VCU_DRIVE_STATE_FAULT);
        assert(!state().fault_latched && state().requested_run);
        assert(state().latest_left_target == 0);
        disabled();
    }
    tx_ok = true;
    recover_and_drive(); /* High throttle cannot restart; neutral then resumes without a dwell. */

    init_running();
    tick(1, 0);
    tx_ok = false; tick(1, 0);
    tx_ok = true; tick(1, 0);
    assert(state().reason == VCU_DRIVE_REASON_MOTOR_FAULT);
    disabled();
    recover_and_drive();

    init_running();
    now += 30U;
    vcu_drive_runtime_report_schedule_overrun(now);
    assert(state().reason == VCU_DRIVE_REASON_SCHEDULE_OVERRUN);
    disabled();
    recover_and_drive();

    init_running();
    now += 200U; /* Sync feedback expired; controller is still within online timeout. */
    step();
    assert(state().reason == VCU_DRIVE_REASON_CONTROL_FAILURE);
    disabled();
    recover_and_drive();
}
static void can_recovery_preserves_stop_and_other_gates(void)
{
    init_running();
    tx_ok = false; tick(0, 0);
    vcu_drive_runtime_request_stop();
    tick(0, 0);
    assert(!state().requested_run);
    tx_ok = true;
    rc.channels[2] = 992U;
    for (unsigned int i = 0; i < 120U; i++) {
        tick(0, 0);
        assert(state().reason == VCU_DRIVE_REASON_STOPPED);
        disabled();
    }
    vcu_drive_runtime_request_run(5U, 0U);
    rc.link_ok = false;
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_CRSF_LOST); disabled();
    rc.link_ok = true; rc.channels[6] = 172U;
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_KILL); disabled();
    rc.channels[6] = 1811U;
    assert(vcu_motor_can_init()); /* No fresh controller feedback. */
    now += 5U; step();
    assert(state().reason == VCU_DRIVE_REASON_MOTOR_TIMEOUT); disabled();
    rc.channels[2] = 1400U;
    tick(0, 0); /* First heartbeat is only a baseline, not proof it is advancing. */
    assert(state().reason == VCU_DRIVE_REASON_MOTOR_TIMEOUT); disabled();
    recover_and_drive();
}
static void failed_zero_blocks_rearming(void)
{
    init_running();
    tx_ok = false; tick(0, 0);
    rc.channels[2] = 992U;
    for (unsigned int i = 0; i < 10U; i++) {
        tick(0, 0);
        assert(state().reason == VCU_DRIVE_REASON_CAN_TX_FAILURE);
        disabled();
    }
    tx_ok = true;
    tick(0, 0);
    assert(state().state == VCU_DRIVE_STATE_ARMED); disabled();
    assert(state().reset_tx_count == 0U);
}
static void explicit_reset_retries_without_drive(void)
{
    init_running();
    vcu_drive_runtime_request_reset();
    tx_ok = false; tick(0, 0); disabled();
    tx_ok = true; tick(0, 0); disabled();
    assert(state().reset_tx_count == 0U);
    tx_ok = false; tick(0, 0);
    assert((pair[0].data[4] & MX_CAN_COMMAND_RUN_MASK) == 0U);
    tx_ok = true; tick(0, 0);
    assert(state().reset_tx_count == 1U && !state().fault_latched);
    tick(0, 0);
    assert(state().reason == VCU_DRIVE_REASON_WAITING_NEUTRAL); disabled();
}
static void controller_owns_speed_ramp(void)
{
    init_running();
    assert(vcu_motor_can_init());
    assert(vcu_drive_runtime_init());
    rc.channels[2] = 992U;
    for (unsigned int i = 0; i < 104U; i++) {
        now += 5U; feedback(0, 0, 0, 0); step();
    }
    /* An old gateway ramp parameter cannot re-enable a VCU speed ramp. */
    vcu_drive_runtime_request_run(5U, 100U);
    assert(state().acceleration_rpm_s == 0U);
    const uint16_t inputs[] = {1811U, 172U, 992U};
    const int16_t expected[] = {5000, -5000, 0};
    for (unsigned int i = 0; i < 3U; i++) {
        rc.channels[2] = inputs[i];
        now += 5U; feedback(0, 0, 0, 0); step();
        assert(state().reason == VCU_DRIVE_REASON_RUNNING);
        assert(state().latest_left_target == expected[i]);
        assert(state().latest_right_target == 0); /* Offline-side gate retained. */
        /* Decode the actual CAN speed word, including left direction reversal. */
        int32_t left_wire = (int32_t)((uint16_t)pair[0].data[2] |
                                      ((uint16_t)pair[0].data[3] << 8)) - 32000;
        assert(left_wire == -expected[i]);
        assert((pair[0].data[4] & MX_CAN_COMMAND_RUN_MASK) != 0U);
    }
    vcu_drive_runtime_request_stop();
    now += 5U; feedback(0, 0, 0, 0); step();
    disabled();
    assert(state().reason == VCU_DRIVE_REASON_STOPPED);
}
static void expect_brake(unsigned side, unsigned current){
    assert(pair[side].data[0]==0 && pair[side].data[1]==0x7d);
    assert(pair[side].data[2]==0 && pair[side].data[3]==0x7d);
    assert(pair[side].data[4]==(current ? 1U : 0U));
    assert(((unsigned)pair[side].data[5] | ((unsigned)pair[side].data[6]<<8))==current);
}
static void ch6_brake(void){
    vcu_rc_drive_config_t cfg; vcu_rc_drive_preview_t p;
    init_running(); vcu_rc_drive_get_default_config(&cfg);
    assert(cfg.brake_channel==6 && cfg.arm_channel==7);
    rc.channels[5]=1499; tick(0,0); assert(state().reason==VCU_DRIVE_REASON_RUNNING);
    rc.channels[5]=1500; tick(0,0);
    assert(state().state==VCU_DRIVE_STATE_BRAKING && state().reason==VCU_DRIVE_REASON_BRAKING);
    assert(!state().preview.drive_allowed && state().preview.brake_requested);
    expect_brake(0,VCU_MOTOR_BRAKE_REQUEST_A); expect_brake(1,VCU_MOTOR_BRAKE_REQUEST_A);
    assert(state().latest_left_target==0 && state().latest_right_target==0);
    uint8_t hb=pair[0].data[7]; rc.channels[2]=172; rc.channels[0]=1811;
    tick(0,0);expect_brake(0,VCU_MOTOR_BRAKE_REQUEST_A);assert(pair[0].data[7]==(uint8_t)(hb+1));
    rc.channels[5]=992;tick(0,0);assert(state().reason==VCU_DRIVE_REASON_WAITING_NEUTRAL);disabled();
    rc.channels[2]=992;tick(0,0);assert(state().state==VCU_DRIVE_STATE_ARMED);disabled();
    rc.channels[2]=1400;tick(0,0);assert(state().reason==VCU_DRIVE_REASON_RUNNING);
    for(unsigned fault=0;fault<6;fault++){
        init_running();rc.channels[5]=1811;tick(0,0);
        if(fault==0)rc.link_ok=false;
        if(fault==1)rc.channels[6]=172;
        if(fault==2)vcu_drive_runtime_request_stop();
        if(fault==3)rc.channels[7]=172;
        if(fault==4){tick(1,0);disabled();continue;}
        if(fault==5){now+=501;step();disabled();continue;}
        tick(0,0);disabled();
    }
    init_running();rc.channels[5]=1811;tx_ok=false;tick(0,0);
    assert(state().reason==VCU_DRIVE_REASON_CAN_TX_FAILURE);disabled();
    tx_ok=true;tick(0,0);expect_brake(0,VCU_MOTOR_BRAKE_REQUEST_A);
    // Healthy side brakes; stale side receives a disabled command.
    for(unsigned i=0;i<103;i++){now+=5;feedback(0,0,0,0);step();}
    expect_brake(0,VCU_MOTOR_BRAKE_REQUEST_A);expect_brake(1,0);
    cfg.brake_channel=0;assert(!vcu_rc_drive_preview_from_channels(rc.channels,true,&cfg,&p));
    cfg.brake_channel=17;assert(!vcu_rc_drive_preview_from_channels(rc.channels,true,&cfg,&p));
    assert(vcu_motor_can_send_brake(200,25));expect_brake(0,200);expect_brake(1,25);
    unsigned n=tx_count;assert(!vcu_motor_can_send_brake(4001,1));assert(tx_count==n);
    puts("CH6 threshold, brake priority, release, fault/kill/link gates, paired units: PASS");
}
int main(void)
{
    ch6_brake();
    puts("controller_owns_speed_ramp"); fflush(stdout);
    controller_owns_speed_ramp();
    puts("all_motor_faults_recover"); fflush(stdout);
    all_motor_faults_recover();
    puts("persistent_and_dual_faults"); fflush(stdout);
    persistent_and_dual_faults();
    puts("offline_faulted_side"); fflush(stdout);
    offline_faulted_side();
    puts("stale_clear_and_frozen_heartbeat"); fflush(stdout);
    stale_clear_and_frozen_heartbeat();
    puts("stop_link_and_arm_gates"); fflush(stdout);
    stop_link_and_arm_gates();
    puts("single_controller_recovery"); fflush(stdout);
    single_controller_recovery();
    puts("only_throttle_neutral_is_required"); fflush(stdout);
    only_throttle_neutral_is_required();
    puts("internal_faults_auto_recover"); fflush(stdout);
    internal_faults_auto_recover();
    puts("can_recovery_preserves_stop_and_other_gates"); fflush(stdout);
    can_recovery_preserves_stop_and_other_gates();
    puts("failed_zero_blocks_rearming"); fflush(stdout);
    failed_zero_blocks_rearming();
    puts("explicit_reset_retries_without_drive"); fflush(stdout);
    explicit_reset_retries_without_drive();
    puts("VCU drive runtime recovery tests passed");
    return 0;
}
