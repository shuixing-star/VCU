/* Production passive observer + sample formatter. All frames are simulated. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "vcu_parking_sample.h"
static double values[VCU_PARKING_FIELD_COUNT];
static unsigned field(const char *key) {
    for(unsigned i=0;i<VCU_PARKING_FIELD_COUNT;i++)if(!strcmp(vcu_parking_field(i)->key,key))return i;
    assert(0);return 0;
}
static double value(const char *key){return values[field(key)];}
static void put16(uint8_t *p,unsigned value){p[0]=value;p[1]=value>>8;}
int main(void) {
    assert(VCU_LOG_FIELD_COUNT==94U);
    bool seen[VCU_PARKING_FIELD_COUNT]={false};
    for (unsigned i=0;i<VCU_PARKING_CSV_FIELD_COUNT;i++) {
        unsigned n=vcu_parking_csv_fields[i];
        assert(n<VCU_PARKING_FIELD_COUNT && !seen[n]);seen[n]=true;
        const vcu_log_field_t *f=vcu_parking_field(n);assert(f && f->key && f->name);
        assert(!strpbrk(f->key, ":;\n\r") && !strpbrk(f->name, ":;\n\r"));
    }
    assert(vcu_parking_field(VCU_PARKING_FIELD_COUNT)==NULL);
    vcu_stm32_gateway_state_t s={0};vcu_parking_observer_t observers[2]={{0}};vcu_parking_observer_t *o=&observers[0];
    uint8_t cmd[8]={0,125,0,125,1,0,0,0},status[8]={70,70,25,1,2,3,4,0};
    vcu_parking_sample_from_state(&s,observers,1000,values);
    assert(value("parking_schema")==5 && isnan(value("arm_raw")));
    assert(value("left_cmd_age_ms")==-1 && value("left_cmd_fresh")==0);
    assert(isnan(value("left_cmd_kind")) && isnan(value("left_hold_reported")));
    vcu_parking_observe(o,0,cmd,8,1000);
    assert(o->command_kind==VCU_PARK_CMD_DRIVE&&o->command_changes==1);
    cmd[7]++;vcu_parking_observe(o,0,cmd,8,1001);assert(o->command_changes==1);
    const unsigned strengths[]={1,2,400,1000,65535};
    for(unsigned i=0;i<5;i++){
        put16(cmd+5,strengths[i]);vcu_parking_observe(o,0,cmd,8,1010+i);
        assert(o->command_kind==VCU_PARK_CMD_BRAKE);
    }
    assert(o->brake_count==5);
    cmd[4]=0;vcu_parking_observe(o,0,cmd,8,1020);assert(o->command_kind==VCU_PARK_CMD_STOP);
    cmd[4]=5;vcu_parking_observe(o,0,cmd,8,1020);assert(o->command_kind==VCU_PARK_CMD_RESET);
    put16(cmd+5,0);cmd[4]=9;vcu_parking_observe(o,0,cmd,8,1030);
    assert(o->command_kind==VCU_PARK_CMD_PARK&&o->park_count==1);
    const unsigned bad[]={8,11,13,15,0x89};
    for(unsigned i=0;i<5;i++){
        cmd[4]=bad[i];vcu_parking_observe(o,0,cmd,8,1040+i);
        assert(o->command_kind==VCU_PARK_CMD_INVALID);
    }
    cmd[4]=9;cmd[5]=1;vcu_parking_observe(o,0,cmd,8,1045);assert(o->command_kind==5);
    cmd[5]=0;cmd[0]=1;vcu_parking_observe(o,0,cmd,8,1046);assert(o->command_kind==5);
    cmd[0]=0;cmd[2]=1;vcu_parking_observe(o,0,cmd,8,1047);assert(o->command_kind==5);
    cmd[2]=0;
    for(unsigned len=0;len<8;len++){
        vcu_parking_observe(o,0,cmd,len,1050+len);assert(o->command_kind==5);
    }
    assert(o->invalid_count==16&&o->park_count==1);
    vcu_parking_sample_from_state(&s,observers,1060,values);
    assert(value("left_cmd_len")==7&&isnan(value("left_cmd_hb")));
    assert(value("left_bad_cmd_n")==16&&value("left_park_age_ms")==30);
    /* A one-frame PARK then drive is still visible in the latched count/age. */
    cmd[4]=1;vcu_parking_observe(o,0,cmd,8,1070);
    vcu_parking_sample_from_state(&s,observers,1070,values);
    assert(value("left_cmd_kind")==1&&value("left_park_cmd_n")==1&&value("left_park_age_ms")==40);
    vcu_parking_observe(o,2,status,8,1100);assert(o->hold_entries==1);
    vcu_parking_observe(o,2,status,8,1101);assert(o->hold_entries==1);
    s.k2_diag[0].valid=true;s.k2_diag[0].rx_ms=1100;
    s.k2_diag[0].state_flags=1|(8<<3)|(1<<9)|(1<<10);
    vcu_parking_sample_from_state(&s,observers,1200,values);
    assert(value("left_park_reported")==1&&value("left_hold_reported")==1);
    assert(value("left_actual_state")==1&&value("left_actual_mode")==8&&value("left_park_active")==1);
    assert(value("left_neutral_done")==1&&value("left_neutral_released")==1);
    assert(value("left_status_fault_bits")==0x04030201UL);
    vcu_parking_sample_from_state(&s,observers,1202,values);
    assert(value("left_status_fresh")==0&&isnan(value("left_hold_reported")));
    assert(isnan(value("left_park_active"))&&value("left_hold_entries")==1);
    status[2]=16;vcu_parking_observe(o,2,status,8,1203);vcu_parking_sample_from_state(&s,observers,1203,values);
    assert(value("left_hold_reported")==0); /* HOLD bit alone is insufficient. */
    status[2]=25;vcu_parking_observe(o,2,status,8,1204);assert(o->hold_entries==2);
    vcu_parking_observe(o,2,status,7,1205);assert(o->bad_status_count==1&&o->status_ms==1204);
    uint32_t count=o->command_count;
    vcu_parking_observe(o,1,cmd,8,1230);vcu_parking_observe(o,0,NULL,8,1230);
    vcu_parking_observe(o,0,cmd,9,1230);assert(o->command_count==count);
    o->command_count=UINT32_MAX;vcu_parking_observe(o,0,cmd,8,UINT32_MAX-49);
    vcu_parking_sample_from_state(&s,observers,50,values);
    assert(value("left_cmd_count")==UINT32_MAX&&value("left_cmd_age_ms")==100&&value("left_cmd_fresh")==1);
    vcu_parking_sample_from_state(&s,observers,51,values);assert(value("left_cmd_fresh")==0);
    assert(value("right_cmd_count")==0&&isnan(value("right_cmd_kind")));
    /* Existing 20 ms current windows contain the Id used for static hold. */
    s.k2_diag[0].window_present=true;s.k2_diag[0].window_schema=1;s.k2_diag[0].window_count=200;
    s.k2_diag[0].window_flags=1;s.k2_diag[0].window_end_ticks=1000;
    s.k2_diag[0].sample_ticks=s.k2_diag[0].window_diag_ticks=1100;s.k2_diag[0].window_rx_ms=100;
    s.k2_diag[0].id_ref_mean_dA=4000;s.k2_diag[0].id_mean_dA=3950;
    vcu_parking_sample_from_state(&s,observers,150,values);
    assert(value("left_id_ref_mean_a")==400&&value("left_id_mean_a")==395&&value("left_win_fresh")==1);
    s.k2_diag[0].window_flags=3;vcu_parking_sample_from_state(&s,observers,150,values);
    assert(value("left_win_fresh")==0&&isnan(value("left_id_ref_mean_a")));
    s.has_brake_diag=true;s.brake_diag_rx_ms=UINT32_MAX-49;
    s.brake_diag.flags=VCU_BRAKE_DIAG_VERSION|VCU_BD_INPUT_VALID|VCU_BD_BRAKE;
    s.brake_diag.ch6=1809;s.brake_diag.ch8=992;
    vcu_parking_sample_from_state(&s,observers,100,values);
    assert(value("brake_diag_age_ms")==150 && value("brake_diag_fresh")==1 && value("ch6_raw")==1809);
    assert(value("brake_tx_valid")==0 && isnan(value("brake_tx_ok")));
    vcu_parking_sample_from_state(&s,observers,101,values);
    assert(value("brake_diag_fresh")==0 && isnan(value("ch6_raw")));
    s.brake_diag_rx_ms=101;s.brake_diag.flags=VCU_BRAKE_DIAG_VERSION;
    vcu_parking_sample_from_state(&s,observers,101,values);
    assert(value("brake_input_valid")==0 && isnan(value("ch6_raw")));
    /* Status2 temperature decode, per-side freshness, malformed data and wrap. */
    memset(observers,0,sizeof(observers));
    vcu_parking_sample_from_state(&s,observers,1000,values);
    assert(isnan(value("left_mos_temp_c"))&&isnan(value("right_motor_temp_c")));
    uint8_t thermal_left[8]={65,120,25,0,0,0,0,1};
    uint8_t thermal_right[8]={70,75,25,0,0,0,0,1};
    vcu_parking_observe(&observers[0],2,thermal_left,8,1000);
    vcu_parking_observe(&observers[1],2,thermal_right,8,1050);
    vcu_parking_sample_from_state(&s,observers,1100,values);
    assert(value("left_mos_temp_c")==25&&value("left_motor_temp_c")==80);
    assert(value("right_mos_temp_c")==30&&value("right_motor_temp_c")==35);
    assert(value("left_hold_reported")==1&&value("right_hold_reported")==1);
    vcu_parking_sample_from_state(&s,observers,1101,values);
    assert(isnan(value("left_mos_temp_c"))&&isnan(value("left_motor_temp_c")));
    assert(value("right_mos_temp_c")==30);
    thermal_left[0]=110;thermal_left[1]=115;
    vcu_parking_observe(&observers[0],2,thermal_left,7,1101);
    vcu_parking_sample_from_state(&s,observers,1101,values);
    assert(isnan(value("left_mos_temp_c"))); /* short frame cannot refresh */
    thermal_left[0]=0;thermal_left[1]=250;
    vcu_parking_observe(&observers[0],2,thermal_left,8,UINT32_MAX-49);
    vcu_parking_sample_from_state(&s,observers,50,values);
    assert(value("left_mos_temp_c")==-40&&value("left_motor_temp_c")==210);
    vcu_parking_sample_from_state(&s,observers,51,values);
    assert(isnan(value("left_mos_temp_c")));
    thermal_left[0]=251;thermal_left[1]=255;
    vcu_parking_observe(&observers[0],2,thermal_left,8,60);
    vcu_parking_sample_from_state(&s,observers,60,values);
    assert(isnan(value("left_mos_temp_c"))&&isnan(value("left_motor_temp_c")));
    puts("PASS: passive brake/PARK/invalid/overwrite/HOLD/Id, missing/stale data, counter saturation and time wrap");
    return 0;
}
