#include "vcu_parking_sample.h"
#include <math.h>
_Static_assert(VCU_LOG_FIELD_COUNT == 94U, "Review parking indices when changing legacy fields");
static const vcu_log_field_t extra_fields[116] = {
    {"parking_schema", "Parking log schema", "", 0},
    {"rc_age_ms", "RC telemetry age", "ms", 0},
    {"arm_raw", "CH7 raw", "", 0},
    {"rc_flags", "RC flags", "", 0},
    {"left_cmd_age_ms", "Left Command age", "ms", 0},
    {"left_cmd_fresh", "Left Command fresh", "", 0},
    {"left_cmd_kind", "Left Command kind", "", 0},
    {"left_cmd_count", "Left Command frames", "", 0},
    {"left_cmd_changes", "Left Command changes", "", 0},
    {"left_brake_cmd_n", "Left Brake commands", "", 0},
    {"left_park_cmd_n", "Left PARK commands", "", 0},
    {"left_bad_cmd_n", "Left Invalid commands", "", 0},
    {"left_cmd_len", "Left Command length", "", 0},
    {"left_cmd_torque_raw", "Left Torque raw LE16", "", 0},
    {"left_cmd_speed_raw", "Left Speed raw LE16", "", 0},
    {"left_cmd_control", "Left Command D4", "", 0},
    {"left_cmd_brake_raw", "Left Brake raw LE16", "", 0},
    {"left_cmd_hb", "Left Command heartbeat", "", 0},
    {"left_brake_age_ms", "Left Last brake age", "ms", 0},
    {"left_park_age_ms", "Left Last PARK age", "ms", 0},
    {"left_status_age_ms", "Left Status2 age", "ms", 0},
    {"left_status_fresh", "Left Status2 fresh", "", 0},
    {"left_status_bits", "Left Status2 D2", "", 0},
    {"left_park_reported", "Left PARK reported", "", 0},
    {"left_hold_reported", "Left HOLD reported", "", 0},
    {"left_hold_entries", "Left HOLD entries", "", 0},
    {"left_status_fault_bits", "Left Status2 faults LE32", "", 0},
    {"left_bad_status_n", "Left Bad status2 frames", "", 0},
    {"left_actual_state", "Left Actual K2 state", "", 0},
    {"left_actual_mode", "Left Actual K2 mode", "", 0},
    {"left_brake_active", "Left Brake mode active", "", 0},
    {"left_park_active", "Left PARK mode active", "", 0},
    {"left_neutral_done", "Left Neutral stop done", "", 0},
    {"left_neutral_released", "Left Neutral released", "", 0},
    {"right_cmd_age_ms", "Right Command age", "ms", 0},
    {"right_cmd_fresh", "Right Command fresh", "", 0},
    {"right_cmd_kind", "Right Command kind", "", 0},
    {"right_cmd_count", "Right Command frames", "", 0},
    {"right_cmd_changes", "Right Command changes", "", 0},
    {"right_brake_cmd_n", "Right Brake commands", "", 0},
    {"right_park_cmd_n", "Right PARK commands", "", 0},
    {"right_bad_cmd_n", "Right Invalid commands", "", 0},
    {"right_cmd_len", "Right Command length", "", 0},
    {"right_cmd_torque_raw", "Right Torque raw LE16", "", 0},
    {"right_cmd_speed_raw", "Right Speed raw LE16", "", 0},
    {"right_cmd_control", "Right Command D4", "", 0},
    {"right_cmd_brake_raw", "Right Brake raw LE16", "", 0},
    {"right_cmd_hb", "Right Command heartbeat", "", 0},
    {"right_brake_age_ms", "Right Last brake age", "ms", 0},
    {"right_park_age_ms", "Right Last PARK age", "ms", 0},
    {"right_status_age_ms", "Right Status2 age", "ms", 0},
    {"right_status_fresh", "Right Status2 fresh", "", 0},
    {"right_status_bits", "Right Status2 D2", "", 0},
    {"right_park_reported", "Right PARK reported", "", 0},
    {"right_hold_reported", "Right HOLD reported", "", 0},
    {"right_hold_entries", "Right HOLD entries", "", 0},
    {"right_status_fault_bits", "Right Status2 faults LE32", "", 0},
    {"right_bad_status_n", "Right Bad status2 frames", "", 0},
    {"right_actual_state", "Right Actual K2 state", "", 0},
    {"right_actual_mode", "Right Actual K2 mode", "", 0},
    {"right_brake_active", "Right Brake mode active", "", 0},
    {"right_park_active", "Right PARK mode active", "", 0},
    {"right_neutral_done", "Right Neutral stop done", "", 0},
    {"right_neutral_released", "Right Neutral released", "", 0},
    {"brake_diag_version", "Brake diagnostic version", "", 0},
    {"brake_diag_age_ms", "Brake diagnostic age", "ms", 0},
    {"brake_diag_fresh", "Brake diagnostic fresh", "", 0},
    {"ch6_raw", "CH6 used raw", "", 0},
    {"ch8_raw", "CH8 used raw", "", 0},
    {"brake_input_valid", "STM input valid", "", 0},
    {"brake_link", "STM RC link", "", 0},
    {"brake_arm", "STM CH7 arm", "", 0},
    {"brake_reset", "STM CH8 reset request", "", 0},
    {"brake_run", "STM run request", "", 0},
    {"brake_requested", "STM CH6 brake request", "", 0},
    {"brake_canbox_test", "STM CAN box test build", "", 0},
    {"brake_reason", "STM sampled control reason", "", 0},
    {"brake_tx_valid", "STM pair attempt seen", "", 0},
    {"brake_tx_queued", "STM pair mailbox mask", "", 0},
    {"brake_tx_ok", "STM pair TXOK mask", "", 0},
    {"brake_tx_success", "STM pair success", "", 0},
    {"brake_tx_seq8", "STM pair sequence low 8 bits", "", 0},
    {"left_mos_temp_c", "Left MOS temperature", "degC", 0},
    {"left_motor_temp_c", "Left motor temperature", "degC", 0},
    {"right_mos_temp_c", "Right MOS temperature", "degC", 0},
    {"right_motor_temp_c", "Right motor temperature", "degC", 0},
    {"left_diag_token", "Left Diagnostic token", "", 0},
    {"left_diag_valid", "Left Snapshot present", "", 0},
    {"left_diag_fresh", "Left Snapshot fresh", "", 0},
    {"left_sample_ticks", "Left K2 sample ticks", "", 0},
    {"left_state_flags", "Left K2 raw state flags", "", 0},
    {"left_startup_state", "Left Startup state", "", 0},
    {"left_direction_change_active", "Left Direction change", "", 0},
    {"left_neutral_stop_active", "Left Neutral stop", "", 0},
    {"left_hall_valid", "Left Hall valid", "", 0},
    {"left_hall_speed_valid", "Left Hall speed valid", "", 0},
    {"left_voltage_saturated", "Left Voltage saturated", "", 0},
    {"left_integral_limited", "Left Integral limited", "", 0},
    {"left_hall_feedback_limited", "Left Hall feedback limited", "", 0},
    {"left_handoff_active", "Left Integral release", "", 0},
    {"left_low_recovery_active", "Left Low speed recovery", "", 0},
    {"right_diag_token", "Right Diagnostic token", "", 0},
    {"right_diag_valid", "Right Snapshot present", "", 0},
    {"right_diag_fresh", "Right Snapshot fresh", "", 0},
    {"right_sample_ticks", "Right K2 sample ticks", "", 0},
    {"right_state_flags", "Right K2 raw state flags", "", 0},
    {"right_startup_state", "Right Startup state", "", 0},
    {"right_direction_change_active", "Right Direction change", "", 0},
    {"right_neutral_stop_active", "Right Neutral stop", "", 0},
    {"right_hall_valid", "Right Hall valid", "", 0},
    {"right_hall_speed_valid", "Right Hall speed valid", "", 0},
    {"right_voltage_saturated", "Right Voltage saturated", "", 0},
    {"right_integral_limited", "Right Integral limited", "", 0},
    {"right_hall_feedback_limited", "Right Hall feedback limited", "", 0},
    {"right_handoff_active", "Right Integral release", "", 0},
    {"right_low_recovery_active", "Right Low speed recovery", "", 0},

};
const vcu_log_field_t *vcu_parking_field(unsigned int index) {
    if (index < VCU_LOG_FIELD_COUNT) return &vcu_log_fields[index];
    if (index < VCU_PARKING_FIELD_COUNT) return &extra_fields[index - VCU_LOG_FIELD_COUNT];
    return NULL;
}
const unsigned int vcu_parking_csv_fields[VCU_PARKING_CSV_FIELD_COUNT] = {
    94, 0, 1, 2, 3, 4, 5, 96, 97, 95, 23, 8, 9, 19, 20, 49, 16, 48, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 24, 25, 26, 28, 29, 33, 34, 35, 52, 54, 55, 58, 70, 71, 73, 74, 75, 76, 80, 81, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 36, 37, 38, 40, 41, 45, 46, 47, 62, 64, 65, 68, 82, 83, 85, 86, 87, 88, 92, 93, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
    /* Append only: preserve all schema-4 CSV positions and the Lisp ABI. */
    27, 30, 31, 32, 39, 42, 43, 44,
    180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209
};

void vcu_parking_sample_from_state(const vcu_stm32_gateway_state_t *s,
        const vcu_parking_observer_t observers[2], uint32_t now_ms,
        double v[VCU_PARKING_FIELD_COUNT]) {
    vcu_log_sample_from_state(s, now_ms, v);
    v[94]=5; /* Parking CSV schema, not a controller firmware version. */
    v[95]=s->has_rc_input?(double)(uint32_t)(now_ms-s->frame_last_rx_ms[1]):-1;
    v[96]=s->has_rc_input?s->rc_input.arm_raw:NAN;
    v[97]=s->has_rc_input?s->rc_input.flags:NAN;
    /* Missing/stale diagnostic values stay NaN, never fabricate a CH6=0. */
    for (unsigned i=158;i<176;i++) v[i]=NAN;
    v[159]=s->has_brake_diag?(double)(uint32_t)(now_ms-s->brake_diag_rx_ms):-1;
    v[160]=s->has_brake_diag && (uint32_t)(now_ms-s->brake_diag_rx_ms)<=VCU_BRAKE_DIAG_FRESH_MS;
    if (v[160]) {
        const vcu_brake_diag_t *d=&s->brake_diag;
        v[158]=1;
        v[163]=(d->flags&VCU_BD_INPUT_VALID)!=0;
        if (v[163]) {v[161]=d->ch6;v[162]=d->ch8;}
        v[164]=(d->flags&VCU_BD_LINK)!=0;
        v[165]=(d->flags&VCU_BD_ARM)!=0;
        v[166]=(d->flags&VCU_BD_RESET)!=0;
        v[167]=(d->flags&VCU_BD_RUN)!=0;
        v[168]=(d->flags&VCU_BD_BRAKE)!=0;
        v[169]=(d->flags&VCU_BD_CANBOX)!=0;
        v[170]=d->reason;
        v[171]=(d->tx&VCU_BD_TX_VALID)!=0;
        if (v[171]) {
            v[172]=d->tx&3U;v[173]=(d->tx>>2)&3U;
            v[174]=(d->tx&VCU_BD_TX_SUCCESS)!=0;v[175]=d->tx_seq;
        }
    }
    for (unsigned side=0;side<2;side++) {
        const vcu_parking_observer_t *o=&observers[side];
        const vcu_k2_diag_t *d=&s->k2_diag[side];
        /* SD-only extension. A valid snapshot can be stale; its identity remains
         * visible, but added measurement/state columns must not look current.
         * Raw control_flags retains its historical startup<<16 encoding. */
        const unsigned core=24U+side*12U;
        const unsigned ext=180U+side*15U;
        const bool fresh=d->valid && (uint32_t)(now_ms-d->rx_ms)<=VCU_K2_DIAG_TIMEOUT_MS;
        for (unsigned j=0;j<15U;j++) v[ext+j]=NAN;
        v[ext+1]=d->valid;v[ext+2]=fresh;
        if(d->valid) {v[ext]=d->token;v[ext+3]=d->sample_ticks;}
        if(fresh) {
            v[ext+4]=d->state_flags;v[ext+5]=d->startup;
            v[ext+6]=(d->state_flags>>7)&1U;
            v[ext+7]=(d->state_flags>>8)&1U;
            v[ext+8]=(d->state_flags>>11)&1U;
            v[ext+9]=(d->state_flags>>12)&1U;
            v[ext+10]=(d->state_flags>>15)&1U;
            v[ext+11]=(d->limit_flags&0x0100U)!=0;
            v[ext+12]=(d->limit_flags&0x0200U)!=0;
            v[ext+13]=(d->limit_flags&0x0800U)!=0;
            v[ext+14]=(d->limit_flags&0x1000U)!=0;
        } else {
            /* These legacy slots were absent from the old parking CSV. */
            v[core+3]=NAN;v[core+6]=NAN;v[core+7]=NAN;v[core+8]=NAN;
        }
        unsigned b=98U+side*30U;
        unsigned temp=176U+side*2U;
        v[temp]=NAN;v[temp+1]=NAN;
        for (unsigned j=0;j<30;j++)v[b+j]=NAN;
        v[b]=o->command_seen?(double)(uint32_t)(now_ms-o->command_ms):-1;
        v[b+1]=o->command_seen && (uint32_t)(now_ms-o->command_ms)<=VCU_PARK_OBS_FRESH_MS;
        v[b+3]=o->command_count;v[b+4]=o->command_changes;
        v[b+5]=o->brake_count;v[b+6]=o->park_count;v[b+7]=o->invalid_count;
        v[b+14]=o->brake_count?(double)(uint32_t)(now_ms-o->last_brake_ms):-1;
        v[b+15]=o->park_count?(double)(uint32_t)(now_ms-o->last_park_ms):-1;
        if (o->command_seen) {
            v[b+2]=o->command_kind;v[b+8]=o->command_len;
            if(o->command_len>=2)v[b+9]=(uint16_t)o->command[0]|((uint16_t)o->command[1]<<8);
            if(o->command_len>=4)v[b+10]=(uint16_t)o->command[2]|((uint16_t)o->command[3]<<8);
            if(o->command_len>=5)v[b+11]=o->command[4];
            if(o->command_len>=7)v[b+12]=(uint16_t)o->command[5]|((uint16_t)o->command[6]<<8);
            if(o->command_len>=8)v[b+13]=o->command[7];
        }
        v[b+16]=o->status_seen?(double)(uint32_t)(now_ms-o->status_ms):-1;
        v[b+17]=o->status_seen && (uint32_t)(now_ms-o->status_ms)<=VCU_PARK_OBS_FRESH_MS;
        v[b+21]=o->hold_entries;v[b+23]=o->bad_status_count;
        if(o->status_seen) {
            v[b+18]=o->status[2];
            v[b+22]=(uint32_t)o->status[3]|((uint32_t)o->status[4]<<8)|
                ((uint32_t)o->status[5]<<16)|((uint32_t)o->status[6]<<24);
            if(v[b+17]) {
                /* K2 Status2 D0/D1: integer degC + 40, encoded range 0..250.
                 * Use the same frame/freshness as HOLD; stale temperature is missing.
                 * This protocol has no independent temperature-sensor validity bit. */
                if(o->status[0]<=250U)v[temp]=(int)o->status[0]-40;
                if(o->status[1]<=250U)v[temp+1]=(int)o->status[1]-40;
                v[b+19]=(o->status[2]&9U)==9U;
                v[b+20]=(o->status[2]&0x19U)==0x19U;
            }
        }
        if(d->valid && (uint32_t)(now_ms-d->rx_ms)<=VCU_K2_DIAG_TIMEOUT_MS) {
            unsigned st=d->state_flags&7U, mode=(d->state_flags>>3)&15U;
            v[b+24]=st;v[b+25]=mode;
            v[b+26]=st==1U && mode==7U;v[b+27]=st==1U && mode==8U;
            v[b+28]=(d->state_flags>>9)&1U;v[b+29]=(d->state_flags>>10)&1U;
        }
    }

}
