/* Offline vectors through the production assembler and SD sampler. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vcu_parking_sample.h"
static double v[VCU_PARKING_FIELD_COUNT];
static double val(const char *key) {
    for(unsigned i=0;i<VCU_PARKING_FIELD_COUNT;i++)
        if(!strcmp(vcu_parking_field(i)->key,key)) return v[i];
    assert(0);return 0;
}
static void sample(vcu_stm32_gateway_state_t *s,uint32_t now) {
    vcu_parking_observer_t obs[2]={{0}};
    vcu_parking_sample_from_state(s,obs,now,v);
}
static bool feed(vcu_k2_diag_assembler_t *a,vcu_k2_diag_t *d,unsigned part,
                 uint16_t token,const uint16_t words[12],uint32_t now) {
    uint8_t bytes[8]={(uint8_t)token,(uint8_t)(token>>8)};
    for(unsigned j=0;j<3;j++) {
        uint16_t w=words[part*3+j];bytes[2+j*2]=(uint8_t)w;bytes[3+j*2]=(uint8_t)(w>>8);
    }
    return vcu_k2_diag_accept(a,d,part,bytes,8,now);
}
int main(void) {
    vcu_stm32_gateway_state_t s={0};vcu_k2_diag_assembler_t a[2]={{0}};
    uint16_t w[12]={2000,1728,2186,(uint16_t)-1750,(uint16_t)-1740,
                   (uint16_t)-2300,1030,0x9B00,0xB991,0xFFFF,0xFFFF,0x0309};
    sample(&s,1000);
    assert(val("left_diag_valid")==0 && val("right_diag_fresh")==0);
    assert(isnan(val("left_diag_token")) && isnan(val("left_speed_integral_a")));
    assert(isnan(val("left_startup_state")) && isnan(val("left_limit_flags")));
    unsigned side=9,part=9;
    assert(vcu_k2_diag_route(0x18A3D0F0,true,&side,&part) && side==1 && part==3);
    assert(!vcu_k2_diag_route(0x18A3D0EE,true,&side,&part));
    assert(!vcu_k2_diag_route(0x18A3D0F0,false,&side,&part));
    /* Out of order and interleaved nodes: partials cannot manufacture a sample. */
    assert(!feed(&a[0],&s.k2_diag[0],2,65535,w,1000));
    assert(!feed(&a[1],&s.k2_diag[1],0,44,w,1001));
    assert(!feed(&a[0],&s.k2_diag[0],0,65535,w,1002));
    assert(!feed(&a[0],&s.k2_diag[0],3,65535,w,1003));
    sample(&s,1003);assert(val("left_diag_valid")==0 && val("right_diag_valid")==0);
    assert(feed(&a[0],&s.k2_diag[0],1,65535,w,1004));
    sample(&s,1100);
    assert(val("left_diag_fresh")==1 && val("right_diag_fresh")==0);
    assert(val("left_diag_age_ms")==100 && val("left_diag_token")==65535);
    assert(val("left_sample_ticks")==4294967295.0 && val("left_actual_rpm")==2186);
    assert(val("left_speed_integral_a")==-230.0 && val("left_limit_flags")==0x9B00);
    assert(val("left_control_flags")==0x3B991 && val("left_startup_state")==3);
    assert(val("left_motor_fault")==9 && val("left_direction_change_active")==1);
    /* A repeated complete snapshot must not make old state fresh. */
    for(unsigned i=0;i<4;i++)assert(!feed(&a[0],&s.k2_diag[0],i,65535,w,1101));
    sample(&s,1101);assert(val("left_diag_fresh")==0 && val("left_diag_valid")==1);
    assert(isnan(val("left_speed_integral_a")) && isnan(val("left_handoff_active")));
    assert(val("left_diag_age_ms")==101 && val("left_diag_token")==65535);
    /* Token wraps; incomplete newer data leaves the previous snapshot aged. */
    w[5]=532;w[8]=0x4011;w[11]=0x0200;w[7]=0;
    assert(!feed(&a[0],&s.k2_diag[0],0,0,w,1102));
    sample(&s,1102);assert(val("left_diag_fresh")==0);
    for(unsigned i=1;i<4;i++)feed(&a[0],&s.k2_diag[0],i,0,w,1103);
    sample(&s,1103);assert(val("left_speed_integral_a")==53.2 && val("left_diag_token")==0);
    assert(val("left_startup_state")==2 && val("left_handoff_active")==0);
    assert(val("left_direction_change_active")==0 && val("left_hall_speed_valid")==0);
    /* Each flag decoded independently; unknown limit bits retained in raw word. */
    const char *keys[]={"left_integral_limited","left_hall_feedback_limited","left_handoff_active","left_low_recovery_active"};
    const uint16_t bits[]={0x100,0x200,0x800,0x1000};
    for(unsigned bit=0;bit<4;bit++) {
        s.k2_diag[0].limit_flags=bits[bit]|0x8000;
        sample(&s,1103);assert(val("left_limit_flags")==bits[bit]+0x8000);
        for(unsigned j=0;j<4;j++)assert(val(keys[j])==(j==bit));
    }
    /* Independent sides and fresh K2 while STM32 telemetry is offline. */
    s.k2_diag[1]=s.k2_diag[0];s.k2_diag[1].integral_dA=-321;s.k2_diag[1].rx_ms=1200;
    sample(&s,1204);
    assert(val("stm32_online")==0 && val("left_diag_fresh")==0 && val("right_diag_fresh")==1);
    assert(isnan(val("left_speed_integral_a")) && val("right_speed_integral_a")==-32.1);
    s.k2_diag[1].rx_ms=UINT32_MAX-49;
    sample(&s,50);assert(val("right_diag_fresh")==1 && val("right_diag_age_ms")==100);
    sample(&s,51);assert(val("right_diag_fresh")==0 && isnan(val("right_startup_state")));
    puts("PASS: SD diagnostic values, signed units, missing/partial/out-of-order/interleaved frames, token/time wrap, stale/duplicates, per-bit flags and independent source freshness");
    return 0;
}
