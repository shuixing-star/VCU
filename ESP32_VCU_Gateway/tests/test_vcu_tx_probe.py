"""Compile real ESP gateway receive/assembly path, replacing only RTOS locks/time."""
from pathlib import Path
import subprocess
import tempfile
import argparse
ROOT=Path(__file__).resolve().parents[1]

def run(cc):
    src=(ROOT/'main/vcu_stm32_gateway.c').read_text(encoding='utf-8-sig')
    context=src[src.index('#define VCU_HEARTBEAT_FLAG'):src.index('static uint8_t s_command_sequence;')]
    lookups=src[src.index('static int telemetry_index_from_id'):src.index('static const char *drive_state_name')]
    receive=src[src.index('static bool process_tx_probe'):src.index('bool vcu_stm32_gateway_get_state')]
    prelude='''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vcu_stm32_gateway.h"
#include "vcu_k2_fault.h"
/* SD adapter is covered separately; this fixture checks gateway dispatch. */
static bool vcu_fault_log_receive(uint32_t id,const uint8_t *data,uint8_t len,bool ext,uint32_t now) {
    unsigned side,part;(void)data;(void)len;(void)now;
    return vcu_k2_fault_route(id,ext,&side,&part);
}
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
static uint32_t now_ms(void) { return 10000U; }
'''
    tests='''
static void error(uint8_t seq,uint16_t stamp) {
    vcu_gateway_can_error_t event={seq,VCU_GATEWAY_CAN_ERROR_TX_TIMEOUT,0x40,2,0,0,stamp};
    uint8_t data[8];vcu_gateway_pack_can_error(&event,data);
    assert(vcu_stm32_gateway_process_can_frame(VCU_GATEWAY_ID_CAN_ERROR,data,8,false));
}
static bool fragment(uint8_t seq,uint16_t stamp,uint8_t part,uint32_t value) {
    uint8_t data[8];vcu_gateway_pack_tx_probe(seq,stamp,part,value,data);
    assert(data[0]==seq && data[1]==(stamp&255) && data[2]==(stamp>>8));
    assert(data[3]==part && data[4]==(value&255) && data[7]==(value>>24));
    return vcu_stm32_gateway_process_can_frame(VCU_GATEWAY_ID_CAN_TX_PROBE,data,8,false);
}
int main(void) {
    (void)s_motor_raw_names;
    m_ctx.public_state.initialized=true;
    uint8_t other[8]={0};
    assert(vcu_stm32_gateway_process_can_frame(0x1900D0EF,other,8,true));
    assert(vcu_stm32_gateway_process_can_frame(0x191AD0F0,other,8,true));
    assert(!vcu_stm32_gateway_process_can_frame(0x191BD0F0,other,8,true));
    assert(!vcu_stm32_gateway_process_can_frame(0x500,other,8,true));
    assert(!vcu_stm32_gateway_process_can_frame(0x123,other,2,false));
    assert(m_ctx.public_state.invalid_frame_count==0);
    assert(!vcu_stm32_gateway_process_can_frame(VCU_GATEWAY_ID_CAN_ERROR,other,2,false));
    assert(m_ctx.public_state.invalid_frame_count==1);
    error(1,1250);error(1,3108); /* Same sequence after wrap; different timestamp. */
    for(unsigned p=7;p>0;--p) assert(fragment(1,1250,p-1,0x01020304U+p));
    assert(m_ctx.can_error_log[0].probe_mask==0x7f);
    assert(m_ctx.can_error_log[1].probe_mask==0);
    assert(fragment(1,3108,0,0x12345678));
    assert(fragment(1,3108,0,0x12345678)); /* duplicate is idempotent */
    assert(m_ctx.can_error_log[1].probe_mask==1);
    assert(m_ctx.can_error_log[1].probe[0]==0x12345678);
    assert(!fragment(1,9999,0,0));
    assert(!fragment(1,3108,7,0));
    assert(m_ctx.public_state.can_error_event_count==2);
    assert(m_ctx.public_state.can_error_sequence_gap_count==0);
    for(unsigned i=0;i<64;i++) error((uint8_t)(i+2),(uint16_t)(4000+i));
    assert(m_ctx.can_error_log[0].probe_mask==0);
    assert(m_ctx.can_error_log[1].probe_mask==0);
    assert(!fragment(1,1250,0,0));
    m_ctx.public_state.can_error_stored_count=0; /* clear/reboot loses base event */
    assert(!fragment(65,4063,0,0));
    puts("PASS real ESP parser: event identity, out-of-order/duplicate/missing fragments, ring overwrite, unrelated frames");
    return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix='vcu-probe-rx-') as temp:
        source=Path(temp)/'test.c'; exe=Path(temp)/'test.exe'
        source.write_text(prelude+context+lookups+receive+tests,encoding='utf-8')
        subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'main'),str(source),str(ROOT/'main/vcu_gateway_protocol.c'),str(ROOT/'main/vcu_k2_diag.c'),str(ROOT/'main/vcu_k2_fault.c'),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--cc',default='gcc');run(parser.parse_args().cc)
