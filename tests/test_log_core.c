#include "vcu_log_store.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static vcu_log_store_t store;
static uint8_t record[VCU_LOG_SIZE];
static void sample(uint32_t seq,uint32_t ms,uint32_t nonce) {
    unsigned i;uint8_t data[8];
    memset(record,0,sizeof(record));record[0]=0xa7;record[1]=1;record[2]=1;
    vcu_log_put32(record+4,seq);vcu_log_put32(record+8,ms);vcu_log_put32(record+12,nonce);
    vcu_log_put16(record+174,vcu_log_crc(record,174));
    for(i=0;i<VCU_LOG_PARTS;i++){vcu_log_fragment(record,(uint8_t)i,data);vcu_log_store_frame(&store,VCU_LOG_CAN_ID,data,8,false,false,ms);}
}
int main(void) {
    uint8_t fragment[8],fault[8]={40,40,0,1,0,0,0,1},tx[8]={0};
    unsigned i;
    assert(vcu_log_crc((const uint8_t *)"123456789",9)==0x29b1);
    vcu_log_store_init(&store,42);
    for(i=0;i<120;i++)sample(i+1,i*100,99);
    assert(store.sample_total==120 && !store.triggered);
    vcu_log_store_frame(&store,0x1802D0EF,fault,8,true,false,12000);
    fault[3]=0;vcu_log_store_frame(&store,0x1802D0EF,fault,8,true,false,12001);
    assert(store.raw_total==2 && store.raw[0].data[3]==1 && store.triggered);
    for(i=120;i<150;i++)sample(i+1,i*100,99);
    vcu_log_store_tick(&store,15000);assert(store.frozen);
    unsigned in_window=0;
    for(i=60;i<150;i++) if(vcu_log_in_window(&store,store.samples[i%VCU_LOG_SAMPLES].rx_ms))in_window++;
    assert(in_window==80);
    sample(151,15100,99);assert(store.sample_total==150 && store.frozen_discards);
    puts("PASS 5 s pre / 3 s post, fault-clear raw retained, frozen capture protected");
    vcu_log_store_init(&store,42);sample(1,0xfffff000U,99);sample(3,0xfffff064U,99);assert(store.gaps==1);
    sample(3,0xfffff064U,99);assert(store.gaps==1);
    sample(1,0xfffff100U,100);assert(store.reboot_count==1 && store.triggered);
    vcu_log_store_tick(&store,0x00002000U);assert(store.frozen);
    puts("PASS wraparound, missing sequence, duplicate and STM reboot detection");
    vcu_log_store_init(&store,1);sample(1,0,1);
    vcu_log_fragment(record,0,fragment);vcu_log_store_frame(&store,VCU_LOG_CAN_ID,fragment,8,false,false,100);
    vcu_log_fragment(record,2,fragment);vcu_log_store_frame(&store,VCU_LOG_CAN_ID,fragment,8,false,false,101);
    sample(2,200,1);assert(store.decoder.incomplete==1);
    record[24]^=1;
    for(i=0;i<VCU_LOG_PARTS;i++){vcu_log_fragment(record,(uint8_t)i,fragment);vcu_log_store_frame(&store,VCU_LOG_CAN_ID,fragment,8,false,false,300);}
    assert(store.decoder.invalid==1 && store.sample_total==2);
    vcu_log_store_tick(&store,1000);assert(store.triggered);
    puts("PASS fragment loss / CRC rejection / link-loss trigger");
    vcu_log_store_init(&store,1);tx[4]=31;
    for(i=0;i<4000;i++){vcu_log_put16(tx,(uint16_t)i);vcu_log_store_frame(&store,VCU_LOG_TX_ID,tx,8,false,false,i);}
    assert(store.tx_total==4000 && store.tx_overwrites==4000-VCU_LOG_TX_ROWS && !store.tx_gaps);
    puts("PASS bounded TX history and counters");
    printf("store_bytes=%u\n",(unsigned)sizeof(store));
    return 0;
}
