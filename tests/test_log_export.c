#define VCU_LOG_HOST_TEST
#include "../ESP32_VCU_Gateway/main/vcu_flight_log.c"
#include <assert.h>
static void feed(uint32_t seq,uint32_t ms,bool valid) {
    uint8_t p[VCU_LOG_SIZE]={0},d[8];unsigned i;
    p[0]=0xa7;p[1]=1;p[2]=1;p[47]=valid;p[172]=1;
    vcu_log_put32(p+4,seq);vcu_log_put32(p+8,ms);vcu_log_put32(p+12,55);vcu_log_put32(p+16,VCU_LOG_BUILD_ID);
    if(valid) {
        vcu_log_put16(p+24,1000);vcu_log_put16(p+32,500);vcu_log_put16(p+36,350);vcu_log_put16(p+40,350);
        for(i=0;i<2;i++) {
            uint8_t *q=p+48+24*i;vcu_log_put16(q,720);vcu_log_put16(q+2,32100);
            vcu_log_put16(q+4,32200);vcu_log_put16(q+6,32300);
            q[8]=65;q[9]=70;q[22]=7; q[11]=(seq==3 && i==0)?1:0;
        }
        vcu_log_put32(p+100,seq);p[104]=3;p[105]=seq==3?1:3;p[106]=seq!=3;
        vcu_log_put16(p+110,32350);vcu_log_put16(p+118,32350);
    } else { vcu_log_put16(p+64,65535);vcu_log_put16(p+66,65535);vcu_log_put16(p+68,65535); }
    if(seq>=2) {
        uint8_t t[8]={0};vcu_log_put16(t,(uint16_t)seq);vcu_log_put16(t+2,(uint16_t)ms);
        t[4]=seq==3?7:31;t[5]=t[6]=(uint8_t)seq;t[7]=1;
        vcu_flight_log_receive(VCU_LOG_TX_ID,t,8,false,false,ms);
        uint8_t fault[8]={65,70,0,0,0,0,0,1};fault[3]=seq==3?1:0;
        vcu_flight_log_receive(0x1802D0EF,fault,8,true,false,ms);
    }
    if(seq==3) {p[44]=4;p[45]=5;}
    vcu_log_put16(p+174,vcu_log_crc(p,174));
    for(i=0;i<VCU_LOG_PARTS;i++){vcu_log_fragment(p,(uint8_t)i,d);vcu_flight_log_receive(VCU_LOG_CAN_ID,d,8,false,false,ms);}
}
int main(int argc,char **argv) {
    assert(argc==2);file_basepath=argv[1];
    mock_free_heap=100;vcu_flight_log_init();assert(!ready);
    mock_free_heap=150000;vcu_flight_log_init();assert(ready);
    feed(1,1000,false);feed(2,1100,true);feed(3,1200,true);assert(store->triggered);
    feed(4,1300,true);vcu_log_store_tick(store,4200);assert(store->frozen);
    char *saved=file_basepath;file_basepath="__missing_parent__/";assert(!export_files());
    file_basepath=saved;assert(store->frozen && store->sample_total==4);
    assert(export_files());exported=true;
    printf("SIMULATED export directory: %s\n",last_path);
    terminal_rearm(0,NULL);assert(!store->frozen && store->sample_total==0);
    puts("PASS low-memory refusal, file-error retention/retry, actual CSV writer and rearm");
    return 0;
}
