#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "can.h"
#include "vcu_can_port.h"

static CAN_TypeDef hw;
test_dwt_t test_dwt;
test_debug_t test_debug;
uint32_t SystemCoreClock = 168000000U;
static bool jump_tick_after_pair;
CAN_HandleTypeDef hcan1 = { &hw, 0 };
static uint64_t us, due[3], abort_due[3];
static bool busy[3], bus_stuck, abort_stuck, empty_without_txok;
static unsigned adds, fail_add, sent, aborted, delay_us, stuck_add, abort_calls, fast_add;
static uint8_t payload[3][8];
static uint8_t sent_data[256][8];
static uint32_t payload_id[3], sent_id[256];
static const uint32_t rqcp[3] = {1U,1U<<8,1U<<16};
static const uint32_t txok[3] = {CAN_TSR_TXOK0,CAN_TSR_TXOK1,CAN_TSR_TXOK2};
static void advance(void) {
    us += 10;
    for (unsigned i=0;i<3;i++) {
        if (!busy[i]) continue;
        if (abort_due[i] && us >= abort_due[i]) {
            busy[i]=false; abort_due[i]=0; aborted++; hw.TSR |= rqcp[i];
        } else if (due[i] && us >= due[i]) {
            busy[i]=false; hw.TSR |= rqcp[i];
            if (!empty_without_txok) hw.TSR |= txok[i];
            sent_id[sent]=payload_id[i];
            memcpy(sent_data[sent++],payload[i],8);
        }
    }
    for (unsigned i=0;i<3;i++) {
        if (busy[i]) hw.TSR &= ~(1UL << (26U+i));
        else hw.TSR |= 1UL << (26U+i);
    }
    test_dwt.CYCCNT = (uint32_t)(us * (SystemCoreClock/1000000U));
}
uint32_t HAL_GetTick(void) {
    advance();
    if (jump_tick_after_pair && adds >= 2) {
        jump_tick_after_pair=false;
        us+=2500; advance();
    }
    return (uint32_t)(us/1000);
}
uint32_t HAL_CAN_GetState(const CAN_HandleTypeDef *h) { (void)h;return 2; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h) { return h->ErrorCode; }
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *h,uint32_t mask) {
    (void)h; advance();
    for(unsigned i=0;i<3;i++) if ((mask&(1U<<i)) && busy[i]) return 1;
    return 0;
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h) {
    (void)h;advance();return !busy[0]+!busy[1]+!busy[2];
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,const CAN_TxHeaderTypeDef *header,uint8_t *data,uint32_t *mailbox) {
    (void)h;(void)header;advance();adds++;
    if(adds==fail_add) return HAL_ERROR;
    for(unsigned i=0;i<3;i++) if(!busy[i]) {
        *mailbox=1U<<i; busy[i]=true;hw.TSR&=~(txok[i]|rqcp[i]|(1UL<<(26U+i)));
        memcpy(payload[i],data,8);abort_due[i]=0;
        payload_id[i]=header->IDE==CAN_ID_EXT?header->ExtId:header->StdId;
        due[i]=(bus_stuck || adds==stuck_add)?0:us+(adds==fast_add?400:delay_us);
        return HAL_OK;
    }
    return HAL_ERROR;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h,uint32_t mask) {
    uint32_t request=0;
    if(mask&1U) request|=CAN_TSR_ABRQ0;
    if(mask&2U) request|=CAN_TSR_ABRQ1;
    if(mask&4U) request|=CAN_TSR_ABRQ2;
    /* Match the real HAL's TSR read/modify/write, including W1C side effects. */
    mock_write_reg(&h->Instance->TSR,h->Instance->TSR|request);
    return HAL_OK;
}
void mock_write_reg(volatile uint32_t *reg, uint32_t value) {
    assert(reg==&hw.TSR);
    const uint32_t abrq[3]={CAN_TSR_ABRQ0,CAN_TSR_ABRQ1,CAN_TSR_ABRQ2};
    advance();abort_calls++;
    for(unsigned i=0;i<3;i++) {
        if(value&rqcp[i]) hw.TSR&=~(txok[i]|rqcp[i]);
        if((value&abrq[i]) && busy[i] && !abort_stuck && !abort_due[i]) abort_due[i]=us+100;
    }
}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h,const CAN_FilterTypeDef *f) {(void)h;(void)f;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) {(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h,uint32_t mask) {(void)h;(void)mask;return HAL_OK;}
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *h,uint32_t fifo) {(void)h;(void)fifo;return 0;}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h,uint32_t fifo,CAN_RxHeaderTypeDef *header,uint8_t *data) {(void)h;(void)fifo;(void)header;(void)data;return HAL_ERROR;}
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) {h->ErrorCode=0;return HAL_OK;}
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *h) {(void)h;}
void HAL_NVIC_DisableIRQ(int irq) {(void)irq;}
void HAL_NVIC_ClearPendingIRQ(int irq) {(void)irq;}
void HAL_NVIC_EnableIRQ(int irq) {(void)irq;}
void HAL_NVIC_SetPriority(int irq,uint32_t a,uint32_t b) {(void)irq;(void)a;(void)b;}
static vcu_can_port_frame_t left = {.id=0x0C01EFD0,.extended_id=true,.data_length=8,.data={1}};
static vcu_can_port_frame_t right = {.id=0x0C01EFD1,.extended_id=true,.data_length=8,.data={2}};
static vcu_can_port_frame_t telemetry = {.id=0x680,.data_length=8,.data={3}};
static void reset(void) {
    us=0;memset(&hw,0,sizeof(hw));memset(busy,0,sizeof(busy));
    memset(due,0,sizeof(due));memset(abort_due,0,sizeof(abort_due));
    bus_stuck=abort_stuck=empty_without_txok=jump_tick_after_pair=false;
    adds=fail_add=sent=aborted=stuck_add=abort_calls=fast_add=0;delay_us=400;
    assert(vcu_can_port_init());
    assert((hw.MCR&CAN_MCR_NART)==0);
}
static void expect_event(vcu_can_port_error_type_t type) {
    vcu_can_port_error_event_t e; bool found=false;
    while(vcu_can_port_take_error_event(&e)) if(e.type==type) found=true;
    assert(found);
}
int main(void) {
    /* Model preemption between the pending read and deadline check. Hardware
     * finishes meanwhile; keep existing fault policy and diagnose the window. */
    reset();delay_us=1000;jump_tick_after_pair=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    vcu_can_port_error_event_t boundary;
    assert(vcu_can_port_take_error_event(&boundary));
    assert(boundary.has_tx_probe);
    assert(boundary.tx_probe[2]==left.id && boundary.tx_probe[3]==right.id);
    assert((boundary.tx_probe[0] & (CAN_TSR_TXOK0|CAN_TSR_TXOK1)) ==
           (CAN_TSR_TXOK0|CAN_TSR_TXOK1));
    assert(boundary.tx_probe[4]>=2500 && boundary.tx_probe[5]>=2500);
    assert(boundary.tx_probe[6]==0x01020101U);
    puts("PASS reproduced deadline window: TX_TIMEOUT despite TXOK at snapshot");
    assert(((boundary.tx_probe[6]>>2)&63U)==0);
    reset();delay_us=1000;jump_tick_after_pair=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(vcu_can_port_take_error_event(&boundary));
    assert(((boundary.tx_probe[6]>>2)&63U)==0);
    puts("PASS bare-metal probe preserves zero task-switch bits");


    reset();bus_stuck=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(vcu_can_port_take_error_event(&boundary));
    assert(boundary.has_tx_probe && (boundary.tx_probe[0] & (3UL<<26))==0);
    assert(boundary.tx_probe[4]>=1900 && boundary.tx_probe[4]<2200);
    assert(boundary.tx_probe[5]<200);
    assert(boundary.tx_probe[1]==0);
    puts("PASS pending-before-abort snapshot differs from completion race");

    reset();empty_without_txok=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(vcu_can_port_take_error_event(&boundary));
    assert((boundary.tx_probe[6]>>24)==2);
    assert((boundary.tx_probe[0] & (3UL<<26))==(3UL<<26));
    puts("PASS completed-without-TXOK has a distinct trigger");

    reset();
    busy[0]=busy[1]=true;due[0]=due[1]=500;
    assert(vcu_can_port_send_pair(&left,&right));
    assert(us>=500 && us<2000 && adds==2);
    assert(vcu_can_port_tx_error_count()==0);
    puts("PASS short mailbox contention recovers without TX fault");

    reset();delay_us=10000;
    assert(vcu_can_port_send(&telemetry));
    us=1050;due[0]=1110;delay_us=1400;
    assert(vcu_can_port_send_pair(&left,&right));
    assert(sent==3 && aborted==0);
    puts("PASS mailbox reused after telemetry completion keeps new owner");

    reset();delay_us=10000;
    assert(vcu_can_port_send(&telemetry));
    us=1050;delay_us=1400;fast_add=2;
    assert(vcu_can_port_send_pair(&left,&right));
    assert(aborted==1 && sent==2);
    puts("PASS telemetry abort preserves other mailbox TXOK completion");

    reset();delay_us=1400;
    assert(vcu_can_port_send_pair(&left,&right));
    assert(sent==2 && us<2000);
    puts("PASS delayed/retried transmission completes inside budget");

    reset();bus_stuck=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(us<3200 && aborted==2 && !busy[0] && !busy[1]);
    expect_event(VCU_CAN_PORT_ERROR_TX_TIMEOUT);
    bus_stuck=false;
    vcu_can_port_frame_t zero=left;memset(zero.data,0,8);
    assert(vcu_can_port_send_pair(&zero,&zero));
    assert(sent==2 && sent_data[0][0]==0 && sent_data[1][0]==0);
    puts("PASS endless retransmission cancelled; safe zero replaces old pair");

    reset();fail_add=2;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(aborted==1 && !busy[0] && sent==0);
    expect_event(VCU_CAN_PORT_ERROR_TX_ADD_FAILED);
    reset();stuck_add=2;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(sent==1 && aborted==1);
    puts("PASS partial enqueue and one-sided completion handled as failures");

    reset();empty_without_txok=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    expect_event(VCU_CAN_PORT_ERROR_TX_TIMEOUT);
    puts("PASS empty mailbox without TXOK is not success");

    reset();delay_us=10000;
    assert(vcu_can_port_send(&telemetry));
    assert(!vcu_can_port_send(&telemetry));
    assert(adds==1 && vcu_can_port_tx_error_count()==0);
    us=2100;vcu_can_port_poll();us+=200;vcu_can_port_poll();
    assert(aborted==1 && !busy[0]);
    expect_event(VCU_CAN_PORT_ERROR_TX_TIMEOUT);
    puts("PASS telemetry reserves two mailboxes and expires without accumulating");

    reset();delay_us=10000;
    assert(!vcu_can_port_send_diagnostic(&telemetry));
    assert(sent==0 && aborted==1 && us<3200);
    vcu_can_port_error_event_t e;assert(!vcu_can_port_take_error_event(&e));
    delay_us=400;
    assert(vcu_can_port_send_diagnostic(&telemetry));
    assert(sent==1);
    reset();empty_without_txok=true;
    assert(!vcu_can_port_send_diagnostic(&telemetry));
    assert(!vcu_can_port_take_error_event(&e));
    puts("PASS diagnostic success needs TXOK; timeout retries without recursive logs");

    reset();bus_stuck=true;abort_stuck=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    unsigned before=adds;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(adds==before && us<6000);
    abort_stuck=false;bus_stuck=false;vcu_can_port_poll();us+=200;
    assert(vcu_can_port_send_pair(&left,&right));
    puts("PASS asynchronous/stuck abort blocks reuse until released");

    reset();us=((uint64_t)UINT32_MAX-1)*1000;delay_us=1400;
    assert(vcu_can_port_send_pair(&left,&right));
    reset();us=((uint64_t)UINT32_MAX-1)*1000;bus_stuck=true;
    assert(!vcu_can_port_send_pair(&left,&right));
    assert(aborted==2);
    puts("PASS tick wrap for success and timeout");

    reset();
    assert(vcu_can_port_send_pair(&left,&right));
    vcu_can_tx_observation_t obs;uint8_t txlog[8];
    vcu_can_port_get_tx_observation(&obs);
    assert(obs.queued_mask==3 && obs.txok_mask==3 && obs.success);
    assert(vcu_can_port_take_tx_log(txlog) && txlog[4]==31);
    reset();fail_add=2;
    assert(!vcu_can_port_send_pair(&left,&right));
    vcu_can_port_get_tx_observation(&obs);
    assert(obs.queued_mask==1 && !obs.success);
    reset();stuck_add=2;
    assert(!vcu_can_port_send_pair(&left,&right));
    vcu_can_port_get_tx_observation(&obs);
    assert(obs.queued_mask==3 && obs.txok_mask==1 && !obs.success);
    puts("PASS admission, one-sided TXOK, and complete pair are distinct");
    reset();delay_us=10000;
    uint64_t begin=us;
    assert(!vcu_can_port_send_diagnostic_async(&telemetry));
    assert(us-begin<500);
    us=3000;vcu_can_port_poll();
    assert(!vcu_can_port_send_diagnostic_async(&telemetry));
    delay_us=100;us+=500;
    assert(!vcu_can_port_send_diagnostic_async(&telemetry));
    us+=500;vcu_can_port_poll();
    assert(vcu_can_port_send_diagnostic_async(&telemetry));
    puts("PASS async diagnostic never waits; failed hardware TX remains retryable");
    return 0;
}
