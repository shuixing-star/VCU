#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdint.h>
typedef struct { volatile uint32_t MCR, ESR, TSR; } CAN_TypeDef;
typedef struct { CAN_TypeDef *Instance; uint32_t ErrorCode; } CAN_HandleTypeDef;
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC; } CAN_RxHeaderTypeDef;
typedef struct { uint32_t FilterBank, FilterMode, FilterScale, FilterFIFOAssignment, FilterActivation, SlaveStartFilterBank; } CAN_FilterTypeDef;
extern CAN_HandleTypeDef hcan1;
typedef struct { volatile uint32_t CTRL, CYCCNT; } test_dwt_t;
typedef struct { volatile uint32_t DEMCR; } test_debug_t;
extern test_dwt_t test_dwt;
extern test_debug_t test_debug;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define CAN1 (hcan1.Instance)
#define HAL_OK 0
#define HAL_ERROR 1
#define ENABLE 1
#define DISABLE 0
#define CAN_ID_STD 0
#define CAN_ID_EXT 4
#define CAN_RTR_DATA 0
#define CAN_RTR_REMOTE 2
#define CAN_FILTERMODE_IDMASK 0
#define CAN_FILTERSCALE_32BIT 1
#define CAN_RX_FIFO0 0
#define CAN1_RX0_IRQn 0
#define CAN1_SCE_IRQn 1
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_TSR_ABRQ0 (1U << 7)
#define CAN_TSR_ABRQ1 (1U << 15)
#define CAN_TSR_ABRQ2 (1U << 23)
void mock_write_reg(volatile uint32_t *reg, uint32_t value);
#define WRITE_REG(reg,value) mock_write_reg(&(reg),(value))
#define CAN_TSR_TXOK0 (1U << 1)
#define CAN_TSR_TXOK1 (1U << 9)
#define CAN_TSR_TXOK2 (1U << 17)
#define CAN_ESR_TEC_Msk 0xFF0000U
#define CAN_ESR_TEC_Pos 16U
#define CAN_ESR_REC_Msk 0xFF000000U
#define CAN_ESR_REC_Pos 24U
#define CAN_MCR_ABOM (1U << 6)
#define CAN_MCR_NART (1U << 4)
#define SET_BIT(reg,bits) ((reg) |= (bits))
#define CLEAR_BIT(reg,bits) ((reg) &= ~(bits))
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
uint32_t HAL_GetTick(void);
uint32_t HAL_CAN_GetState(const CAN_HandleTypeDef *h);
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h);
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *h, uint32_t mask);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,const CAN_TxHeaderTypeDef *header,uint8_t *data,uint32_t *mailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h,uint32_t mask);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h,const CAN_FilterTypeDef *f);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h,uint32_t mask);
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *h,uint32_t fifo);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h,uint32_t fifo,CAN_RxHeaderTypeDef *header,uint8_t *data);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *h);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_ClearPendingIRQ(int irq);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_SetPriority(int irq,uint32_t preempt,uint32_t sub);
#define HAL_CAN_ERROR_ACK (1U << 0)
#define HAL_CAN_ERROR_BD (1U << 1)
#define HAL_CAN_ERROR_BOF (1U << 2)
#define HAL_CAN_ERROR_BR (1U << 3)
#define HAL_CAN_ERROR_CRC (1U << 4)
#define HAL_CAN_ERROR_EPV (1U << 5)
#define HAL_CAN_ERROR_EWG (1U << 6)
#define HAL_CAN_ERROR_FOR (1U << 7)
#define HAL_CAN_ERROR_NOT_INITIALIZED (1U << 8)
#define HAL_CAN_ERROR_NOT_READY (1U << 9)
#define HAL_CAN_ERROR_NOT_STARTED (1U << 10)
#define HAL_CAN_ERROR_RX_FOV0 (1U << 11)
#define HAL_CAN_ERROR_STF (1U << 12)
#define HAL_CAN_ERROR_TIMEOUT (1U << 13)
#define HAL_CAN_ERROR_TX_TERR0 (1U << 14)
#define HAL_CAN_ERROR_TX_TERR1 (1U << 15)
#define HAL_CAN_ERROR_TX_TERR2 (1U << 16)
#define CAN_IT_BUSOFF (1U << 0)
#define CAN_IT_ERROR (1U << 1)
#define CAN_IT_ERROR_PASSIVE (1U << 2)
#define CAN_IT_ERROR_WARNING (1U << 3)
#define CAN_IT_LAST_ERROR_CODE (1U << 4)
#define CAN_IT_RX_FIFO0_MSG_PENDING (1U << 5)
#define CAN_IT_RX_FIFO0_OVERRUN (1U << 6)
#endif
