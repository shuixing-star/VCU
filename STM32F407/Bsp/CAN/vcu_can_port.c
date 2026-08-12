#include "vcu_can_port.h"

#include <string.h>

#include "can.h"

#define VCU_CAN_RX_QUEUE_SIZE 16U
#define VCU_CAN_RX_QUEUE_MASK (VCU_CAN_RX_QUEUE_SIZE - 1U)

#if (VCU_CAN_RX_QUEUE_SIZE & VCU_CAN_RX_QUEUE_MASK) != 0
#error "VCU_CAN_RX_QUEUE_SIZE must be a power of two"
#endif

static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static vcu_can_port_frame_t s_rx_queue[VCU_CAN_RX_QUEUE_SIZE];
static volatile uint32_t s_rx_overflow_count;
static volatile uint32_t s_rx_error_count;
static volatile uint32_t s_tx_error_count;

static bool frame_valid(const vcu_can_port_frame_t *frame)
{
    return frame != NULL &&
           frame->data_length <= 8U &&
           ((!frame->extended_id && frame->id <= 0x7FFU) ||
            (frame->extended_id && frame->id <= 0x1FFFFFFFUL));
}

static void make_tx_header(const vcu_can_port_frame_t *frame,
                           CAN_TxHeaderTypeDef *header)
{
    memset(header, 0, sizeof(*header));
    header->StdId = frame->extended_id ? 0U : frame->id;
    header->ExtId = frame->extended_id ? frame->id : 0U;
    header->IDE = frame->extended_id ? CAN_ID_EXT : CAN_ID_STD;
    header->RTR =
            frame->remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    header->DLC = frame->data_length;
    header->TransmitGlobalTime = DISABLE;
}

bool vcu_can_port_init(void)
{
    CAN_FilterTypeDef filter = {0};

    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
    HAL_NVIC_ClearPendingIRQ(CAN1_SCE_IRQn);

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow_count = 0U;
    s_rx_error_count = 0U;
    s_tx_error_count = 0U;

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    SET_BIT(hcan1.Instance->MCR, CAN_MCR_ABOM);
    CLEAR_BIT(hcan1.Instance->MCR, CAN_MCR_NART);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6U, 0U);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 6U, 0U);

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(
                &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING |
                        CAN_IT_RX_FIFO0_OVERRUN |
                        CAN_IT_ERROR_WARNING |
                        CAN_IT_ERROR_PASSIVE |
                        CAN_IT_BUSOFF |
                        CAN_IT_LAST_ERROR_CODE |
                        CAN_IT_ERROR) != HAL_OK) {
        return false;
    }

    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    return true;
}

bool vcu_can_port_read(vcu_can_port_frame_t *frame)
{
    if (frame == NULL || s_rx_tail == s_rx_head) {
        return false;
    }

    __DMB();
    *frame = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint8_t)((s_rx_tail + 1U) & VCU_CAN_RX_QUEUE_MASK);
    return true;
}

bool vcu_can_port_send(const vcu_can_port_frame_t *frame)
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;

    if (!frame_valid(frame) ||
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        s_tx_error_count++;
        return false;
    }

    make_tx_header(frame, &header);
    if (HAL_CAN_AddTxMessage(&hcan1, &header,
                             (uint8_t *)frame->data, &mailbox) != HAL_OK) {
        s_tx_error_count++;
        return false;
    }
    return true;
}

bool vcu_can_port_send_pair(const vcu_can_port_frame_t *first,
                            const vcu_can_port_frame_t *second)
{
    CAN_TxHeaderTypeDef first_header;
    CAN_TxHeaderTypeDef second_header;
    uint32_t first_mailbox;
    uint32_t second_mailbox;

    if (!frame_valid(first) || !frame_valid(second) ||
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 2U) {
        s_tx_error_count++;
        return false;
    }

    make_tx_header(first, &first_header);
    make_tx_header(second, &second_header);

    if (HAL_CAN_AddTxMessage(
                &hcan1, &first_header,
                (uint8_t *)first->data,
                &first_mailbox) != HAL_OK) {
        s_tx_error_count++;
        return false;
    }

    if (HAL_CAN_AddTxMessage(
                &hcan1, &second_header,
                (uint8_t *)second->data,
                &second_mailbox) != HAL_OK) {
        (void)HAL_CAN_AbortTxRequest(&hcan1, first_mailbox);
        s_tx_error_count++;
        return false;
    }

    return true;
}

uint32_t vcu_can_port_rx_overflow_count(void)
{
    return s_rx_overflow_count;
}

uint32_t vcu_can_port_rx_error_count(void)
{
    return s_rx_error_count;
}

uint32_t vcu_can_port_tx_error_count(void)
{
    return s_tx_error_count;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hcan == NULL || hcan->Instance != CAN1) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        const uint8_t next =
                (uint8_t)((s_rx_head + 1U) & VCU_CAN_RX_QUEUE_MASK);

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                &header, data) != HAL_OK) {
            s_rx_error_count++;
            return;
        }

        if (header.DLC > 8U) {
            s_rx_error_count++;
            continue;
        }

        if (next == s_rx_tail) {
            s_rx_overflow_count++;
            continue;
        }

        s_rx_queue[s_rx_head].id =
                header.IDE == CAN_ID_EXT ? header.ExtId : header.StdId;
        s_rx_queue[s_rx_head].received_ms = HAL_GetTick();
        s_rx_queue[s_rx_head].extended_id = header.IDE == CAN_ID_EXT;
        s_rx_queue[s_rx_head].remote_frame = header.RTR == CAN_RTR_REMOTE;
        s_rx_queue[s_rx_head].data_length = (uint8_t)header.DLC;
        memset(s_rx_queue[s_rx_head].data, 0,
               sizeof(s_rx_queue[s_rx_head].data));
        memcpy(s_rx_queue[s_rx_head].data, data, header.DLC);
        __DMB();
        s_rx_head = next;
    }
}

void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error;

    if (hcan == NULL || hcan->Instance != CAN1) {
        return;
    }

    error = HAL_CAN_GetError(hcan);
    if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U) {
        s_rx_overflow_count++;
    }
    s_rx_error_count++;
    (void)HAL_CAN_ResetError(hcan);
}
