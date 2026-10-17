#include "vcu_can_port.h"

#include <string.h>

#include "can.h"

/* All TX APIs are owned by the foreground main loop; never call them from an ISR. */
#define VCU_CAN_TX_BUDGET_MS 2U
#define VCU_CAN_ABORT_BUDGET_MS 1U

#define VCU_CAN_RX_QUEUE_SIZE 16U
#define VCU_CAN_RX_QUEUE_MASK (VCU_CAN_RX_QUEUE_SIZE - 1U)
#define VCU_CAN_ERROR_QUEUE_SIZE 32U
#define VCU_CAN_ERROR_QUEUE_MASK (VCU_CAN_ERROR_QUEUE_SIZE - 1U)
#define VCU_CAN_NOTIFICATION_MASK \
    (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN | \
     CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | \
     CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR)

#define VCU_CAN_ERROR_LEC_MASK 0x07U
#define VCU_CAN_ERROR_WARNING (1U << 3)
#define VCU_CAN_ERROR_PASSIVE (1U << 4)
#define VCU_CAN_ERROR_BUS_OFF (1U << 5)
#define VCU_CAN_ERROR_TX (1U << 6)
#define VCU_CAN_ERROR_RX_OVERFLOW (1U << 7)

#if (VCU_CAN_RX_QUEUE_SIZE & VCU_CAN_RX_QUEUE_MASK) != 0
#error "VCU_CAN_RX_QUEUE_SIZE must be a power of two"
#endif
#if (VCU_CAN_ERROR_QUEUE_SIZE & VCU_CAN_ERROR_QUEUE_MASK) != 0
#error "VCU_CAN_ERROR_QUEUE_SIZE must be a power of two"
#endif

static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static vcu_can_port_frame_t s_rx_queue[VCU_CAN_RX_QUEUE_SIZE];
static volatile uint8_t s_error_head;
static volatile uint8_t s_error_tail;
static vcu_can_port_error_event_t
        s_error_queue[VCU_CAN_ERROR_QUEUE_SIZE];
static volatile uint8_t s_error_sequence;
static volatile uint32_t s_error_event_overflow_count;
static volatile bool s_error_capture_suppressed;
static volatile uint32_t s_rx_overflow_count;
static volatile uint32_t s_rx_error_count;
static volatile uint32_t s_tx_error_count;
static uint32_t s_background_mailbox;
static uint32_t s_background_started_ms;
static bool s_background_diagnostic;
/* 0 idle, 1 pending, 2 TXOK, 3 failed; only one diagnostic owner. */
static uint8_t s_async_diagnostic;
static uint32_t s_async_mailbox;
static uint32_t s_abort_pending;
static vcu_can_tx_observation_t s_tx_observation;
static uint32_t s_observed_mailbox[2];
#define TX_LOG_DEPTH 32U
static uint8_t s_tx_log[TX_LOG_DEPTH][8];
static unsigned s_tx_log_head, s_tx_log_count;
static uint32_t s_tx_log_lost;

void vcu_can_port_get_tx_observation(vcu_can_tx_observation_t *out) { *out = s_tx_observation; }
uint32_t vcu_can_port_now_ms(void) { return HAL_GetTick(); }
uint32_t vcu_can_port_esr(void) { return hcan1.Instance->ESR; }
uint32_t vcu_can_port_tx_log_dropped(void) { return s_tx_log_lost; }
bool vcu_can_port_take_tx_log(uint8_t out[8]) {
    if (!s_tx_log_count) return false;
    memcpy(out, s_tx_log[s_tx_log_head], 8);
    s_tx_log_head = (s_tx_log_head + 1U) % TX_LOG_DEPTH;
    s_tx_log_count--;
    return true;
}


typedef struct {
    uint32_t started_cycles, checked_cycles, max_gap_cycles;
    uint32_t first_id, second_id;
    uint8_t source, first_mailbox, second_mailbox;
} tx_probe_t;
static tx_probe_t s_background_probe;

static void probe_start(tx_probe_t *probe, uint8_t source,
                         uint32_t first_id, uint32_t second_id)
{
    memset(probe, 0, sizeof(*probe));
    probe->started_cycles = DWT->CYCCNT;
    probe->checked_cycles = probe->started_cycles;
    probe->source = source;
    probe->first_id = first_id;
    probe->second_id = second_id;
}

static void probe_check(tx_probe_t *probe)
{
    const uint32_t now = DWT->CYCCNT;
    const uint32_t gap = now - probe->checked_cycles;
    if (gap > probe->max_gap_cycles) probe->max_gap_cycles = gap;
    probe->checked_cycles = now;
}


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

static uint8_t lec_from_hal_error(uint32_t error)
{
    if ((error & HAL_CAN_ERROR_STF) != 0U) {
        return 1U;
    }
    if ((error & HAL_CAN_ERROR_FOR) != 0U) {
        return 2U;
    }
    if ((error & HAL_CAN_ERROR_ACK) != 0U) {
        return 3U;
    }
    if ((error & HAL_CAN_ERROR_BR) != 0U) {
        return 4U;
    }
    if ((error & HAL_CAN_ERROR_BD) != 0U) {
        return 5U;
    }
    if ((error & HAL_CAN_ERROR_CRC) != 0U) {
        return 6U;
    }
    return 0U;
}

static uint8_t flags_from_hal_error(uint32_t error)
{
    uint8_t flags = lec_from_hal_error(error) & VCU_CAN_ERROR_LEC_MASK;

    if ((error & HAL_CAN_ERROR_EWG) != 0U) {
        flags |= VCU_CAN_ERROR_WARNING;
    }
    if ((error & HAL_CAN_ERROR_EPV) != 0U) {
        flags |= VCU_CAN_ERROR_PASSIVE;
    }
    if ((error & HAL_CAN_ERROR_BOF) != 0U) {
        flags |= VCU_CAN_ERROR_BUS_OFF;
    }
    if ((error & (HAL_CAN_ERROR_TX_TERR0 |
                  HAL_CAN_ERROR_TX_TERR1 |
                  HAL_CAN_ERROR_TX_TERR2)) != 0U) {
        flags |= VCU_CAN_ERROR_TX;
    }
    if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U) {
        flags |= VCU_CAN_ERROR_RX_OVERFLOW;
    }
    return flags;
}

static void capture_error_event_with_probe(vcu_can_port_error_type_t type,
                                uint32_t hal_error, tx_probe_t *probe,
                                uint8_t trigger)
{
    vcu_can_port_error_event_t event;
    uint32_t esr;
    uint32_t primask;
    uint8_t next;

    if (s_error_capture_suppressed) {
        return;
    }

    memset(&event, 0, sizeof(event));
    /* Snapshot before abort; retain the existing timeout decision so TXOK
     * can reveal completion between the pending check and deadline test. */
    if (probe != NULL) {
        const uint32_t tsr = hcan1.Instance->TSR;
        const uint32_t cycles = DWT->CYCCNT;
        const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
        probe_check(probe);
        event.has_tx_probe = true;
        event.tx_probe[0] = tsr;
        event.tx_probe[2] = probe->first_id;
        event.tx_probe[3] = probe->second_id;
        event.tx_probe[4] = (cycles - probe->started_cycles) / cycles_per_us;
        event.tx_probe[5] = probe->max_gap_cycles / cycles_per_us;
        /* Preserve wire format: task-switch bits 7:2 stay zero. */
        event.tx_probe[6] = (uint32_t)probe->source |
                ((uint32_t)probe->first_mailbox << 8U) |
                ((uint32_t)probe->second_mailbox << 16U) |
                ((uint32_t)trigger << 24U);
    }
    esr = hcan1.Instance->ESR;
    event.tx_probe[1] = esr;
    event.sequence = 0U;
    event.type = (uint8_t)type;
    event.flags = flags_from_hal_error(hal_error);
    if (type != VCU_CAN_PORT_ERROR_HAL_IRQ) {
        event.flags |= VCU_CAN_ERROR_TX;
    }
    event.hal_state = (uint8_t)HAL_CAN_GetState(&hcan1);
    event.tx_error_count =
            (uint8_t)((esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
    event.rx_error_count =
            (uint8_t)((esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
    event.timestamp_10ms = (uint16_t)(HAL_GetTick() / 10U);

    primask = __get_PRIMASK();
    __disable_irq();
    event.sequence = ++s_error_sequence;
    next = (uint8_t)((s_error_head + 1U) & VCU_CAN_ERROR_QUEUE_MASK);
    if (next == s_error_tail) {
        s_error_tail = (uint8_t)(
                (s_error_tail + 1U) & VCU_CAN_ERROR_QUEUE_MASK);
        s_error_event_overflow_count++;
    }
    s_error_queue[s_error_head] = event;
    __DMB();
    s_error_head = next;
    __set_PRIMASK(primask);
}

static void capture_error_event(vcu_can_port_error_type_t type,
                                uint32_t hal_error)
{
    capture_error_event_with_probe(type, hal_error, NULL, 0U);
}

static void capture_tx_event(vcu_can_port_error_type_t type,
                             tx_probe_t *probe, uint8_t trigger)
{
    capture_error_event_with_probe(type, HAL_CAN_GetError(&hcan1),
                                   probe, trigger);
}

static void make_accept_all_filter(CAN_FilterTypeDef *filter)
{
    memset(filter, 0, sizeof(*filter));
    filter->FilterBank = 0U;
    filter->FilterMode = CAN_FILTERMODE_IDMASK;
    filter->FilterScale = CAN_FILTERSCALE_32BIT;
    filter->FilterFIFOAssignment = CAN_RX_FIFO0;
    filter->FilterActivation = ENABLE;
    filter->SlaveStartFilterBank = 14U;
}

static HAL_StatusTypeDef configure_and_start_can(void)
{
    CAN_FilterTypeDef filter;

    make_accept_all_filter(&filter);
    SET_BIT(hcan1.Instance->MCR, CAN_MCR_ABOM);
    CLEAR_BIT(hcan1.Instance->MCR, CAN_MCR_NART);

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(
                &hcan1, VCU_CAN_NOTIFICATION_MASK) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static uint32_t pending_mailboxes(uint32_t mask)
{
    uint32_t pending = 0U;
    uint32_t bit;
    for (bit = CAN_TX_MAILBOX0; bit <= CAN_TX_MAILBOX2; bit <<= 1U) {
        if ((mask & bit) != 0U &&
            HAL_CAN_IsTxMessagePending(&hcan1, bit) != 0U) {
            pending |= bit;
        }
    }
    return pending;
}

static bool transmission_succeeded(uint32_t mask)
{
    uint32_t expected = 0U;
    if ((mask & CAN_TX_MAILBOX0) != 0U) { expected |= CAN_TSR_TXOK0; }
    if ((mask & CAN_TX_MAILBOX1) != 0U) { expected |= CAN_TSR_TXOK1; }
    if ((mask & CAN_TX_MAILBOX2) != 0U) { expected |= CAN_TSR_TXOK2; }
    /* TX interrupts are disabled: no handler consumes TXOK before this check.
     * Hardware clears TXOK when a new request enters the mailbox (RM0090). */
    return (hcan1.Instance->TSR & expected) == expected;
}

static void issue_abort(uint32_t mask)
{
    uint32_t request = 0U;
    if ((mask & CAN_TX_MAILBOX0) != 0U) { request |= CAN_TSR_ABRQ0; }
    if ((mask & CAN_TX_MAILBOX1) != 0U) { request |= CAN_TSR_ABRQ1; }
    if ((mask & CAN_TX_MAILBOX2) != 0U) { request |= CAN_TSR_ABRQ2; }
    /* TSR completion bits are write-one-to-clear. HAL's read/modify/write
     * abort can clear another mailbox's TXOK; write only the ABRQ bits. */
    WRITE_REG(hcan1.Instance->TSR, request);
}

static void request_abort(uint32_t mask)
{
    s_abort_pending |= pending_mailboxes(mask);
    if (s_abort_pending != 0U) {
        issue_abort(s_abort_pending);
    }
}

void vcu_can_port_poll(void)
{
    /* Abort is asynchronous. Do not reuse any mailbox until it is released. */
    s_abort_pending = pending_mailboxes(s_abort_pending);
    if (s_abort_pending != 0U) {
        issue_abort(s_abort_pending);
    }
    if (s_background_mailbox == 0U) {
        return;
    }
    probe_check(&s_background_probe);
    if (pending_mailboxes(s_background_mailbox) == 0U) {
        if (s_async_diagnostic == 1U && s_async_mailbox == s_background_mailbox)
            s_async_diagnostic = transmission_succeeded(s_background_mailbox) ? 2U : 3U;
        s_background_mailbox = 0U;
    } else if ((uint32_t)(HAL_GetTick() - s_background_started_ms) >=
               VCU_CAN_TX_BUDGET_MS) {
        if (!s_background_diagnostic) {
            capture_tx_event(VCU_CAN_PORT_ERROR_TX_TIMEOUT,
                              &s_background_probe, 1U);
        }
        s_tx_error_count++;
        if (s_async_diagnostic == 1U && s_async_mailbox == s_background_mailbox) s_async_diagnostic = 3U;
        request_abort(s_background_mailbox);
        s_background_mailbox = 0U;
    }
}

static void observe_txok(void) {
    unsigned i;
    for (i = 0; i < 2; ++i) {
        if (s_observed_mailbox[i] && transmission_succeeded(s_observed_mailbox[i]))
            s_tx_observation.txok_mask |= (uint8_t)(1U << i);
    }
}

static void cancel_transmission(uint32_t mask)
{
    const uint32_t started_ms = HAL_GetTick();
    observe_txok();
    request_abort(mask);
    while (pending_mailboxes(mask) != 0U &&
           (uint32_t)(HAL_GetTick() - started_ms) < VCU_CAN_ABORT_BUDGET_MS) {
        /* IRQs remain enabled so the HAL time base and CAN RX can progress. */
    }
    s_abort_pending = pending_mailboxes(s_abort_pending);
}

static bool send_background(const vcu_can_port_frame_t *frame, bool diagnostic)
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;
    HAL_StatusTypeDef status;

    vcu_can_port_poll();
    if (!frame_valid(frame)) {
        s_tx_error_count++;
        return false;
    }
    /* One best-effort frame at most; preserve two mailboxes for motor control.
     * A skipped status frame is congestion, not a new physical CAN error. */
    if (s_abort_pending != 0U || s_background_mailbox != 0U ||
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U) {
        return false;
    }
    make_tx_header(frame, &header);
    probe_start(&s_background_probe, 2U, frame->id, 0U);
    s_error_capture_suppressed = diagnostic;
    status = HAL_CAN_AddTxMessage(&hcan1, &header,
                                  (uint8_t *)frame->data, &mailbox);
    s_error_capture_suppressed = false;
    if (status != HAL_OK) {
        if (!diagnostic) {
            capture_tx_event(VCU_CAN_PORT_ERROR_TX_ADD_FAILED,
                              &s_background_probe, 4U);
        }
        s_tx_error_count++;
        return false;
    }
    s_background_mailbox = mailbox;
    s_background_probe.first_mailbox = (uint8_t)mailbox;
    s_background_started_ms = HAL_GetTick();
    s_background_diagnostic = diagnostic;
    return true;
}

static bool send_pair_once(const vcu_can_port_frame_t *first,
                           const vcu_can_port_frame_t *second)
{
    CAN_TxHeaderTypeDef first_header;
    CAN_TxHeaderTypeDef second_header;
    uint32_t first_mailbox;
    uint32_t second_mailbox;
    uint32_t pair;
    uint8_t trigger = 1U;
    tx_probe_t probe;
    const uint32_t started_ms = HAL_GetTick();
    probe_start(&probe, 1U, first->id, second->id);

    for (;;) {
        probe_check(&probe);
        vcu_can_port_poll();
        if (s_abort_pending == 0U &&
            HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >= 2U) {
            break;
        }
        if ((uint32_t)(HAL_GetTick() - started_ms) >= VCU_CAN_TX_BUDGET_MS) {
            capture_tx_event(VCU_CAN_PORT_ERROR_MAILBOX_TIMEOUT, &probe, 3U);
            return false;
        }
    }

    make_tx_header(first, &first_header);
    make_tx_header(second, &second_header);
    if (HAL_CAN_AddTxMessage(&hcan1, &first_header,
                             (uint8_t *)first->data,
                             &first_mailbox) != HAL_OK) {
        capture_tx_event(VCU_CAN_PORT_ERROR_TX_ADD_FAILED, &probe, 4U);
        return false;
    }
    s_observed_mailbox[0] = first_mailbox;
    s_tx_observation.queued_mask |= 1U;
    probe.first_mailbox = (uint8_t)first_mailbox;
    /* A best-effort frame can finish between poll and HAL mailbox selection.
     * If HAL reused that mailbox, its old expiry must not cancel this command. */
    if (s_background_mailbox == first_mailbox) {
        if (s_async_diagnostic == 1U && s_async_mailbox == first_mailbox) s_async_diagnostic = 3U;
        s_background_mailbox = 0U;
    }
    if (HAL_CAN_AddTxMessage(&hcan1, &second_header,
                             (uint8_t *)second->data,
                             &second_mailbox) != HAL_OK) {
        capture_tx_event(VCU_CAN_PORT_ERROR_TX_ADD_FAILED, &probe, 4U);
        cancel_transmission(first_mailbox);
        return false;
    }
    s_observed_mailbox[1] = second_mailbox;
    s_tx_observation.queued_mask |= 2U;
    probe.second_mailbox = (uint8_t)second_mailbox;
    if (s_background_mailbox == second_mailbox) {
        if (s_async_diagnostic == 1U && s_async_mailbox == second_mailbox) s_async_diagnostic = 3U;
        s_background_mailbox = 0U;
    }
    pair = first_mailbox | second_mailbox;
    for (;;) {
        probe_check(&probe);
        vcu_can_port_poll();
        if (pending_mailboxes(pair) == 0U) {
            if (transmission_succeeded(pair)) {
                return true;
            }
            trigger = 2U;
            /* An empty/aborted mailbox alone is not a successful send. */
            break;
        }
        if ((uint32_t)(HAL_GetTick() - started_ms) >= VCU_CAN_TX_BUDGET_MS) {
            break;
        }
    }
    capture_tx_event(VCU_CAN_PORT_ERROR_TX_TIMEOUT, &probe, trigger);
    cancel_transmission(pair);
    return false;
}

bool vcu_can_port_init(void)
{
    /* Enable cycle profiling without changing HAL or RTOS time bases. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
    HAL_NVIC_ClearPendingIRQ(CAN1_SCE_IRQn);

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_error_head = 0U;
    s_error_tail = 0U;
    s_error_sequence = 0U;
    s_error_event_overflow_count = 0U;
    s_error_capture_suppressed = false;
    s_rx_overflow_count = 0U;
    s_rx_error_count = 0U;
    s_tx_error_count = 0U;
    s_background_mailbox = 0U;
    s_background_started_ms = 0U;
    s_background_diagnostic = false;
    s_async_diagnostic = 0U; s_async_mailbox = 0U;
    s_abort_pending = 0U;
    memset(&s_tx_observation, 0, sizeof(s_tx_observation));
    s_tx_log_head = s_tx_log_count = 0; s_tx_log_lost = 0;
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6U, 0U);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 6U, 0U);

    if (configure_and_start_can() != HAL_OK) {
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
    return send_background(frame, false);
}

bool vcu_can_port_send_diagnostic_async(const vcu_can_port_frame_t *frame)
{
    vcu_can_port_poll();
    if (s_async_diagnostic == 2U) { s_async_diagnostic = 0U; return true; }
    if (s_async_diagnostic == 3U) { s_async_diagnostic = 0U; return false; }
    if (s_async_diagnostic == 1U) return false;
    if (send_background(frame, true)) {
        s_async_mailbox = s_background_mailbox;
        s_async_diagnostic = 1U;
    }
    return false;
}

bool vcu_can_port_send_diagnostic(const vcu_can_port_frame_t *frame)
{
    uint32_t mailbox;
    uint32_t started_ms;
    bool sent;

    if (!send_background(frame, true)) {
        return false;
    }
    mailbox = s_background_mailbox;
    started_ms = s_background_started_ms;
    /* The caller removes an error record only on true. Mailbox admission is
     * insufficient: a later expiry/abort would otherwise lose that record. */
    while (pending_mailboxes(mailbox) != 0U) {
        if ((uint32_t)(HAL_GetTick() - started_ms) >= VCU_CAN_TX_BUDGET_MS) {
            cancel_transmission(mailbox);
            break;
        }
    }
    sent = pending_mailboxes(mailbox) == 0U &&
           transmission_succeeded(mailbox);
    s_background_mailbox = 0U;
    if (!sent) {
        /* Keep the original event at the caller; do not create a log about
         * failure to send a log. Outstanding aborts still block mailbox reuse. */
        s_tx_error_count++;
    }
    return sent;
}

bool vcu_can_port_send_log(const vcu_can_port_frame_t *frame) {
    return send_background(frame, true);
}
bool vcu_can_port_send_pair(const vcu_can_port_frame_t *first,
                            const vcu_can_port_frame_t *second)
{
    bool ok;
    uint8_t *log;
    uint32_t elapsed;
    s_tx_observation.sequence++;
    s_tx_observation.started_ms = HAL_GetTick();
    s_tx_observation.queued_mask = s_tx_observation.txok_mask = 0;
    s_observed_mailbox[0] = s_observed_mailbox[1] = 0;
    if (!frame_valid(first) || !frame_valid(second)) {
        s_tx_error_count++;
        return false;
    }
    s_tx_observation.id[0] = first->id; s_tx_observation.id[1] = second->id;
    memcpy(s_tx_observation.data[0], first->data, 8);
    memcpy(s_tx_observation.data[1], second->data, 8);
    ok = send_pair_once(first, second);
    observe_txok();
    s_tx_observation.success = ok ? 1U : 0U;
    s_tx_observation.completed_ms = HAL_GetTick();
    elapsed = s_tx_observation.completed_ms - s_tx_observation.started_ms;
    if (s_tx_log_count == TX_LOG_DEPTH) {
        s_tx_log_head = (s_tx_log_head + 1U) % TX_LOG_DEPTH;
        s_tx_log_count--; s_tx_log_lost++;
    }
    log = s_tx_log[(s_tx_log_head + s_tx_log_count++) % TX_LOG_DEPTH];
    log[0] = (uint8_t)s_tx_observation.sequence;
    log[1] = (uint8_t)(s_tx_observation.sequence >> 8);
    log[2] = (uint8_t)s_tx_observation.started_ms;
    log[3] = (uint8_t)(s_tx_observation.started_ms >> 8);
    log[4] = s_tx_observation.queued_mask | (s_tx_observation.txok_mask << 2) | (ok ? 16U : 0U);
    log[5] = first->data[7]; log[6] = second->data[7];
    log[7] = elapsed > 255U ? 255U : (uint8_t)elapsed;
    if (!ok) s_tx_error_count++;
    return ok;
}

uint32_t vcu_can_port_rx_overflow_count(void)
{
    return s_rx_overflow_count;
}

bool vcu_can_port_take_error_event(vcu_can_port_error_event_t *event)
{
    uint32_t primask;

    if (event == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_error_tail == s_error_head) {
        __set_PRIMASK(primask);
        return false;
    }
    *event = s_error_queue[s_error_tail];
    s_error_tail = (uint8_t)(
            (s_error_tail + 1U) & VCU_CAN_ERROR_QUEUE_MASK);
    __set_PRIMASK(primask);
    return true;
}

uint32_t vcu_can_port_error_event_overflow_count(void)
{
    return s_error_event_overflow_count;
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
    const uint32_t tx_error_mask =
            HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV |
            HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_TX_TERR0 |
            HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2 |
            HAL_CAN_ERROR_TIMEOUT | HAL_CAN_ERROR_NOT_INITIALIZED |
            HAL_CAN_ERROR_NOT_READY | HAL_CAN_ERROR_NOT_STARTED;

    if (hcan == NULL || hcan->Instance != CAN1) {
        return;
    }

    error = HAL_CAN_GetError(hcan);
    capture_error_event(VCU_CAN_PORT_ERROR_HAL_IRQ, error);
    if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U) {
        s_rx_overflow_count++;
    }
    if ((error & tx_error_mask) != 0U) {
        s_tx_error_count++;
    }
    s_rx_error_count++;
    (void)HAL_CAN_ResetError(hcan);
}

#ifdef VCU_FLIGHT_LOG_ENABLED
uint32_t vcu_can_port_boot_nonce(void) {
    uint32_t attempts, value=0;
    /* F407 hardware RNG; bounded init only, before control starts. Zero means
     * unavailable and must not be presented as a unique session identifier. */
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);
    SET_BIT(RNG->CR, RNG_CR_RNGEN);
    for(attempts=0;attempts<100000U;attempts++) {
        if(RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) break;
        if(RNG->SR & RNG_SR_DRDY) { value=RNG->DR; break; }
    }
    CLEAR_BIT(RNG->CR,RNG_CR_RNGEN);
    return value;
}
#endif
