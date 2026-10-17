#ifndef VCU_CAN_PORT_H
#define VCU_CAN_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;
    uint32_t received_ms;
    bool extended_id;
    bool remote_frame;
    uint8_t data_length;
    uint8_t data[8];
} vcu_can_port_frame_t;

typedef enum {
    VCU_CAN_PORT_ERROR_HAL_IRQ = 1,
    VCU_CAN_PORT_ERROR_MAILBOX_TIMEOUT,
    VCU_CAN_PORT_ERROR_TX_TIMEOUT,
    VCU_CAN_PORT_ERROR_TX_ADD_FAILED,
    VCU_CAN_PORT_ERROR_SOFT_RECOVERY_FAILED,
    VCU_CAN_PORT_ERROR_HARD_RECOVERY_FAILED
} vcu_can_port_error_type_t;

typedef struct {
    uint8_t sequence;
    uint8_t type;
    uint8_t flags;
    uint8_t hal_state;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint16_t timestamp_10ms;
    bool has_tx_probe;
    uint32_t tx_probe[7];
} vcu_can_port_error_event_t;

/* Foreground observation only: admission and TXOK are different facts.
 * TXOK means the CAN peripheral completed arbitration/ACK, not motor execution. */
typedef struct {
    uint32_t sequence, started_ms, completed_ms;
    uint32_t id[2];
    uint8_t data[2][8];
    uint8_t queued_mask, txok_mask, success;
} vcu_can_tx_observation_t;
void vcu_can_port_get_tx_observation(vcu_can_tx_observation_t *out);
bool vcu_can_port_take_tx_log(uint8_t out[8]);
uint32_t vcu_can_port_tx_log_dropped(void);
uint32_t vcu_can_port_esr(void);
uint32_t vcu_can_port_now_ms(void);
/* Nonblocking log admission, with no recursive error record on log failure. */
bool vcu_can_port_send_log(const vcu_can_port_frame_t *frame);
bool vcu_can_port_init(void);
/* Call every foreground-loop iteration to expire best-effort TX requests. */
void vcu_can_port_poll(void);
bool vcu_can_port_read(vcu_can_port_frame_t *frame);
/* Single-frame sends are best effort and reserve two mailboxes for control. */
bool vcu_can_port_send(const vcu_can_port_frame_t *frame);
/* Task-only diagnostic submission: true requires hardware TXOK.
 * False retains the caller's event for a later retry. Same 2+1 ms budget. */
bool vcu_can_port_send_diagnostic(const vcu_can_port_frame_t *frame);
/* Pollable TXOK confirmation for the logging target; never waits. */
bool vcu_can_port_send_diagnostic_async(const vcu_can_port_frame_t *frame);
/* Task-only: waits up to 2 ms for the pair to complete, then requests abort
 * with up to 1 ms cleanup. True means both transmissions completed with TXOK. */
bool vcu_can_port_send_pair(const vcu_can_port_frame_t *first,
                            const vcu_can_port_frame_t *second);
bool vcu_can_port_take_error_event(vcu_can_port_error_event_t *event);
uint32_t vcu_can_port_error_event_overflow_count(void);
uint32_t vcu_can_port_rx_overflow_count(void);
uint32_t vcu_can_port_rx_error_count(void);
uint32_t vcu_can_port_tx_error_count(void);

#endif
