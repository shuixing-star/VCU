#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vcu_control_telemetry.h"
#include "vcu_esp_link.h"

static unsigned free_mailboxes, attempts, normal_sent, error_sent, errors_taken;
static unsigned queued_errors;
static uint32_t ids[128];
static vcu_gateway_can_error_t delivered[32];
static vcu_crsf_uart_state_t crsf;
static vcu_drive_runtime_state_t drive;
static vcu_motor_can_state_t motors;
static vcu_gateway_heartbeat_t last_heartbeat;

bool vcu_motor_can_controller_recent(const vcu_motor_controller_state_t *c,
        uint32_t now, uint32_t timeout) { (void)c; (void)now; (void)timeout; return true; }
uint32_t vcu_can_port_rx_overflow_count(void) { return 0; }
uint32_t vcu_can_port_rx_error_count(void) { return 1; }
uint32_t vcu_can_port_tx_error_count(void) { return 1; }
bool vcu_esp_link_duplicate_command_seen(void) { return false; }
bool vcu_esp_link_invalid_command_seen(void) { return false; }
bool vcu_esp_link_command_seen(void) { return false; }
bool vcu_can_port_take_error_event(vcu_can_port_error_event_t *event) {
    if (!queued_errors) return false;
    memset(event, 0, sizeof(*event));
    event->sequence = (uint8_t)++errors_taken;
    event->type = VCU_CAN_PORT_ERROR_MAILBOX_TIMEOUT;
    event->tx_error_count = 8;
    queued_errors--;
    return true;
}
bool vcu_esp_link_send(uint32_t id, const uint8_t data[8]) {
    if (id == VCU_GATEWAY_ID_HEARTBEAT)
        vcu_gateway_unpack_heartbeat(data, &last_heartbeat);
    attempts++;
    if (!free_mailboxes) return false;
    free_mailboxes--;
    ids[normal_sent++] = id;
    return true;
}
bool vcu_esp_link_send_diagnostic(uint32_t id, const uint8_t data[8]) {
    attempts++;
    assert(id == VCU_GATEWAY_ID_CAN_ERROR);
    if (!free_mailboxes) return false;
    free_mailboxes--;
    vcu_gateway_unpack_can_error(data, &delivered[error_sent++]);
    return true;
}
static void reset(unsigned events) {
    normal_sent = error_sent = errors_taken = attempts = 0;
    queued_errors = events;
    memset(&crsf, 0, sizeof(crsf));
    memset(&drive, 0, sizeof(drive));
    memset(&motors, 0, sizeof(motors));
    vcu_control_telemetry_init();
}
static void step(uint32_t now, unsigned free_count) {
    free_mailboxes = free_count;
    attempts = 0;
    vcu_control_telemetry_step(now, &crsf, &drive, &motors);
}
static void current_can_fault_flag_recovers(void) {
    reset(0);
    crsf.link_ok = true;
    motors.tx_failure_count = 25U; /* Historical counter must not hold fault bit. */
    drive.state = VCU_DRIVE_STATE_FAULT;
    drive.reason = VCU_DRIVE_REASON_CAN_TX_FAILURE;
    step(5, 1);
    assert((last_heartbeat.fault_flags & (1U << 5)) != 0U);
    drive.state = VCU_DRIVE_STATE_ARMING_NEUTRAL;
    drive.reason = VCU_DRIVE_REASON_WAITING_NEUTRAL;
    for (unsigned t = 10; t <= 40; t += 5) step(t, 1);
    assert(last_heartbeat.fault_flags == 0U);
    assert(motors.tx_failure_count == 25U);
}
int main(void) {
    unsigned max_attempts = 0;
    current_can_fault_flag_recovers();
    reset(0);drive.state=VCU_DRIVE_STATE_BRAKING;drive.reason=VCU_DRIVE_REASON_BRAKING;
    step(5,1);assert(last_heartbeat.state==VCU_DRIVE_STATE_BRAKING);
    assert(last_heartbeat.reason==VCU_DRIVE_REASON_BRAKING);
    assert(last_heartbeat.flags & (1U << 4));
    reset(1);
    /* Two motor commands are still pending: one hardware mailbox remains. */
    for (unsigned t = 5; t <= 70; t += 5) {
        step(t, 1);
        if (attempts > max_attempts) max_attempts = attempts;
    }
    printf("one free mailbox: normal=%u errors=%u max_attempts_per_slot=%u\n",
           normal_sent, error_sent, max_attempts);
    fflush(stdout);
    assert(error_sent == 1);
    assert(max_attempts <= 1);
    assert(delivered[0].type == VCU_GATEWAY_CAN_ERROR_MAILBOX_TIMEOUT);
    assert(delivered[0].tx_error_count == 8);

    reset(2);
    /* Saturation must retain the oldest event, then retry when room returns. */
    for (unsigned t = 5; t <= 35; t += 5) step(t, 0);
    assert(error_sent == 0 && errors_taken == 1);
    for (unsigned t = 40; t <= 105; t += 5) {
        step(t, 1);
        assert(attempts <= 1);
    }
    assert(error_sent == 2 && errors_taken == 2);
    assert(delivered[0].sequence == 1 && delivered[1].sequence == 2);

    reset(20);
    for (unsigned t = 5; t <= 350; t += 5) {
        step(t, 1);
        assert(attempts <= 1);
    }
    assert(normal_sent == 60 && error_sent == 10);
    for (unsigned i = 0; i < normal_sent; i++)
        assert(ids[i] == VCU_GATEWAY_ID_HEARTBEAT + i % 6);
    reset(0);
    step(4, 1); assert(attempts == 0);
    step(5, 1); assert(normal_sent == 1);
    step(5, 1); assert(attempts == 0);
    puts("PASS: mailbox budget, error delivery/retry/order, telemetry fairness, cadence");
    return 0;
}
