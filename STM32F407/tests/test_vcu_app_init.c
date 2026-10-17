#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "vcu_app.h"
#include "vcu_control_telemetry.h"
#include "vcu_esp_link.h"
static bool config_ok = true;
static bool tx_ok;
static unsigned zeros;
void vcu_control_telemetry_init(void) {}
void vcu_esp_link_init(void) {}
bool vcu_crsf_uart_init(void) { return config_ok; }
bool vcu_motor_can_init(void) { return config_ok; }
bool vcu_drive_runtime_init(void) { return config_ok; }
bool vcu_motor_can_send_targets(int16_t left, int16_t right, bool enable) {
    assert(left == 0 && right == 0 && !enable);
    zeros++;
    return tx_ok;
}
static unsigned polls, reports, steps, overruns, rx_remaining, rx_processed;
void vcu_can_port_poll(void) { polls++; }
void vcu_crsf_uart_step(uint32_t now) { (void)now; }
bool vcu_can_port_read(vcu_can_port_frame_t *f) { if (!rx_remaining) return false; rx_remaining--; memset(f,0,sizeof(*f)); return true; }
bool vcu_esp_link_process_frame(const vcu_can_port_frame_t *f) { (void)f; return false; }
bool vcu_motor_can_process_frame(const vcu_can_port_frame_t *f, uint32_t now) { (void)f; (void)now; rx_processed++; return false; }
bool vcu_esp_link_take_command(vcu_gateway_command_t *c) { (void)c; return false; }
void vcu_drive_runtime_request_run(uint32_t p, uint32_t a) { (void)p; (void)a; }
void vcu_drive_runtime_request_stop(void) {}
void vcu_drive_runtime_request_reset(void) {}
bool vcu_crsf_uart_get_state(uint32_t now, vcu_crsf_uart_state_t *s) { (void)now; memset(s,0,sizeof(*s)); return true; }
bool vcu_motor_can_get_state(vcu_motor_can_state_t *s) { memset(s,0,sizeof(*s)); return true; }
uint32_t vcu_drive_runtime_control_period_ms(void) { return 5; }
void vcu_drive_runtime_report_schedule_overrun(uint32_t now) { (void)now; overruns++; }
void vcu_drive_runtime_step(uint32_t now, const vcu_crsf_uart_state_t *c, const vcu_motor_can_state_t *m) { (void)now; (void)c; (void)m; steps++; }
bool vcu_drive_runtime_get_state(vcu_drive_runtime_state_t *s) { memset(s,0,sizeof(*s)); return true; }
void vcu_control_telemetry_step(uint32_t now, const vcu_crsf_uart_state_t *c, const vcu_drive_runtime_state_t *d, const vcu_motor_can_state_t *m) { (void)now; (void)c; (void)d; (void)m; reports++; }
int main(void) {
    /* No controller ACK during VCU startup must not kill foreground processing. */
    tx_ok = false; assert(vcu_app_init()); assert(zeros == 1);
    vcu_app_step(1); vcu_app_step(6);
    assert(polls == 2 && reports == 2 && steps == 1);
    tx_ok = true; assert(vcu_app_init()); assert(zeros == 2);
    /* Tick wrap and exact 5 ms cadence, with no catch-up burst. */
    polls = reports = steps = overruns = 0;
    vcu_app_step(UINT32_MAX - 2U);
    vcu_app_step(UINT32_MAX);
    vcu_app_step(1U);
    assert(steps == 0 && overruns == 0);
    vcu_app_step(2U);
    assert(steps == 1);
    vcu_app_step(2U);
    vcu_app_step(6U);
    assert(steps == 1);
    vcu_app_step(7U);
    assert(steps == 2);
    vcu_app_step(32U); /* 25 ms is allowed: period + margin. */
    assert(steps == 3 && overruns == 0);
    vcu_app_step(58U); /* 26 ms must take the fault-report path. */
    assert(steps == 3 && overruns == 1);
    /* RX flood is bounded and cannot prevent control from running. */
    rx_remaining = 40;
    vcu_app_step(63U);
    assert(rx_processed == 16 && rx_remaining == 24 && steps == 4);
    vcu_app_step(64U);
    assert(rx_processed == 32 && rx_remaining == 8 && steps == 4);
    vcu_app_step(65U);
    assert(rx_processed == 40 && rx_remaining == 0);
    config_ok = false; assert(!vcu_app_init()); assert(zeros == 2);
    unsigned previous_polls = polls;
    vcu_app_step(1000U);
    assert(polls == previous_polls);
    puts("PASS cadence, tick wrap, overrun, bounded RX, failed-init gating");
    puts("PASS startup TX failure keeps RX/diagnostics alive; config failure remains fatal");
    return 0;
}
