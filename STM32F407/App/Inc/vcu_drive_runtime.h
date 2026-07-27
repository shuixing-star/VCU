#ifndef VCU_DRIVE_RUNTIME_H
#define VCU_DRIVE_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

#include "vcu_crsf_uart.h"
#include "vcu_motor_can.h"
#include "vcu_rc_drive.h"
#include "vcu_speed_sync.h"

typedef enum {
    VCU_DRIVE_STATE_STOPPED = 0,
    VCU_DRIVE_STATE_DISARMED,
    VCU_DRIVE_STATE_ARMING_NEUTRAL,
    VCU_DRIVE_STATE_ARMED,
    VCU_DRIVE_STATE_FAULT
} vcu_drive_state_t;

typedef enum {
    VCU_DRIVE_REASON_INIT = 0,
    VCU_DRIVE_REASON_STOPPED,
    VCU_DRIVE_REASON_RESET,
    VCU_DRIVE_REASON_CRSF_LOST,
    VCU_DRIVE_REASON_MOTOR_TIMEOUT,
    VCU_DRIVE_REASON_MOTOR_FAULT,
    VCU_DRIVE_REASON_KILL,
    VCU_DRIVE_REASON_WAITING_NEUTRAL,
    VCU_DRIVE_REASON_NEUTRAL_DWELL,
    VCU_DRIVE_REASON_ARMED,
    VCU_DRIVE_REASON_RUNNING,
    VCU_DRIVE_REASON_CONTROL_FAILURE,
    VCU_DRIVE_REASON_CAN_TX_FAILURE,
    VCU_DRIVE_REASON_SCHEDULE_OVERRUN
} vcu_drive_reason_t;

typedef struct {
    bool initialized;
    bool requested_run;
    bool fault_latched;
    vcu_drive_state_t state;
    vcu_drive_reason_t reason;
    uint32_t period_ms;
    int16_t max_target_rpm;
    uint32_t acceleration_rpm_s;
    uint8_t can_ramp;
    vcu_rc_drive_preview_t preview;
    int16_t latest_left_base_target;
    int16_t latest_right_base_target;
    int16_t latest_left_target;
    int16_t latest_right_target;
    vcu_speed_sync_output_t sync_output;
    uint32_t loop_count;
    uint32_t drive_tx_count;
    uint32_t zero_tx_count;
    uint32_t reset_tx_count;
    uint32_t control_failure_count;
    uint32_t last_step_elapsed_ms;
    uint32_t schedule_overrun_count;
} vcu_drive_runtime_state_t;

bool vcu_drive_runtime_init(void);
uint32_t vcu_drive_runtime_control_period_ms(void);
void vcu_drive_runtime_request_run(uint32_t period_ms,
                                   uint32_t acceleration_rpm_s);
void vcu_drive_runtime_request_stop(void);
void vcu_drive_runtime_request_reset(void);
void vcu_drive_runtime_report_schedule_overrun(uint32_t now_ms);
void vcu_drive_runtime_step(uint32_t now_ms,
                            const vcu_crsf_uart_state_t *crsf,
                            const vcu_motor_can_state_t *motors);
bool vcu_drive_runtime_get_state(vcu_drive_runtime_state_t *state);

#endif
