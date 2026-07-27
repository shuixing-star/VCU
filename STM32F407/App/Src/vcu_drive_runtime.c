#include "vcu_drive_runtime.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "vcu_target_slew.h"

#define VCU_DRIVE_DEFAULT_PERIOD_MS 5U
#define VCU_DRIVE_MIN_PERIOD_MS 5U
#define VCU_DRIVE_MAX_PERIOD_MS 100U
#define VCU_DRIVE_DEFAULT_MAX_TARGET_RPM 4800
#define VCU_DRIVE_DEFAULT_ACCEL_RPM_S 2550U
#define VCU_DRIVE_MIN_ACCEL_RPM_S 100U
#define VCU_DRIVE_MAX_ACCEL_RPM_S 2550U
#define VCU_DRIVE_ACCEL_RAMP_UNIT_RPM_S 10U
#define VCU_DRIVE_STATUS_TIMEOUT_MS 500U
#define VCU_SYNC_STATUS_TIMEOUT_MS 50U
#define VCU_SYNC_PERIOD_MS 20U
#define VCU_DRIVE_NEUTRAL_DWELL_MS 500U
#define VCU_DRIVE_NEUTRAL_PER_MILLE 50

typedef enum {
    VCU_RESET_STAGE_NONE = 0,
    VCU_RESET_STAGE_SAFE_ZERO,
    VCU_RESET_STAGE_COMMAND
} vcu_reset_stage_t;

typedef struct {
    vcu_drive_runtime_state_t public_state;
    bool reset_latched;
    bool reset_pending;
    bool has_last_step;
    vcu_reset_stage_t reset_stage;
    vcu_drive_reason_t latched_fault_reason;
    uint32_t neutral_since_ms;
    uint32_t last_step_ms;
    uint32_t sync_last_update_ms;
    vcu_speed_sync_config_t sync_config;
    vcu_speed_sync_state_t sync_state;
    vcu_target_slew_state_t left_slew;
    vcu_target_slew_state_t right_slew;
} vcu_drive_runtime_context_t;

static vcu_drive_runtime_context_t s_ctx;

static int16_t clamp_i32_to_target(int32_t target, int16_t maximum)
{
    if (target > maximum) {
        return maximum;
    }
    if (target < -maximum) {
        return (int16_t)-maximum;
    }
    return (int16_t)target;
}

static int16_t scale_target(int16_t per_mille, int16_t maximum)
{
    int32_t target;

    if (per_mille == 0) {
        return 0;
    }

    target = ((int32_t)per_mille * maximum) / 1000;
    if (target > -VCU_VEHICLE_RPM_DEADZONE &&
        target < VCU_VEHICLE_RPM_DEADZONE) {
        return 0;
    }
    return (int16_t)target;
}

static uint8_t acceleration_to_can_ramp(uint32_t acceleration_rpm_s)
{
    uint32_t ramp =
            (acceleration_rpm_s +
             VCU_DRIVE_ACCEL_RAMP_UNIT_RPM_S / 2U) /
            VCU_DRIVE_ACCEL_RAMP_UNIT_RPM_S;

    if (ramp == 0U) {
        ramp = 1U;
    }
    if (ramp > UINT8_MAX) {
        ramp = UINT8_MAX;
    }
    return (uint8_t)ramp;
}

static bool neutral_controls(const vcu_rc_drive_preview_t *preview)
{
    return abs(preview->throttle_per_mille) <=
                   VCU_DRIVE_NEUTRAL_PER_MILLE &&
           abs(preview->steering_per_mille) <=
                   VCU_DRIVE_NEUTRAL_PER_MILLE;
}

static bool controllers_recent(const vcu_motor_can_state_t *motors,
                               uint32_t now_ms,
                               uint32_t timeout_ms)
{
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        const vcu_motor_controller_state_t *controller =
                &motors->controller[index];

        if (!controller->has_status ||
            (uint32_t)(now_ms - controller->last_status_ms) >
                    timeout_ms) {
            return false;
        }
    }
    return true;
}

static bool controllers_fault_free(
        const vcu_motor_can_state_t *motors)
{
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        if (motors->controller[index].status.fault_code != 0U) {
            return false;
        }
    }
    return true;
}

static void reset_normal_control(void)
{
    vcu_speed_sync_reset(&s_ctx.sync_state);
    memset(&s_ctx.public_state.sync_output, 0,
           sizeof(s_ctx.public_state.sync_output));
    s_ctx.sync_last_update_ms = 0U;
    vcu_target_slew_reset(&s_ctx.left_slew, 0);
    vcu_target_slew_reset(&s_ctx.right_slew, 0);
    s_ctx.public_state.latest_left_base_target = 0;
    s_ctx.public_state.latest_right_base_target = 0;
    s_ctx.public_state.latest_left_target = 0;
    s_ctx.public_state.latest_right_target = 0;
}

static bool send_zero(void)
{
    reset_normal_control();
    if (!vcu_motor_can_send_targets(0, 0, 0U, false)) {
        return false;
    }
    s_ctx.public_state.zero_tx_count++;
    return true;
}

static bool send_reset(void)
{
    if (!vcu_motor_can_send_reset()) {
        return false;
    }
    s_ctx.public_state.reset_tx_count++;
    return true;
}

static void mark_fault(vcu_drive_reason_t reason)
{
    s_ctx.public_state.requested_run = false;
    s_ctx.public_state.fault_latched = true;
    s_ctx.public_state.state = VCU_DRIVE_STATE_FAULT;
    s_ctx.public_state.reason = reason;
    s_ctx.latched_fault_reason = reason;
    s_ctx.neutral_since_ms = 0U;
    reset_normal_control();
}

static void latch_fault(vcu_drive_reason_t reason)
{
    mark_fault(reason);
    if (!send_zero()) {
        mark_fault(VCU_DRIVE_REASON_CAN_TX_FAILURE);
    }
}

static void enter_safe_state(vcu_drive_state_t state,
                             vcu_drive_reason_t reason,
                             bool reset_neutral_dwell)
{
    if (reset_neutral_dwell) {
        s_ctx.neutral_since_ms = 0U;
    }

    if (!send_zero()) {
        mark_fault(VCU_DRIVE_REASON_CAN_TX_FAILURE);
        return;
    }

    s_ctx.public_state.state = state;
    s_ctx.public_state.reason = reason;
}

static bool run_normal_control(int16_t left_base,
                               int16_t right_base,
                               const vcu_motor_can_state_t *motors,
                               uint32_t now_ms,
                               uint32_t elapsed_ms)
{
    const bool left_valid =
            motors->controller[VCU_MOTOR_LEFT].has_status &&
            (uint32_t)(now_ms -
                motors->controller[VCU_MOTOR_LEFT].last_status_ms) <=
                    VCU_SYNC_STATUS_TIMEOUT_MS;
    const bool right_valid =
            motors->controller[VCU_MOTOR_RIGHT].has_status &&
            (uint32_t)(now_ms -
                motors->controller[VCU_MOTOR_RIGHT].last_status_ms) <=
                    VCU_SYNC_STATUS_TIMEOUT_MS;
    vcu_speed_sync_output_t sync = s_ctx.public_state.sync_output;
    int32_t left_limited;
    int32_t right_limited;
    int16_t left_final;
    int16_t right_final;

    if (s_ctx.sync_last_update_ms == 0U ||
        (uint32_t)(now_ms - s_ctx.sync_last_update_ms) >=
                VCU_SYNC_PERIOD_MS) {
        const uint32_t sync_elapsed_ms =
                s_ctx.sync_last_update_ms == 0U ?
                VCU_SYNC_PERIOD_MS :
                (uint32_t)(now_ms - s_ctx.sync_last_update_ms);
        const vcu_speed_sync_input_t input = {
            true,
            left_valid,
            right_valid,
            motors->controller[VCU_MOTOR_LEFT].status.fault_code != 0U,
            motors->controller[VCU_MOTOR_RIGHT].status.fault_code != 0U,
            (float)sync_elapsed_ms / 1000.0f,
            motors->controller[VCU_MOTOR_LEFT].status.axis2_speed_rpm,
            motors->controller[VCU_MOTOR_RIGHT].status.axis2_speed_rpm,
            left_base,
            right_base
        };

        if (!vcu_speed_sync_step(&s_ctx.sync_state,
                                 &s_ctx.sync_config,
                                 &input, &sync)) {
            return false;
        }
        s_ctx.public_state.sync_output = sync;
        s_ctx.sync_last_update_ms = now_ms;
    }

    if (!vcu_target_slew_step(
                &s_ctx.left_slew, sync.left_target_rpm,
                (int32_t)s_ctx.public_state.acceleration_rpm_s,
                (float)elapsed_ms / 1000.0f,
                &left_limited) ||
        !vcu_target_slew_step(
                &s_ctx.right_slew, sync.right_target_rpm,
                (int32_t)s_ctx.public_state.acceleration_rpm_s,
                (float)elapsed_ms / 1000.0f,
                &right_limited)) {
        return false;
    }

    left_final = clamp_i32_to_target(
            left_limited, s_ctx.public_state.max_target_rpm);
    right_final = clamp_i32_to_target(
            right_limited, s_ctx.public_state.max_target_rpm);

    if (!vcu_motor_can_send_targets(
                left_final, right_final,
                s_ctx.public_state.can_ramp, true)) {
        return false;
    }

    s_ctx.public_state.latest_left_base_target = left_base;
    s_ctx.public_state.latest_right_base_target = right_base;
    s_ctx.public_state.latest_left_target = left_final;
    s_ctx.public_state.latest_right_target = right_final;
    s_ctx.public_state.drive_tx_count++;
    return true;
}

bool vcu_drive_runtime_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.public_state.state = VCU_DRIVE_STATE_STOPPED;
    s_ctx.public_state.reason = VCU_DRIVE_REASON_INIT;
    s_ctx.public_state.period_ms = VCU_DRIVE_DEFAULT_PERIOD_MS;
    s_ctx.public_state.max_target_rpm =
            VCU_DRIVE_DEFAULT_MAX_TARGET_RPM;
    s_ctx.public_state.acceleration_rpm_s =
            VCU_DRIVE_DEFAULT_ACCEL_RPM_S;
    s_ctx.public_state.can_ramp =
            acceleration_to_can_ramp(
                    VCU_DRIVE_DEFAULT_ACCEL_RPM_S);
    s_ctx.latched_fault_reason = VCU_DRIVE_REASON_INIT;
    vcu_speed_sync_default_config(&s_ctx.sync_config);
    reset_normal_control();
    s_ctx.public_state.initialized = true;
    return true;
}

uint32_t vcu_drive_runtime_control_period_ms(void)
{
    return s_ctx.public_state.initialized ?
           s_ctx.public_state.period_ms :
           VCU_DRIVE_DEFAULT_PERIOD_MS;
}

void vcu_drive_runtime_request_run(uint32_t period_ms,
                                   uint32_t acceleration_rpm_s)
{
    if (s_ctx.public_state.fault_latched) {
        return;
    }

    if (period_ms < VCU_DRIVE_MIN_PERIOD_MS) {
        period_ms = VCU_DRIVE_MIN_PERIOD_MS;
    }
    if (period_ms > VCU_DRIVE_MAX_PERIOD_MS) {
        period_ms = VCU_DRIVE_MAX_PERIOD_MS;
    }
    if (acceleration_rpm_s < VCU_DRIVE_MIN_ACCEL_RPM_S) {
        acceleration_rpm_s = VCU_DRIVE_MIN_ACCEL_RPM_S;
    }
    if (acceleration_rpm_s > VCU_DRIVE_MAX_ACCEL_RPM_S) {
        acceleration_rpm_s = VCU_DRIVE_MAX_ACCEL_RPM_S;
    }

    s_ctx.public_state.period_ms = period_ms;
    s_ctx.public_state.acceleration_rpm_s = acceleration_rpm_s;
    s_ctx.public_state.can_ramp =
            acceleration_to_can_ramp(acceleration_rpm_s);
    s_ctx.public_state.requested_run = true;
    s_ctx.neutral_since_ms = 0U;
}

void vcu_drive_runtime_request_stop(void)
{
    s_ctx.public_state.requested_run = false;
    s_ctx.neutral_since_ms = 0U;
}

void vcu_drive_runtime_request_reset(void)
{
    s_ctx.reset_pending = true;
}

void vcu_drive_runtime_report_schedule_overrun(uint32_t now_ms)
{
    uint32_t elapsed_ms = s_ctx.public_state.period_ms;

    if (!s_ctx.public_state.initialized) {
        return;
    }

    if (s_ctx.has_last_step) {
        elapsed_ms = (uint32_t)(now_ms - s_ctx.last_step_ms);
    }
    s_ctx.last_step_ms = now_ms;
    s_ctx.has_last_step = true;
    s_ctx.public_state.last_step_elapsed_ms = elapsed_ms;
    s_ctx.public_state.schedule_overrun_count++;
    latch_fault(VCU_DRIVE_REASON_SCHEDULE_OVERRUN);
}

void vcu_drive_runtime_step(uint32_t now_ms,
                            const vcu_crsf_uart_state_t *crsf,
                            const vcu_motor_can_state_t *motors)
{
    vcu_rc_drive_config_t config;
    vcu_rc_drive_preview_t preview;
    bool reset_edge;
    bool status_recent;
    bool fault_free;
    bool neutral;
    uint32_t elapsed_ms;

    if (!s_ctx.public_state.initialized ||
        crsf == NULL || motors == NULL) {
        return;
    }

    elapsed_ms = s_ctx.has_last_step ?
            (uint32_t)(now_ms - s_ctx.last_step_ms) :
            s_ctx.public_state.period_ms;
    s_ctx.last_step_ms = now_ms;
    s_ctx.has_last_step = true;
    s_ctx.public_state.last_step_elapsed_ms = elapsed_ms;

    s_ctx.public_state.loop_count++;
    vcu_rc_drive_get_default_config(&config);
    if (!vcu_rc_drive_preview_from_channels(
                crsf->channels, crsf->link_ok,
                &config, &preview)) {
        s_ctx.public_state.control_failure_count++;
        latch_fault(VCU_DRIVE_REASON_CONTROL_FAILURE);
        return;
    }
    s_ctx.public_state.preview = preview;

    status_recent = controllers_recent(
            motors, now_ms, VCU_DRIVE_STATUS_TIMEOUT_MS);
    fault_free = controllers_fault_free(motors);
    neutral = neutral_controls(&preview);
    reset_edge = (preview.reset_requested &&
                  !s_ctx.reset_latched) ||
                 s_ctx.reset_pending;
    s_ctx.reset_latched = preview.reset_requested;
    s_ctx.reset_pending = false;

    if (reset_edge) {
        s_ctx.public_state.requested_run = false;
        s_ctx.neutral_since_ms = 0U;
        s_ctx.reset_stage = VCU_RESET_STAGE_SAFE_ZERO;
    }

    if (s_ctx.reset_stage == VCU_RESET_STAGE_SAFE_ZERO) {
        if (send_zero()) {
            s_ctx.reset_stage = VCU_RESET_STAGE_COMMAND;
            s_ctx.public_state.state = VCU_DRIVE_STATE_DISARMED;
            s_ctx.public_state.reason = VCU_DRIVE_REASON_RESET;
        } else {
            mark_fault(VCU_DRIVE_REASON_CAN_TX_FAILURE);
        }
    } else if (s_ctx.reset_stage == VCU_RESET_STAGE_COMMAND) {
        if (!send_reset()) {
            mark_fault(VCU_DRIVE_REASON_CAN_TX_FAILURE);
        } else {
            s_ctx.reset_stage = VCU_RESET_STAGE_NONE;
            s_ctx.public_state.fault_latched = false;
            s_ctx.latched_fault_reason = VCU_DRIVE_REASON_RESET;
            s_ctx.public_state.state = VCU_DRIVE_STATE_DISARMED;
            s_ctx.public_state.reason = VCU_DRIVE_REASON_RESET;
        }
    } else if (s_ctx.public_state.fault_latched) {
        enter_safe_state(VCU_DRIVE_STATE_FAULT,
                         s_ctx.latched_fault_reason, true);
    } else if (!s_ctx.public_state.requested_run) {
        enter_safe_state(VCU_DRIVE_STATE_STOPPED,
                         VCU_DRIVE_REASON_STOPPED, true);
    } else if (!preview.link_ok) {
        enter_safe_state(VCU_DRIVE_STATE_DISARMED,
                         VCU_DRIVE_REASON_CRSF_LOST, true);
    } else if (!status_recent) {
        enter_safe_state(VCU_DRIVE_STATE_DISARMED,
                         VCU_DRIVE_REASON_MOTOR_TIMEOUT, true);
    } else if (!fault_free) {
        latch_fault(VCU_DRIVE_REASON_MOTOR_FAULT);
    } else if (!preview.arm_requested) {
        enter_safe_state(VCU_DRIVE_STATE_DISARMED,
                         VCU_DRIVE_REASON_KILL, true);
    } else if (s_ctx.public_state.state !=
               VCU_DRIVE_STATE_ARMED) {
        if (!neutral) {
            enter_safe_state(VCU_DRIVE_STATE_ARMING_NEUTRAL,
                             VCU_DRIVE_REASON_WAITING_NEUTRAL,
                             true);
        } else {
            if (s_ctx.neutral_since_ms == 0U) {
                s_ctx.neutral_since_ms = now_ms;
            }
            enter_safe_state(
                    (uint32_t)(now_ms - s_ctx.neutral_since_ms) >=
                            VCU_DRIVE_NEUTRAL_DWELL_MS ?
                    VCU_DRIVE_STATE_ARMED :
                    VCU_DRIVE_STATE_ARMING_NEUTRAL,
                    (uint32_t)(now_ms - s_ctx.neutral_since_ms) >=
                            VCU_DRIVE_NEUTRAL_DWELL_MS ?
                    VCU_DRIVE_REASON_ARMED :
                    VCU_DRIVE_REASON_NEUTRAL_DWELL,
                    false);
        }
    } else {
        const int16_t left_base =
                scale_target(preview.axis1_per_mille,
                             s_ctx.public_state.max_target_rpm);
        const int16_t right_base =
                scale_target(preview.axis2_per_mille,
                             s_ctx.public_state.max_target_rpm);

        if (run_normal_control(left_base, right_base,
                               motors, now_ms, elapsed_ms)) {
            s_ctx.public_state.state = VCU_DRIVE_STATE_ARMED;
            s_ctx.public_state.reason = VCU_DRIVE_REASON_RUNNING;
        } else {
            s_ctx.public_state.control_failure_count++;
            latch_fault(VCU_DRIVE_REASON_CONTROL_FAILURE);
        }
    }
}

bool vcu_drive_runtime_get_state(vcu_drive_runtime_state_t *state)
{
    if (state == NULL) {
        return false;
    }
    *state = s_ctx.public_state;
    return true;
}
