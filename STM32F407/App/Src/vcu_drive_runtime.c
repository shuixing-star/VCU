#include "vcu_drive_runtime.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define VCU_DRIVE_DEFAULT_PERIOD_MS 5U
#define VCU_DRIVE_MIN_PERIOD_MS 5U
#define VCU_DRIVE_MAX_PERIOD_MS 100U
#define VCU_DRIVE_DEFAULT_MAX_TARGET_RPM 5000
#define VCU_DRIVE_STATUS_TIMEOUT_MS 500U
#define VCU_SYNC_STATUS_TIMEOUT_MS 150U
#define VCU_SYNC_PERIOD_MS 20U
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
    uint32_t pending_motor_fault_mask;
    vcu_reset_stage_t reset_stage;
    uint32_t last_step_ms;
    uint32_t sync_last_update_ms;
    vcu_speed_sync_config_t sync_config;
    vcu_speed_sync_state_t sync_state;
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
    if (per_mille == 0) {
        return 0;
    }
    return (int16_t)(((int32_t)per_mille * maximum) / 1000);
}


static bool throttle_neutral(const vcu_rc_drive_preview_t *preview)
{
    return abs(preview->throttle_per_mille) <=
                   VCU_DRIVE_NEUTRAL_PER_MILLE;
}

static bool controller_recent(const vcu_motor_can_state_t *motors,
                              unsigned int index,
                              uint32_t now_ms,
                              uint32_t timeout_ms)
{
    const vcu_motor_controller_state_t *controller;

    if (index >= VCU_MOTOR_COUNT) {
        return false;
    }

    controller = &motors->controller[index];
    return vcu_motor_can_controller_recent(controller, now_ms, timeout_ms);
}

static bool any_controller_recent(
        const vcu_motor_can_state_t *motors,
        uint32_t now_ms,
        uint32_t timeout_ms)
{
    return controller_recent(motors, VCU_MOTOR_LEFT,
                             now_ms, timeout_ms) ||
           controller_recent(motors, VCU_MOTOR_RIGHT,
                             now_ms, timeout_ms);
}

/* A missing controller cannot acknowledge that its previous fault cleared. */
static bool motor_fault_active(const vcu_motor_can_state_t *motors,
                               uint32_t now_ms)
{
    unsigned int index;

    for (index = 0U; index < VCU_MOTOR_COUNT; index++) {
        const vcu_motor_controller_state_t *controller =
                &motors->controller[index];
        const uint32_t mask = 1UL << index;

        if (controller->has_state_status &&
            mx_can_status_has_fault(&controller->status)) {
            s_ctx.pending_motor_fault_mask |= mask;
        } else if (controller_recent(motors, index, now_ms,
                                     VCU_DRIVE_STATUS_TIMEOUT_MS)) {
            /* Both feedback frames and the advancing heartbeat must be fresh. */
            s_ctx.pending_motor_fault_mask &= ~mask;
        }
    }
    return s_ctx.pending_motor_fault_mask != 0U;
}

static void reset_normal_control(void)
{
    vcu_speed_sync_reset(&s_ctx.sync_state);
    memset(&s_ctx.public_state.sync_output, 0,
           sizeof(s_ctx.public_state.sync_output));
    s_ctx.sync_last_update_ms = 0U;
    s_ctx.public_state.latest_left_base_target = 0;
    s_ctx.public_state.latest_right_base_target = 0;
    s_ctx.public_state.latest_left_target = 0;
    s_ctx.public_state.latest_right_target = 0;
}

static bool send_zero(void)
{
    reset_normal_control();
    if (!vcu_motor_can_send_targets(0, 0, false)) {
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
    /* Keep RUN intent. Retry with disabled zero commands and normal neutral/arming gates. */
    s_ctx.public_state.fault_latched = false;
    s_ctx.public_state.state = VCU_DRIVE_STATE_FAULT;
    s_ctx.public_state.reason = reason;
    reset_normal_control();
}

static void enter_fault(vcu_drive_reason_t reason)
{
    mark_fault(reason);
    if (!send_zero()) {
        mark_fault(VCU_DRIVE_REASON_CAN_TX_FAILURE);
    }
}

static void enter_safe_state(vcu_drive_state_t state,
                             vcu_drive_reason_t reason)
{
    if (!send_zero()) {
        mark_fault(VCU_DRIVE_REASON_CAN_TX_FAILURE);
        return;
    }

    s_ctx.public_state.state = state;
    s_ctx.public_state.reason = reason;
}

static vcu_drive_reason_t run_normal_control(int16_t left_base,
                               int16_t right_base,
                               const vcu_motor_can_state_t *motors,
                               uint32_t now_ms)
{
    const bool left_online = controller_recent(
            motors, VCU_MOTOR_LEFT, now_ms,
            VCU_DRIVE_STATUS_TIMEOUT_MS);
    const bool right_online = controller_recent(
            motors, VCU_MOTOR_RIGHT, now_ms,
            VCU_DRIVE_STATUS_TIMEOUT_MS);
    const bool left_valid = controller_recent(
            motors, VCU_MOTOR_LEFT, now_ms,
            VCU_SYNC_STATUS_TIMEOUT_MS);
    const bool right_valid = controller_recent(
            motors, VCU_MOTOR_RIGHT, now_ms,
            VCU_SYNC_STATUS_TIMEOUT_MS);
    vcu_speed_sync_output_t sync = s_ctx.public_state.sync_output;
    int16_t left_final;
    int16_t right_final;

    if (left_online && right_online) {
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
                motors->controller[VCU_MOTOR_LEFT].status.speed_rpm,
                motors->controller[VCU_MOTOR_RIGHT].status.speed_rpm,
                left_base,
                right_base
            };

            if (!vcu_speed_sync_step(&s_ctx.sync_state,
                                     &s_ctx.sync_config,
                                     &input, &sync)) {
                return VCU_DRIVE_REASON_CONTROL_FAILURE;
            }
            s_ctx.public_state.sync_output = sync;
            s_ctx.sync_last_update_ms = now_ms;
        }
    } else {
        vcu_speed_sync_reset(&s_ctx.sync_state);
        memset(&sync, 0, sizeof(sync));
        sync.diagnostic_flags = VCU_SYNC_DIAG_DISABLED;
        if (!left_online) {
            sync.diagnostic_flags |= VCU_SYNC_DIAG_LEFT_CAN_INVALID;
        }
        if (!right_online) {
            sync.diagnostic_flags |= VCU_SYNC_DIAG_RIGHT_CAN_INVALID;
        }
        sync.left_target_rpm = left_online ? left_base : 0;
        sync.right_target_rpm = right_online ? right_base : 0;
        s_ctx.public_state.sync_output = sync;
        s_ctx.sync_last_update_ms = now_ms;
    }

    /* Motor controllers own acceleration/deceleration; send the target directly. */
    left_final = left_online ?
            clamp_i32_to_target(
                    sync.left_target_rpm,
                    s_ctx.public_state.max_target_rpm) : 0;
    right_final = right_online ?
            clamp_i32_to_target(
                    sync.right_target_rpm,
                    s_ctx.public_state.max_target_rpm) : 0;

    if (!vcu_motor_can_send_targets(
                left_final, right_final, true)) {
        return VCU_DRIVE_REASON_CAN_TX_FAILURE;
    }

    s_ctx.public_state.latest_left_base_target = left_base;
    s_ctx.public_state.latest_right_base_target = right_base;
    s_ctx.public_state.latest_left_target = left_final;
    s_ctx.public_state.latest_right_target = right_final;
    s_ctx.public_state.drive_tx_count++;
    return VCU_DRIVE_REASON_RUNNING;
}

bool vcu_drive_runtime_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.public_state.state = VCU_DRIVE_STATE_STOPPED;
    s_ctx.public_state.reason = VCU_DRIVE_REASON_INIT;
    s_ctx.public_state.requested_run = true;
    s_ctx.public_state.period_ms = VCU_DRIVE_DEFAULT_PERIOD_MS;
    s_ctx.public_state.max_target_rpm =
            VCU_DRIVE_DEFAULT_MAX_TARGET_RPM;
    s_ctx.public_state.acceleration_rpm_s = 0U; /* VCU ramp disabled. */
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
    if (period_ms < VCU_DRIVE_MIN_PERIOD_MS) {
        period_ms = VCU_DRIVE_MIN_PERIOD_MS;
    }
    if (period_ms > VCU_DRIVE_MAX_PERIOD_MS) {
        period_ms = VCU_DRIVE_MAX_PERIOD_MS;
    }
    /* Keep the legacy RUN payload/API compatible; do not apply its ramp field. */
    (void)acceleration_rpm_s;
    s_ctx.public_state.period_ms = period_ms;
    s_ctx.public_state.acceleration_rpm_s = 0U;
    s_ctx.public_state.requested_run = true;
}

void vcu_drive_runtime_request_stop(void)
{
    s_ctx.public_state.requested_run = false;
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
    enter_fault(VCU_DRIVE_REASON_SCHEDULE_OVERRUN);
}

void vcu_drive_runtime_step(uint32_t now_ms,
                            const vcu_crsf_uart_state_t *crsf,
                            const vcu_motor_can_state_t *motors)
{
    vcu_rc_drive_config_t config;
    vcu_rc_drive_preview_t preview;
    bool reset_edge;
    bool status_recent;
    bool motor_fault;
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
        enter_fault(VCU_DRIVE_REASON_CONTROL_FAILURE);
        return;
    }
    s_ctx.public_state.preview = preview;
    s_ctx.public_state.input_rx_ms = crsf->last_valid_frame_ms;
    s_ctx.public_state.control_step_ms = now_ms;
    s_ctx.public_state.input_valid = crsf->valid_frame_count != 0U;

    status_recent = any_controller_recent(
            motors, now_ms, VCU_DRIVE_STATUS_TIMEOUT_MS);
    motor_fault = motor_fault_active(motors, now_ms);
    neutral = throttle_neutral(&preview);
    reset_edge = (preview.reset_requested &&
                  !s_ctx.reset_latched) ||
                 s_ctx.reset_pending;
    s_ctx.reset_latched = preview.reset_requested;
    s_ctx.reset_pending = false;

    if (reset_edge) {
        s_ctx.public_state.requested_run = false;
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
            s_ctx.public_state.requested_run = true;
            s_ctx.public_state.state = VCU_DRIVE_STATE_DISARMED;
            s_ctx.public_state.reason = VCU_DRIVE_REASON_RESET;
        }
    } else if (!s_ctx.public_state.requested_run) {
        enter_safe_state(VCU_DRIVE_STATE_STOPPED,
                         VCU_DRIVE_REASON_STOPPED);
    } else if (!preview.link_ok) {
        enter_safe_state(VCU_DRIVE_STATE_DISARMED,
                         VCU_DRIVE_REASON_CRSF_LOST);
    } else if (!status_recent) {
        enter_safe_state(VCU_DRIVE_STATE_DISARMED,
                         VCU_DRIVE_REASON_MOTOR_TIMEOUT);
    } else if (motor_fault) {
        /* Preserve RUN intent; healthy feedback must still pass neutral arming. */
        enter_safe_state(VCU_DRIVE_STATE_FAULT,
                         VCU_DRIVE_REASON_MOTOR_FAULT);
    } else if (!preview.arm_requested) {
        enter_safe_state(VCU_DRIVE_STATE_DISARMED,
                         VCU_DRIVE_REASON_KILL);
    } else if (preview.brake_requested) {
        /* Brake before mixing. Leaving BRAKING must pass neutral arming. */
        const uint16_t left_a = controller_recent(motors, VCU_MOTOR_LEFT,
                now_ms, VCU_DRIVE_STATUS_TIMEOUT_MS) ? VCU_MOTOR_BRAKE_REQUEST_A : 0U;
        const uint16_t right_a = controller_recent(motors, VCU_MOTOR_RIGHT,
                now_ms, VCU_DRIVE_STATUS_TIMEOUT_MS) ? VCU_MOTOR_BRAKE_REQUEST_A : 0U;
        reset_normal_control();
        if (!vcu_motor_can_send_brake(left_a, right_a)) {
            enter_fault(VCU_DRIVE_REASON_CAN_TX_FAILURE);
        } else {
            s_ctx.public_state.state = VCU_DRIVE_STATE_BRAKING;
            s_ctx.public_state.reason = VCU_DRIVE_REASON_BRAKING;
        }
    } else if (s_ctx.public_state.state !=
               VCU_DRIVE_STATE_ARMED) {
        if (!neutral) {
            enter_safe_state(VCU_DRIVE_STATE_ARMING_NEUTRAL,
                             VCU_DRIVE_REASON_WAITING_NEUTRAL);
        } else {
            /* A successful disabled-zero pair proves TX is available before arming. */
            enter_safe_state(VCU_DRIVE_STATE_ARMED, VCU_DRIVE_REASON_ARMED);
        }
    } else {
        const int16_t left_base =
                scale_target(preview.axis1_per_mille,
                             s_ctx.public_state.max_target_rpm);
        const int16_t right_base =
                scale_target(preview.axis2_per_mille,
                             s_ctx.public_state.max_target_rpm);

        const vcu_drive_reason_t result = run_normal_control(
                left_base, right_base, motors, now_ms);

        if (result == VCU_DRIVE_REASON_RUNNING) {
            s_ctx.public_state.state = VCU_DRIVE_STATE_ARMED;
            s_ctx.public_state.reason = VCU_DRIVE_REASON_RUNNING;
        } else {
            s_ctx.public_state.control_failure_count++;
            enter_fault(result);
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
