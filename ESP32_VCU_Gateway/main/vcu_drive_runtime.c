#include "vcu_drive_runtime.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "terminal.h"
#include "vcu_crsf_uart.h"
#include "vcu_mx_can_test.h"
#include "vcu_rc_drive.h"
#include "vcu_speed_sync.h"
#include "vcu_target_slew.h"

#define VCU_DRIVE_TASK_STACK_SIZE 4096
#define VCU_DRIVE_TASK_PRIORITY 8
#define VCU_DRIVE_DEFAULT_PERIOD_MS 5U
#define VCU_DRIVE_MIN_PERIOD_MS 5U
#define VCU_DRIVE_MAX_PERIOD_MS 100U
#define VCU_DRIVE_DEFAULT_MAX_TARGET 4800
#define VCU_DRIVE_DEFAULT_ACCEL_RPM_S 2550U
#define VCU_DRIVE_MIN_ACCEL_RPM_S 100U
#define VCU_DRIVE_MAX_ACCEL_RPM_S 2550U
#define VCU_DRIVE_ACCEL_RAMP_UNIT_RPM_S 10U
#define VCU_DRIVE_STATUS_TIMEOUT_MS 500U
#define VCU_SYNC_STATUS_TIMEOUT_MS 50U
#define VCU_SYNC_PERIOD_MS 20U
#define VCU_DRIVE_NEUTRAL_DWELL_MS 500U
#define VCU_DRIVE_NEUTRAL_PER_MILLE 50

typedef struct {
    vcu_drive_runtime_state_t public;
    bool reset_latched;
    uint32_t neutral_since_ms;
    uint32_t sync_last_update_ms;
    vcu_speed_sync_config_t sync_config;
    vcu_speed_sync_state_t sync_state;
    vcu_speed_sync_output_t sync_output;
    vcu_target_slew_state_t left_slew;
    vcu_target_slew_state_t right_slew;
    portMUX_TYPE lock;
} vcu_drive_runtime_context_t;

static vcu_drive_runtime_context_t m_ctx = {
    .public = {
        .state = VCU_DRIVE_STATE_STOPPED,
        .period_ms = VCU_DRIVE_DEFAULT_PERIOD_MS,
        .max_target = VCU_DRIVE_DEFAULT_MAX_TARGET,
        .accel_rpm_s = VCU_DRIVE_DEFAULT_ACCEL_RPM_S,
        .can_ramp = VCU_DRIVE_DEFAULT_ACCEL_RPM_S / VCU_DRIVE_ACCEL_RAMP_UNIT_RPM_S,
        .sync_enabled = true,
        .sync_target_slew_rpm_s = VCU_DRIVE_DEFAULT_ACCEL_RPM_S,
    },
    .lock = portMUX_INITIALIZER_UNLOCKED,
};

static uint32_t now_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static int16_t clamp_i16_target(int32_t target, int16_t maximum) {
    if (target > maximum) return maximum;
    if (target < -maximum) return (int16_t)-maximum;
    return (int16_t)target;
}

const char *vcu_drive_runtime_state_name(vcu_drive_state_t state) {
    switch (state) {
    case VCU_DRIVE_STATE_STOPPED: return "STOPPED";
    case VCU_DRIVE_STATE_DISARMED: return "DISARMED";
    case VCU_DRIVE_STATE_ARMING_NEUTRAL: return "ARMING_NEUTRAL";
    case VCU_DRIVE_STATE_ARMED: return "ARMED";
    case VCU_DRIVE_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

static bool neutral_controls(const vcu_rc_drive_preview_t *preview) {
    return abs(preview->throttle_per_mille) <= VCU_DRIVE_NEUTRAL_PER_MILLE &&
            abs(preview->steering_per_mille) <= VCU_DRIVE_NEUTRAL_PER_MILLE;
}

static bool controllers_recent(const vcu_mx_can_test_state_t *state, uint32_t now,
        uint32_t timeout_ms) {
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        const vcu_mx_can_controller_state_t *controller = &state->controller[i];
        if (!controller->configured || controller->command_id == 0U ||
                !controller->has_status || now - controller->last_status_ms > timeout_ms) {
            return false;
        }
    }
    return true;
}

static bool controllers_fault_free(const vcu_mx_can_test_state_t *state) {
    for (unsigned int i = 0U; i < VCU_MX_CAN_CONTROLLER_COUNT; i++) {
        if (state->controller[i].status.fault_code != 0U) return false;
    }
    return true;
}

static int16_t scale_target(int16_t per_mille, int16_t max_target) {
    if (per_mille == 0) return 0;
    const int32_t target = ((int32_t)per_mille * max_target) / 1000;
    if (target > -VCU_VEHICLE_RPM_DEADZONE && target < VCU_VEHICLE_RPM_DEADZONE) return 0;
    return (int16_t)target;
}

static uint8_t accel_to_can_ramp(uint32_t accel_rpm_s) {
    uint32_t ramp = (accel_rpm_s + VCU_DRIVE_ACCEL_RAMP_UNIT_RPM_S / 2U) /
            VCU_DRIVE_ACCEL_RAMP_UNIT_RPM_S;
    if (ramp == 0U) ramp = 1U;
    if (ramp > UINT8_MAX) ramp = UINT8_MAX;
    return (uint8_t)ramp;
}

static void set_runtime_state(vcu_drive_state_t state, const char *reason) {
    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.public.state = state;
    if (reason != NULL) {
        strncpy(m_ctx.public.reason, reason, sizeof(m_ctx.public.reason) - 1U);
        m_ctx.public.reason[sizeof(m_ctx.public.reason) - 1U] = '\0';
    }
    portEXIT_CRITICAL(&m_ctx.lock);
}

static void reset_normal_control(void) {
    portENTER_CRITICAL(&m_ctx.lock);
    vcu_speed_sync_reset(&m_ctx.sync_state);
    memset(&m_ctx.sync_output, 0, sizeof(m_ctx.sync_output));
    m_ctx.sync_last_update_ms = 0U;
    vcu_target_slew_reset(&m_ctx.left_slew, 0);
    vcu_target_slew_reset(&m_ctx.right_slew, 0);
    m_ctx.public.latest_left_target = 0;
    m_ctx.public.latest_right_target = 0;
    m_ctx.public.latest_left_base_target = 0;
    m_ctx.public.latest_right_base_target = 0;
    m_ctx.public.left_pre_slew_target = 0;
    m_ctx.public.right_pre_slew_target = 0;
    portEXIT_CRITICAL(&m_ctx.lock);
}

static void send_zero(void) {
    reset_normal_control();
    if (vcu_mx_can_test_send_axis2_targets(0, 0, 0, false)) {
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public.zero_tx_count++;
        portEXIT_CRITICAL(&m_ctx.lock);
    }
}

static void send_reset(void) {
    if (vcu_mx_can_test_send_reset()) {
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public.requested_run = false;
        m_ctx.neutral_since_ms = 0U;
        m_ctx.public.reset_tx_count++;
        portEXIT_CRITICAL(&m_ctx.lock);
    }
}

static void publish_sync_state(const vcu_speed_sync_output_t *sync,
        bool left_can_valid, bool right_can_valid) {
    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.public.sync_diagnostic_flags = sync->diagnostic_flags;
    m_ctx.public.sync_active = sync->sync_active;
    m_ctx.public.weak_field = sync->weak_field;
    m_ctx.public.left_can_valid = left_can_valid;
    m_ctx.public.right_can_valid = right_can_valid;
    m_ctx.public.average_rpm = sync->average_rpm;
    m_ctx.public.longitudinal_target_rpm = sync->longitudinal_target_rpm;
    m_ctx.public.derated_longitudinal_target_rpm = sync->derated_longitudinal_target_rpm;
    m_ctx.public.theoretical_difference_rpm = sync->theoretical_difference_rpm;
    m_ctx.public.actual_difference_rpm = sync->actual_difference_rpm;
    m_ctx.public.sync_raw_error_rpm = sync->raw_error_rpm;
    m_ctx.public.sync_effective_error_rpm = sync->effective_error_rpm;
    m_ctx.public.sync_raw_correction_rpm = sync->raw_correction_rpm;
    m_ctx.public.sync_active_limit_rpm = sync->active_limit_rpm;
    m_ctx.public.sync_limited_correction_rpm = sync->limited_correction_rpm;
    m_ctx.public.sync_large_error_elapsed_ms = sync->large_error_elapsed_ms;
    m_ctx.public.sync_accumulated_derate_rpm = sync->accumulated_derate_rpm;
    portEXIT_CRITICAL(&m_ctx.lock);
}

static bool run_normal_control(int16_t left_base, int16_t right_base,
        const vcu_mx_can_test_state_t *mx, uint32_t now,
        uint32_t loop_period_ms, int16_t max_target, uint8_t can_ramp) {
    bool enabled;
    int32_t slew_rate;
    vcu_speed_sync_config_t config;
    uint32_t last_update;
    vcu_speed_sync_output_t cached;
    portENTER_CRITICAL(&m_ctx.lock);
    enabled = m_ctx.public.sync_enabled;
    slew_rate = m_ctx.public.sync_target_slew_rpm_s;
    config = m_ctx.sync_config;
    last_update = m_ctx.sync_last_update_ms;
    cached = m_ctx.sync_output;
    portEXIT_CRITICAL(&m_ctx.lock);

    const bool left_valid = mx->controller[VCU_MX_CAN_LEFT].has_status &&
            now - mx->controller[VCU_MX_CAN_LEFT].last_status_ms <= VCU_SYNC_STATUS_TIMEOUT_MS;
    const bool right_valid = mx->controller[VCU_MX_CAN_RIGHT].has_status &&
            now - mx->controller[VCU_MX_CAN_RIGHT].last_status_ms <= VCU_SYNC_STATUS_TIMEOUT_MS;

    if (last_update == 0U || now - last_update >= VCU_SYNC_PERIOD_MS) {
        const uint32_t elapsed = last_update == 0U ? VCU_SYNC_PERIOD_MS : now - last_update;
        const vcu_speed_sync_input_t input = {
            .enabled = enabled,
            .left_can_valid = left_valid,
            .right_can_valid = right_valid,
            .left_fault = mx->controller[VCU_MX_CAN_LEFT].status.fault_code != 0U,
            .right_fault = mx->controller[VCU_MX_CAN_RIGHT].status.fault_code != 0U,
            .dt_seconds = (float)elapsed / 1000.0f,
            .left_actual_rpm = mx->controller[VCU_MX_CAN_LEFT].status.axis2_speed_rpm,
            .right_actual_rpm = mx->controller[VCU_MX_CAN_RIGHT].status.axis2_speed_rpm,
            .left_theoretical_target_rpm = left_base,
            .right_theoretical_target_rpm = right_base,
        };
        portENTER_CRITICAL(&m_ctx.lock);
        const bool ok = vcu_speed_sync_step(&m_ctx.sync_state, &config, &input, &cached);
        if (ok) {
            m_ctx.sync_output = cached;
            m_ctx.sync_last_update_ms = now;
        }
        portEXIT_CRITICAL(&m_ctx.lock);
        if (!ok) return false;
    }

    publish_sync_state(&cached, left_valid, right_valid);
    int32_t left_limited;
    int32_t right_limited;
    portENTER_CRITICAL(&m_ctx.lock);
    const bool left_ok = vcu_target_slew_step(&m_ctx.left_slew, cached.left_target_rpm,
            slew_rate, (float)loop_period_ms / 1000.0f, &left_limited);
    const bool right_ok = vcu_target_slew_step(&m_ctx.right_slew, cached.right_target_rpm,
            slew_rate, (float)loop_period_ms / 1000.0f, &right_limited);
    portEXIT_CRITICAL(&m_ctx.lock);
    if (!left_ok || !right_ok) return false;

    const int16_t left_final = clamp_i16_target(left_limited, max_target);
    const int16_t right_final = clamp_i16_target(right_limited, max_target);
    if (!vcu_mx_can_test_send_axis2_targets(left_final, right_final, can_ramp, true)) return false;

    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.public.latest_left_base_target = left_base;
    m_ctx.public.latest_right_base_target = right_base;
    m_ctx.public.left_pre_slew_target = clamp_i16_target(cached.left_target_rpm, max_target);
    m_ctx.public.right_pre_slew_target = clamp_i16_target(cached.right_target_rpm, max_target);
    m_ctx.public.latest_left_target = left_final;
    m_ctx.public.latest_right_target = right_final;
    m_ctx.public.tx_count++;
    portEXIT_CRITICAL(&m_ctx.lock);
    return true;
}

static void drive_task(void *arg) {
    (void)arg;
    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.public.task_running = true;
    portEXIT_CRITICAL(&m_ctx.lock);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vcu_drive_runtime_state_t runtime;
        portENTER_CRITICAL(&m_ctx.lock);
        runtime = m_ctx.public;
        m_ctx.public.loop_count++;
        portEXIT_CRITICAL(&m_ctx.lock);

        vcu_crsf_uart_state_t crsf;
        vcu_rc_drive_config_t rc_config;
        vcu_rc_drive_preview_t preview;
        vcu_mx_can_test_state_t mx;
        vcu_crsf_uart_get_state(&crsf);
        vcu_rc_drive_get_default_config(&rc_config);
        vcu_rc_drive_preview_from_channels(crsf.channels, crsf.link_ok, &rc_config, &preview);
        vcu_mx_can_test_get_state(&mx);

        const uint32_t now = now_ms();
        const bool status_recent = controllers_recent(&mx, now, VCU_DRIVE_STATUS_TIMEOUT_MS);
        const bool fault_free = controllers_fault_free(&mx);
        const bool neutral = neutral_controls(&preview);
        bool reset_edge = false;
        portENTER_CRITICAL(&m_ctx.lock);
        m_ctx.public.reset_requested = preview.reset_requested;
        reset_edge = preview.reset_requested && !m_ctx.reset_latched;
        m_ctx.reset_latched = preview.reset_requested;
        portEXIT_CRITICAL(&m_ctx.lock);

        if (reset_edge) {
            send_zero(); send_reset(); set_runtime_state(VCU_DRIVE_STATE_DISARMED, "ch8 reset");
        } else if (!runtime.requested_run) {
            send_zero(); set_runtime_state(VCU_DRIVE_STATE_STOPPED, "stopped");
        } else if (!preview.link_ok) {
            send_zero(); set_runtime_state(VCU_DRIVE_STATE_DISARMED, "crsf link lost");
        } else if (!status_recent) {
            send_zero(); set_runtime_state(VCU_DRIVE_STATE_DISARMED, "controller status timeout");
        } else if (!fault_free) {
            send_zero(); set_runtime_state(VCU_DRIVE_STATE_FAULT, "controller fault");
        } else if (!preview.arm_requested) {
            send_zero();
            portENTER_CRITICAL(&m_ctx.lock); m_ctx.neutral_since_ms = 0U; portEXIT_CRITICAL(&m_ctx.lock);
            set_runtime_state(VCU_DRIVE_STATE_DISARMED, "ch7 kill");
        } else if (runtime.state != VCU_DRIVE_STATE_ARMED) {
            if (!neutral) {
                send_zero();
                portENTER_CRITICAL(&m_ctx.lock); m_ctx.neutral_since_ms = 0U; portEXIT_CRITICAL(&m_ctx.lock);
                set_runtime_state(VCU_DRIVE_STATE_ARMING_NEUTRAL, "waiting neutral");
            } else {
                uint32_t neutral_since;
                portENTER_CRITICAL(&m_ctx.lock);
                if (m_ctx.neutral_since_ms == 0U) m_ctx.neutral_since_ms = now;
                neutral_since = m_ctx.neutral_since_ms;
                portEXIT_CRITICAL(&m_ctx.lock);
                send_zero();
                set_runtime_state(now - neutral_since >= VCU_DRIVE_NEUTRAL_DWELL_MS ?
                        VCU_DRIVE_STATE_ARMED : VCU_DRIVE_STATE_ARMING_NEUTRAL,
                        now - neutral_since >= VCU_DRIVE_NEUTRAL_DWELL_MS ? "armed" : "neutral dwell");
            }
        } else {
            const int16_t left_base = scale_target(preview.axis1_per_mille, runtime.max_target);
            const int16_t right_base = scale_target(preview.axis2_per_mille, runtime.max_target);
            if (run_normal_control(left_base, right_base, &mx, now, runtime.period_ms,
                    runtime.max_target, runtime.can_ramp)) {
                set_runtime_state(VCU_DRIVE_STATE_ARMED, "running");
            } else {
                send_zero(); set_runtime_state(VCU_DRIVE_STATE_FAULT, "control calculation failed");
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(runtime.period_ms));
    }
}

bool vcu_drive_runtime_get_state(vcu_drive_runtime_state_t *state) {
    if (state == NULL) return false;
    portENTER_CRITICAL(&m_ctx.lock);
    *state = m_ctx.public;
    portEXIT_CRITICAL(&m_ctx.lock);
    return true;
}

static long arg_long(const char *arg) { return strtol(arg, NULL, 0); }

static void terminal_sync_enable(int argc, const char **argv) {
    if (argc < 2) { commands_printf("Usage: vcu_sync_enable <0|1>"); return; }
    portENTER_CRITICAL(&m_ctx.lock);
    m_ctx.public.sync_enabled = arg_long(argv[1]) != 0;
    vcu_speed_sync_reset(&m_ctx.sync_state);
    m_ctx.sync_last_update_ms = 0U;
    portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("Speed synchronization %s", m_ctx.public.sync_enabled ? "enabled" : "disabled");
}

static void terminal_sync_kp(int argc, const char **argv) {
    if (argc < 2) { commands_printf("Usage: vcu_sync_kp <0.0..10.0>"); return; }
    const float kp = strtof(argv[1], NULL);
    if (!isfinite(kp) || kp < 0.0f || kp > 10.0f) { commands_printf("Invalid Kp"); return; }
    portENTER_CRITICAL(&m_ctx.lock); m_ctx.sync_config.kp = kp; m_ctx.public.sync_kp = kp; portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("Speed synchronization Kp: %.4f", (double)kp);
}

static void terminal_sync_deadzone(int argc, const char **argv) {
    if (argc < 2) { commands_printf("Usage: vcu_sync_deadzone <rpm>"); return; }
    const long value = arg_long(argv[1]); if (value < 0 || value > 1000) { commands_printf("Invalid deadzone"); return; }
    portENTER_CRITICAL(&m_ctx.lock); m_ctx.sync_config.error_deadzone_rpm = value; m_ctx.public.sync_deadzone_rpm = value; portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("Speed synchronization deadzone: %ld rpm", value);
}

static void terminal_sync_limits(int argc, const char **argv) {
    if (argc < 4) { commands_printf("Usage: vcu_sync_limits <low> <medium> <high>"); return; }
    const long a=arg_long(argv[1]),b=arg_long(argv[2]),c=arg_long(argv[3]);
    if (a<=0||b<=0||c<=0||a>4800||b>4800||c>4800) { commands_printf("Invalid limits"); return; }
    portENTER_CRITICAL(&m_ctx.lock); m_ctx.sync_config.low_limit_rpm=a;m_ctx.sync_config.medium_limit_rpm=b;m_ctx.sync_config.high_limit_rpm=c;m_ctx.public.sync_low_limit_rpm=a;m_ctx.public.sync_medium_limit_rpm=b;m_ctx.public.sync_high_limit_rpm=c;portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("Speed synchronization limits: %ld %ld %ld rpm",a,b,c);
}

static void terminal_sync_thresholds(int argc, const char **argv) {
    if (argc < 4) { commands_printf("Usage: vcu_sync_thresholds <medium> <high> <weak>"); return; }
    const long a=arg_long(argv[1]),b=arg_long(argv[2]),c=arg_long(argv[3]);
    if (a<=0||b<=a||c<=0) { commands_printf("Invalid thresholds"); return; }
    portENTER_CRITICAL(&m_ctx.lock);m_ctx.sync_config.medium_speed_threshold_rpm=a;m_ctx.sync_config.high_speed_threshold_rpm=b;m_ctx.sync_config.weak_field_threshold_rpm=c;m_ctx.public.sync_medium_threshold_rpm=a;m_ctx.public.sync_high_threshold_rpm=b;m_ctx.public.sync_weak_threshold_rpm=c;portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("Speed thresholds: medium=%ld high=%ld weak=%ld rpm",a,b,c);
}

static void terminal_sync_slew(int argc, const char **argv) {
    if (argc < 2) { commands_printf("Usage: vcu_sync_slew <rpm/s>"); return; }
    const long value=arg_long(argv[1]);if(value<=0||value>20000){commands_printf("Invalid slew rate");return;}
    portENTER_CRITICAL(&m_ctx.lock);m_ctx.public.sync_target_slew_rpm_s=value;portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("VCU target slew: %ld rpm/s",value);
}

static void terminal_sync_derate(int argc, const char **argv) {
    if(argc<5){commands_printf("Usage: vcu_sync_derate <error_rpm> <hold_ms> <step_rpm> <minimum_rpm>");return;}
    const long e=arg_long(argv[1]),h=arg_long(argv[2]),s=arg_long(argv[3]),m=arg_long(argv[4]);
    if(e<=0||h<=0||s<=0||m<0){commands_printf("Invalid derate configuration");return;}
    portENTER_CRITICAL(&m_ctx.lock);m_ctx.sync_config.large_error_threshold_rpm=e;m_ctx.sync_config.large_error_hold_ms=h;m_ctx.sync_config.longitudinal_derate_step_rpm=s;m_ctx.sync_config.minimum_longitudinal_target_rpm=m;m_ctx.public.sync_large_error_threshold_rpm=e;m_ctx.public.sync_large_error_hold_ms=h;m_ctx.public.sync_derate_step_rpm=s;m_ctx.public.sync_minimum_target_rpm=m;portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("Derate: error=%ld hold=%ld step=%ld minimum=%ld",e,h,s,m);
}

static void terminal_sync_status(int argc, const char **argv) {
    (void)argc;(void)argv;vcu_drive_runtime_state_t s;vcu_drive_runtime_get_state(&s);
    commands_printf("Speed synchronization\n"
            "enabled      : %d\nKp           : %.4f\ndeadzone     : %ld rpm\n"
            "limits       : %ld/%ld/%ld rpm\nthresholds   : %ld/%ld weak=%ld rpm\n"
            "slew         : %ld rpm/s\nweak field   : %d\naverage      : %ld rpm\n"
            "theory diff  : %ld rpm\nactual diff  : %ld rpm\nraw error    : %ld rpm\n"
            "effective    : %ld rpm\ncorrection   : %.1f -> %ld rpm\n"
            "derate       : %ld rpm, timer=%lu ms\ndiag         : 0x%08lx",
            s.sync_enabled,(double)s.sync_kp,(long)s.sync_deadzone_rpm,
            (long)s.sync_low_limit_rpm,(long)s.sync_medium_limit_rpm,(long)s.sync_high_limit_rpm,
            (long)s.sync_medium_threshold_rpm,(long)s.sync_high_threshold_rpm,(long)s.sync_weak_threshold_rpm,
            (long)s.sync_target_slew_rpm_s,s.weak_field,(long)s.average_rpm,
            (long)s.theoretical_difference_rpm,(long)s.actual_difference_rpm,(long)s.sync_raw_error_rpm,
            (long)s.sync_effective_error_rpm,(double)s.sync_raw_correction_rpm,(long)s.sync_limited_correction_rpm,
            (long)s.sync_accumulated_derate_rpm,(unsigned long)s.sync_large_error_elapsed_ms,
            (unsigned long)s.sync_diagnostic_flags);
}

static void terminal_drive_start(int argc, const char **argv) {
    uint32_t period=VCU_DRIVE_DEFAULT_PERIOD_MS,accel=VCU_DRIVE_DEFAULT_ACCEL_RPM_S;
    if (argc >= 2) period = (uint32_t)arg_long(argv[1]);
    if (argc >= 3) accel = (uint32_t)arg_long(argv[2]);
    if (period < VCU_DRIVE_MIN_PERIOD_MS) period = VCU_DRIVE_MIN_PERIOD_MS;
    if (period > VCU_DRIVE_MAX_PERIOD_MS) period = VCU_DRIVE_MAX_PERIOD_MS;
    if (accel < VCU_DRIVE_MIN_ACCEL_RPM_S) accel = VCU_DRIVE_MIN_ACCEL_RPM_S;
    if (accel > VCU_DRIVE_MAX_ACCEL_RPM_S) accel = VCU_DRIVE_MAX_ACCEL_RPM_S;
    portENTER_CRITICAL(&m_ctx.lock);m_ctx.public.period_ms=period;m_ctx.public.accel_rpm_s=accel;m_ctx.public.can_ramp=accel_to_can_ramp(accel);m_ctx.public.requested_run=true;m_ctx.neutral_since_ms=0U;portEXIT_CRITICAL(&m_ctx.lock);
    commands_printf("VCU drive requested: period=%lu ms ramp=%u",(unsigned long)period,accel_to_can_ramp(accel));
}

static void terminal_drive_stop(int argc,const char **argv){(void)argc;(void)argv;portENTER_CRITICAL(&m_ctx.lock);m_ctx.public.requested_run=false;m_ctx.neutral_since_ms=0U;portEXIT_CRITICAL(&m_ctx.lock);send_zero();commands_printf("VCU drive stop requested");}

static void terminal_drive_status(int argc,const char **argv){(void)argc;(void)argv;vcu_drive_runtime_state_t s;vcu_drive_runtime_get_state(&s);commands_printf("VCU drive runtime\ninitialized : %d\ntask running: %d\nrequested   : %d\nstate       : %s\nreason      : %s\nperiod      : %lu ms\nleft target : %d\nright target: %d\nloops       : %lu\ndrive tx    : %lu\nzero tx     : %lu",s.initialized,s.task_running,s.requested_run,vcu_drive_runtime_state_name(s.state),s.reason,(unsigned long)s.period_ms,s.latest_left_target,s.latest_right_target,(unsigned long)s.loop_count,(unsigned long)s.tx_count,(unsigned long)s.zero_tx_count);}

void vcu_drive_runtime_init(void) {
    if(m_ctx.public.initialized)return;
    vcu_speed_sync_default_config(&m_ctx.sync_config);
    m_ctx.public.sync_kp=m_ctx.sync_config.kp;m_ctx.public.sync_deadzone_rpm=m_ctx.sync_config.error_deadzone_rpm;
    m_ctx.public.sync_low_limit_rpm=m_ctx.sync_config.low_limit_rpm;m_ctx.public.sync_medium_limit_rpm=m_ctx.sync_config.medium_limit_rpm;m_ctx.public.sync_high_limit_rpm=m_ctx.sync_config.high_limit_rpm;
    m_ctx.public.sync_medium_threshold_rpm=m_ctx.sync_config.medium_speed_threshold_rpm;m_ctx.public.sync_high_threshold_rpm=m_ctx.sync_config.high_speed_threshold_rpm;m_ctx.public.sync_weak_threshold_rpm=m_ctx.sync_config.weak_field_threshold_rpm;
    m_ctx.public.sync_large_error_threshold_rpm=m_ctx.sync_config.large_error_threshold_rpm;m_ctx.public.sync_large_error_hold_ms=m_ctx.sync_config.large_error_hold_ms;m_ctx.public.sync_derate_step_rpm=m_ctx.sync_config.longitudinal_derate_step_rpm;m_ctx.public.sync_minimum_target_rpm=m_ctx.sync_config.minimum_longitudinal_target_rpm;
    terminal_register_command_callback("vcu_drive_start","Start guarded RC to CAN runtime.","[period_ms] [ramp_rpm_s]",terminal_drive_start);
    terminal_register_command_callback("vcu_drive_stop","Stop guarded RC to CAN runtime.",0,terminal_drive_stop);
    terminal_register_command_callback("vcu_drive_status","Print drive runtime status.",0,terminal_drive_status);
    terminal_register_command_callback("vcu_sync_enable","Enable speed synchronization.","<0|1>",terminal_sync_enable);
    terminal_register_command_callback("vcu_sync_kp","Set synchronization Kp.","<kp>",terminal_sync_kp);
    terminal_register_command_callback("vcu_sync_deadzone","Set synchronization deadzone.","<rpm>",terminal_sync_deadzone);
    terminal_register_command_callback("vcu_sync_limits","Set dynamic correction limits.","<low> <medium> <high>",terminal_sync_limits);
    terminal_register_command_callback("vcu_sync_thresholds","Set speed thresholds.","<medium> <high> <weak>",terminal_sync_thresholds);
    terminal_register_command_callback("vcu_sync_slew","Set VCU target slew.","<rpm/s>",terminal_sync_slew);
    terminal_register_command_callback("vcu_sync_derate","Set weak-field derating.","<error> <hold_ms> <step> <minimum>",terminal_sync_derate);
    terminal_register_command_callback("vcu_sync_status","Print synchronization status.",0,terminal_sync_status);
    portENTER_CRITICAL(&m_ctx.lock);m_ctx.public.initialized=true;strncpy(m_ctx.public.reason,"init",sizeof(m_ctx.public.reason)-1U);portEXIT_CRITICAL(&m_ctx.lock);
    if(xTaskCreatePinnedToCore(drive_task,"vcu_drive",VCU_DRIVE_TASK_STACK_SIZE,NULL,VCU_DRIVE_TASK_PRIORITY,NULL,tskNO_AFFINITY)!=pdPASS)set_runtime_state(VCU_DRIVE_STATE_FAULT,"task create failed");
}
