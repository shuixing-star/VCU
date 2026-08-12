#include "vcu_app.h"

#include "vcu_can_port.h"
#include "vcu_control_telemetry.h"
#include "vcu_crsf_uart.h"
#include "vcu_esp_link.h"
#include "vcu_drive_runtime.h"
#include "vcu_motor_can.h"

#define VCU_APP_CONTROL_OVERRUN_MARGIN_MS 20U
#define VCU_APP_CAN_FRAME_BUDGET 16U

static bool s_initialized;
static bool s_control_started;
static uint32_t s_last_control_ms;

bool vcu_app_init(void)
{
    s_last_control_ms = 0U;
    s_control_started = false;
    vcu_control_telemetry_init();
    vcu_esp_link_init();

    s_initialized = vcu_crsf_uart_init() &&
                    vcu_motor_can_init() &&
                    vcu_drive_runtime_init();

    if (!s_initialized) {
        return false;
    }

    s_initialized =
            vcu_motor_can_send_targets(0, 0, 0U, false);
    return s_initialized;
}

void vcu_app_step(uint32_t now_ms)
{
    vcu_can_port_frame_t frame;
    vcu_gateway_command_t command;
    vcu_crsf_uart_state_t crsf;
    vcu_motor_can_state_t motors;
    vcu_drive_runtime_state_t drive;
    uint32_t control_period_ms;
    uint32_t control_elapsed_ms;
    uint32_t can_frames_processed = 0U;

    if (!s_initialized) {
        return;
    }

    vcu_crsf_uart_step(now_ms);

    while (can_frames_processed < VCU_APP_CAN_FRAME_BUDGET &&
           vcu_can_port_read(&frame)) {
        can_frames_processed++;
        if (!vcu_esp_link_process_frame(&frame)) {
            (void)vcu_motor_can_process_frame(
                    &frame, frame.received_ms);
        }
    }

    if (vcu_esp_link_take_command(&command)) {
        switch (command.type) {
        case VCU_GATEWAY_COMMAND_RUN:
            vcu_drive_runtime_request_run(
                    command.period_ms,
                    command.acceleration_rpm_s);
            break;
        case VCU_GATEWAY_COMMAND_STOP:
            vcu_drive_runtime_request_stop();
            break;
        case VCU_GATEWAY_COMMAND_RESET:
            vcu_drive_runtime_request_reset();
            break;
        default:
            break;
        }
    }

    (void)vcu_crsf_uart_get_state(now_ms, &crsf);
    (void)vcu_motor_can_get_state(&motors);
    control_period_ms = vcu_drive_runtime_control_period_ms();

    if (!s_control_started) {
        s_control_started = true;
        s_last_control_ms = now_ms;
    } else {
        control_elapsed_ms =
                (uint32_t)(now_ms - s_last_control_ms);

        if (control_elapsed_ms >= control_period_ms) {
            s_last_control_ms = now_ms;
            if (control_elapsed_ms >
                control_period_ms +
                VCU_APP_CONTROL_OVERRUN_MARGIN_MS) {
                vcu_drive_runtime_report_schedule_overrun(now_ms);
            } else {
                vcu_drive_runtime_step(now_ms, &crsf, &motors);
            }
        }
    }

    (void)vcu_drive_runtime_get_state(&drive);
    vcu_control_telemetry_step(now_ms, &crsf, &drive, &motors);
}
