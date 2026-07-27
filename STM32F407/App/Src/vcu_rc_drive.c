#include "vcu_rc_drive.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "vcu_crsf.h"

#define VCU_RC_DRIVE_THROTTLE_CH 3U
#define VCU_RC_DRIVE_STEERING_CH 1U
#define VCU_RC_DRIVE_THROTTLE_SIGN 1
#define VCU_RC_DRIVE_STEERING_SIGN -1
#define VCU_RC_DRIVE_ARM_CH 7U
#define VCU_RC_DRIVE_RESET_CH 8U
#define VCU_RC_DRIVE_MIN 172U
#define VCU_RC_DRIVE_MID 992U
#define VCU_RC_DRIVE_MAX 1811U
#define VCU_RC_DRIVE_DEADBAND 80U
#define VCU_RC_DRIVE_ARM_THRESHOLD 1500U
#define VCU_RC_DRIVE_RESET_THRESHOLD 500U
#define VCU_RC_DRIVE_PREVIEW_MAX_TARGET 1000

static int16_t clamp_i32_to_i16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static int16_t clamp_per_mille(int32_t value)
{
    if (value > VCU_RC_DRIVE_PREVIEW_MAX_TARGET) {
        return VCU_RC_DRIVE_PREVIEW_MAX_TARGET;
    }
    if (value < -VCU_RC_DRIVE_PREVIEW_MAX_TARGET) {
        return -VCU_RC_DRIVE_PREVIEW_MAX_TARGET;
    }
    return (int16_t)value;
}

static int16_t normalize_channel(uint16_t raw,
                                 const vcu_rc_drive_config_t *config)
{
    const int32_t delta = (int32_t)raw - (int32_t)config->channel_mid;

    if (delta >= -(int32_t)config->deadband &&
        delta <= (int32_t)config->deadband) {
        return 0;
    }

    if (delta > 0) {
        const int32_t span = (int32_t)config->channel_max -
                             (int32_t)config->channel_mid -
                             (int32_t)config->deadband;
        if (span <= 0) {
            return 0;
        }
        return clamp_per_mille(
                ((delta - (int32_t)config->deadband) * 1000) / span);
    }

    {
        const int32_t span = (int32_t)config->channel_mid -
                             (int32_t)config->channel_min -
                             (int32_t)config->deadband;
        if (span <= 0) {
            return 0;
        }
        return clamp_per_mille(
                ((delta + (int32_t)config->deadband) * 1000) / span);
    }
}

static int16_t scale_preview_target(int16_t per_mille, int16_t max_target)
{
    return clamp_i32_to_i16(
            ((int32_t)per_mille * (int32_t)max_target) / 1000);
}

void vcu_rc_drive_get_default_config(vcu_rc_drive_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->throttle_channel = VCU_RC_DRIVE_THROTTLE_CH;
    config->steering_channel = VCU_RC_DRIVE_STEERING_CH;
    config->throttle_sign = VCU_RC_DRIVE_THROTTLE_SIGN;
    config->steering_sign = VCU_RC_DRIVE_STEERING_SIGN;
    config->arm_channel = VCU_RC_DRIVE_ARM_CH;
    config->reset_channel = VCU_RC_DRIVE_RESET_CH;
    config->channel_min = VCU_RC_DRIVE_MIN;
    config->channel_mid = VCU_RC_DRIVE_MID;
    config->channel_max = VCU_RC_DRIVE_MAX;
    config->deadband = VCU_RC_DRIVE_DEADBAND;
    config->arm_threshold = VCU_RC_DRIVE_ARM_THRESHOLD;
    config->reset_threshold = VCU_RC_DRIVE_RESET_THRESHOLD;
    config->preview_max_target = VCU_RC_DRIVE_PREVIEW_MAX_TARGET;
}

bool vcu_rc_drive_preview_from_channels(const uint16_t *channels,
                                        bool link_ok,
                                        const vcu_rc_drive_config_t *config,
                                        vcu_rc_drive_preview_t *preview)
{
    if (channels == NULL || config == NULL || preview == NULL ||
        config->throttle_channel < 1U ||
        config->throttle_channel > VCU_CRSF_CHANNEL_COUNT ||
        config->steering_channel < 1U ||
        config->steering_channel > VCU_CRSF_CHANNEL_COUNT ||
        config->arm_channel < 1U ||
        config->arm_channel > VCU_CRSF_CHANNEL_COUNT ||
        config->reset_channel < 1U ||
        config->reset_channel > VCU_CRSF_CHANNEL_COUNT) {
        return false;
    }

    memset(preview, 0, sizeof(*preview));
    preview->link_ok = link_ok;
    preview->throttle_raw = channels[config->throttle_channel - 1U];
    preview->steering_raw = channels[config->steering_channel - 1U];
    preview->arm_raw = channels[config->arm_channel - 1U];
    preview->reset_raw = channels[config->reset_channel - 1U];
    preview->arm_requested = preview->arm_raw >= config->arm_threshold;
    preview->reset_requested = preview->reset_raw <= config->reset_threshold;
    preview->drive_allowed = link_ok && preview->arm_requested;
    preview->throttle_per_mille =
            (int16_t)(normalize_channel(preview->throttle_raw, config) *
                      config->throttle_sign);
    preview->steering_per_mille =
            (int16_t)(normalize_channel(preview->steering_raw, config) *
                      config->steering_sign);

    preview->axis1_per_mille =
            clamp_per_mille((int32_t)preview->throttle_per_mille +
                            (int32_t)preview->steering_per_mille);
    preview->axis2_per_mille =
            clamp_per_mille((int32_t)preview->throttle_per_mille -
                            (int32_t)preview->steering_per_mille);
    preview->axis1_preview_target =
            scale_preview_target(preview->axis1_per_mille,
                                 config->preview_max_target);
    preview->axis2_preview_target =
            scale_preview_target(preview->axis2_per_mille,
                                 config->preview_max_target);
    return true;
}
