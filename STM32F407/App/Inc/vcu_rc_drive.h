#ifndef VCU_RC_DRIVE_H
#define VCU_RC_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t throttle_channel;
    uint8_t steering_channel;
    int8_t throttle_sign;
    int8_t steering_sign;
    uint8_t arm_channel;
    uint8_t reset_channel;
    uint8_t brake_channel;
    uint16_t channel_min;
    uint16_t channel_mid;
    uint16_t channel_max;
    uint16_t deadband;
    uint16_t arm_threshold;
    uint16_t reset_threshold;
    uint16_t brake_threshold;
    int16_t preview_max_target;
} vcu_rc_drive_config_t;

typedef struct {
    bool link_ok;
    uint16_t throttle_raw;
    uint16_t steering_raw;
    uint16_t arm_raw;
    uint16_t reset_raw;
    bool arm_requested;
    bool reset_requested;
    bool drive_allowed;
    uint16_t brake_raw;
    bool brake_requested;
    int16_t throttle_per_mille;
    int16_t steering_per_mille;
    int16_t axis1_per_mille;
    int16_t axis2_per_mille;
    int16_t axis1_preview_target;
    int16_t axis2_preview_target;
} vcu_rc_drive_preview_t;

void vcu_rc_drive_get_default_config(vcu_rc_drive_config_t *config);
bool vcu_rc_drive_preview_from_channels(const uint16_t *channels,
                                        bool link_ok,
                                        const vcu_rc_drive_config_t *config,
                                        vcu_rc_drive_preview_t *preview);

#endif
