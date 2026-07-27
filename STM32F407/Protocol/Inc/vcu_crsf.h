#ifndef VCU_CRSF_H
#define VCU_CRSF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VCU_CRSF_CHANNEL_COUNT 16U
#define VCU_CRSF_CHANNEL_MID 992U
#define VCU_CRSF_MAX_FRAME_SIZE 64U

typedef struct {
    uint16_t channels[VCU_CRSF_CHANNEL_COUNT];
    uint8_t frame[VCU_CRSF_MAX_FRAME_SIZE];
    size_t frame_size;
    size_t frame_expected;
    uint32_t last_valid_frame_ms;
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    bool has_valid_frame;
} vcu_crsf_t;

void vcu_crsf_init(vcu_crsf_t *crsf, uint16_t default_channel);
void vcu_crsf_feed(vcu_crsf_t *crsf, const uint8_t *data, size_t size, uint32_t now_ms);
bool vcu_crsf_link_ok(const vcu_crsf_t *crsf, uint32_t now_ms, uint32_t timeout_ms);
uint16_t vcu_crsf_channel(const vcu_crsf_t *crsf, uint8_t index);

#endif
