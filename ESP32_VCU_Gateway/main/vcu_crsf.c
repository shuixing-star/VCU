#include "vcu_crsf.h"

#include <string.h>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8U
#define CRSF_FRAME_RC_CHANNELS_PACKED 0x16U
#define CRSF_RC_PAYLOAD_SIZE 22U

static uint8_t crc8_d5(const uint8_t *data, size_t size) {
    uint8_t crc = 0U;
    for (size_t i = 0U; i < size; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1U) ^ 0xD5U)
                                      : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static void unpack_channels(const uint8_t *payload, uint16_t *channels) {
    uint32_t bit_buffer = 0U;
    uint8_t bits_available = 0U;
    size_t payload_index = 0U;

    for (size_t channel = 0U; channel < VCU_CRSF_CHANNEL_COUNT; channel++) {
        while (bits_available < 11U) {
            bit_buffer |= (uint32_t)payload[payload_index++] << bits_available;
            bits_available = (uint8_t)(bits_available + 8U);
        }
        channels[channel] = (uint16_t)(bit_buffer & 0x07FFU);
        bit_buffer >>= 11U;
        bits_available = (uint8_t)(bits_available - 11U);
    }
}

static void reset_frame(vcu_crsf_t *crsf) {
    crsf->frame_size = 0U;
    crsf->frame_expected = 0U;
}

static void process_frame(vcu_crsf_t *crsf, uint32_t now_ms) {
    const uint8_t length = crsf->frame[1];
    const uint8_t received_crc = crsf->frame[crsf->frame_expected - 1U];

    if (crc8_d5(&crsf->frame[2], (size_t)length - 1U) != received_crc) {
        crsf->crc_error_count++;
        return;
    }
    if (crsf->frame[2] != CRSF_FRAME_RC_CHANNELS_PACKED ||
        length != CRSF_RC_PAYLOAD_SIZE + 2U) {
        return;
    }

    unpack_channels(&crsf->frame[3], crsf->channels);
    crsf->last_valid_frame_ms = now_ms;
    crsf->valid_frame_count++;
    crsf->has_valid_frame = true;
}

void vcu_crsf_init(vcu_crsf_t *crsf, uint16_t default_channel) {
    if (crsf == NULL) return;
    memset(crsf, 0, sizeof(*crsf));
    for (size_t i = 0U; i < VCU_CRSF_CHANNEL_COUNT; i++) {
        crsf->channels[i] = default_channel;
    }
}

void vcu_crsf_feed(vcu_crsf_t *crsf, const uint8_t *data, size_t size, uint32_t now_ms) {
    if (crsf == NULL || data == NULL) return;

    for (size_t i = 0U; i < size; i++) {
        const uint8_t byte = data[i];
        if (crsf->frame_size == 0U) {
            if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                crsf->frame[crsf->frame_size++] = byte;
            }
            continue;
        }

        crsf->frame[crsf->frame_size++] = byte;
        if (crsf->frame_size == 2U) {
            const size_t total = (size_t)crsf->frame[1] + 2U;
            if (crsf->frame[1] < 2U || total > sizeof(crsf->frame)) {
                reset_frame(crsf);
                if (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                    crsf->frame[crsf->frame_size++] = byte;
                }
                continue;
            }
            crsf->frame_expected = total;
        }

        if (crsf->frame_expected != 0U && crsf->frame_size == crsf->frame_expected) {
            process_frame(crsf, now_ms);
            reset_frame(crsf);
        }
    }
}

bool vcu_crsf_link_ok(const vcu_crsf_t *crsf, uint32_t now_ms, uint32_t timeout_ms) {
    return crsf != NULL && crsf->has_valid_frame &&
           (uint32_t)(now_ms - crsf->last_valid_frame_ms) <= timeout_ms;
}

uint16_t vcu_crsf_channel(const vcu_crsf_t *crsf, uint8_t index) {
    return crsf == NULL || index >= VCU_CRSF_CHANNEL_COUNT ? 0U : crsf->channels[index];
}
