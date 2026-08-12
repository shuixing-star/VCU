#include "vcu_crsf_uart.h"

#include <string.h>

#include "vcu_crsf_port.h"

#define VCU_CRSF_BAUDRATE 420000U
#define VCU_CRSF_LINK_TIMEOUT_MS 150U
#define VCU_CRSF_DRAIN_CHUNK_SIZE 64U
#define VCU_CRSF_DRAIN_CHUNK_BUDGET 4U

static vcu_crsf_t s_parser;
static bool s_initialized;

bool vcu_crsf_uart_init(void)
{
    vcu_crsf_init(&s_parser, VCU_CRSF_CHANNEL_MID);
    s_initialized = vcu_crsf_port_init(VCU_CRSF_BAUDRATE);
    return s_initialized;
}

void vcu_crsf_uart_step(uint32_t now_ms)
{
    uint8_t data[VCU_CRSF_DRAIN_CHUNK_SIZE];
    uint32_t received_ms[VCU_CRSF_DRAIN_CHUNK_SIZE];
    size_t count;
    size_t index;
    size_t chunks_processed = 0U;

    (void)now_ms;

    if (!s_initialized) {
        return;
    }

    do {
        count = vcu_crsf_port_read(
                data, received_ms, sizeof(data));
        chunks_processed++;
        for (index = 0U; index < count; index++) {
            vcu_crsf_feed(
                    &s_parser, &data[index], 1U,
                    received_ms[index]);
        }
    } while (count == sizeof(data) &&
             chunks_processed < VCU_CRSF_DRAIN_CHUNK_BUDGET);
}

bool vcu_crsf_uart_get_state(uint32_t now_ms,
                             vcu_crsf_uart_state_t *state)
{
    if (state == NULL) {
        return false;
    }

    memset(state, 0, sizeof(*state));
    state->initialized = s_initialized;
    state->uart_error_count = vcu_crsf_port_error_count();
    state->uart_overflow_count = vcu_crsf_port_overflow_count();
    state->valid_frame_count = s_parser.valid_frame_count;
    state->crc_error_count = s_parser.crc_error_count;
    state->last_valid_frame_ms = s_parser.last_valid_frame_ms;
    memcpy(state->channels, s_parser.channels, sizeof(state->channels));
    state->link_ok = s_initialized &&
            vcu_crsf_link_ok(&s_parser, now_ms,
                             VCU_CRSF_LINK_TIMEOUT_MS);
    return true;
}
