#include "f280_fw_stream.h"

#include <string.h>

#include "esp_attr.h"
#include "comm_can.h"
#include "f280_fw_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define STREAM_PREFIX_BYTES       6U
#define STREAM_ACK_TIMEOUT_MS     1000U
#define STREAM_BOOT_TIMEOUT_MS    5000U
#define STREAM_ERASE_TIMEOUT_MS   12000U
#define STREAM_BLOCK_RETRIES      3U
#define STREAM_MAX_IMAGE_BYTES    (145U * 1024U)

#define STREAM_RETAINED_MAGIC     0x46323830UL
#define STREAM_RETAINED_VERSION   1U
typedef struct {
    bool initialized;
    SemaphoreHandle_t lock;
    SemaphoreHandle_t response_sem;
    uint8_t response[F280_UPD_RESPONSE_FRAME_BYTES];
    uint8_t prefix[STREAM_PREFIX_BYTES];
    uint8_t prefix_mask;
    uint16_t running_crc;
    uint16_t expected_crc;
    f280_fw_stream_status_t public;
} stream_context_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    f280_fw_stream_status_t status;
} stream_retained_status_t;

static stream_context_t s_stream;
RTC_NOINIT_ATTR static stream_retained_status_t s_retained_status;

static void retain_status_locked(void)
{
    /* Commit magic last so a reset during the copy cannot validate a tear. */
    s_retained_status.magic = 0U;
    s_retained_status.version = STREAM_RETAINED_VERSION;
    s_retained_status.status = s_stream.public;
    s_retained_status.magic = STREAM_RETAINED_MAGIC;
}

static bool retained_status_valid(void)
{
    return s_retained_status.magic == STREAM_RETAINED_MAGIC &&
           s_retained_status.version == STREAM_RETAINED_VERSION &&
           s_retained_status.status.state <= F280_FW_STREAM_FAILED &&
           s_retained_status.status.image_size <= STREAM_MAX_IMAGE_BYTES;
}

static uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t length)
{
    size_t index;
    unsigned int bit;
    for (index = 0U; index < length; index++) {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) :
                                   (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t get_be16(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

static uint32_t get_be32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | data[3];
}

static void put_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static void put_be32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}

static void set_state(f280_fw_stream_state_t state, uint8_t status)
{
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    s_stream.public.state = state;
    s_stream.public.last_status = status;
    retain_status_locked();
    xSemaphoreGive(s_stream.lock);
}

static bool send_and_wait(uint8_t packet, const uint8_t *payload,
                          uint8_t length, uint32_t timeout_ms,
                          uint32_t *next_offset)
{
    uint8_t target;
    uint8_t response[F280_UPD_RESPONSE_FRAME_BYTES];

    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    target = s_stream.public.target_id;
    xSemaphoreGive(s_stream.lock);
    while (xSemaphoreTake(s_stream.response_sem, 0) == pdTRUE) {}
    comm_can_transmit_eid((uint32_t)target | ((uint32_t)packet << 8),
                          payload, length);
    if (xSemaphoreTake(s_stream.response_sem,
                       pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return false;
    }
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    memcpy(response, s_stream.response, sizeof(response));
    xSemaphoreGive(s_stream.lock);
    if (response[0] != packet || response[1] != F280_UPD_STATUS_OK) {
        set_state(s_stream.public.state, response[1]);
        return false;
    }
    if (next_offset != NULL) {
        *next_offset = get_be32(&response[2]);
    }
    return true;
}

static bool enter_bootloader(void)
{
    uint8_t payload[8] = {0};
    uint32_t elapsed = 0U;

    put_be32(payload, F280_UPD_ENTER_MAGIC);
    put_be16(&payload[4], F280_UPD_PROTOCOL_VERSION);
    (void)send_and_wait(F280_UPD_PACKET_ENTER, payload, sizeof(payload),
                        STREAM_ACK_TIMEOUT_MS, NULL);
    vTaskDelay(pdMS_TO_TICKS(100U));
    while (elapsed < STREAM_BOOT_TIMEOUT_MS) {
        if (send_and_wait(F280_UPD_PACKET_HELLO, payload, sizeof(payload),
                          250U, NULL)) {
            return true;
        }
        elapsed += 250U;
    }
    return false;
}

static bool send_block(uint32_t offset, const uint8_t *data, uint16_t length)
{
    uint8_t frame[8];
    uint16_t position = 0U;
    uint8_t sequence = 0U;
    uint32_t acknowledged_offset;
    unsigned int attempt;

    for (attempt = 0U; attempt < STREAM_BLOCK_RETRIES; attempt++) {
        put_be32(frame, offset);
        put_be16(&frame[4], length);
        put_be16(&frame[6], crc16_update(0U, data, length));
        comm_can_transmit_eid((uint32_t)s_stream.public.target_id |
                              ((uint32_t)F280_UPD_PACKET_BLOCK_BEGIN << 8),
                              frame, 8U);
        position = 0U;
        sequence = 0U;
        while (position < length) {
            uint8_t count = (uint8_t)(length - position);
            if (count > F280_UPD_DATA_OCTETS_PER_FRAME) {
                count = F280_UPD_DATA_OCTETS_PER_FRAME;
            }

            frame[0] = sequence++;
            memcpy(&frame[1], &data[position], count);
            comm_can_transmit_eid((uint32_t)s_stream.public.target_id |
                                  ((uint32_t)F280_UPD_PACKET_BLOCK_DATA << 8),
                                  frame, (uint8_t)(count + 1U));
            position += count;
        }
        put_be32(frame, offset);
        if (send_and_wait(F280_UPD_PACKET_BLOCK_END, frame, 4U,
                          STREAM_ACK_TIMEOUT_MS, &acknowledged_offset) &&
            acknowledged_offset >= offset + length) {
            return true;
        }
        xSemaphoreTake(s_stream.lock, portMAX_DELAY);
        s_stream.public.retry_count++;
        retain_status_locked();
        xSemaphoreGive(s_stream.lock);
    }
    return false;
}
static bool advance_erased(uint32_t target_offset)
{
    uint8_t erased[F280_UPD_BLOCK_BYTES];

    memset(erased, 0xFF, sizeof(erased));
    while (s_stream.public.next_offset < target_offset) {
        uint32_t remaining = target_offset - s_stream.public.next_offset;
        uint16_t count = remaining > sizeof(erased) ? sizeof(erased) :
                                                        (uint16_t)remaining;
        if (!send_block(s_stream.public.next_offset, erased, count)) {
            return false;
        }
        s_stream.running_crc = crc16_update(s_stream.running_crc,
                                            erased, count);
        xSemaphoreTake(s_stream.lock, portMAX_DELAY);
        s_stream.public.next_offset += count;
        retain_status_locked();
        xSemaphoreGive(s_stream.lock);
    }
    return true;
}


bool f280_fw_stream_init(void)
{
    if (s_stream.initialized) return true;
    memset(&s_stream, 0, sizeof(s_stream));
    s_stream.lock = xSemaphoreCreateMutex();
    s_stream.response_sem = xSemaphoreCreateBinary();
    if (s_stream.lock == NULL || s_stream.response_sem == NULL) return false;
    s_stream.initialized = true;
    return true;
}

bool f280_fw_stream_begin(uint8_t target_id, uint32_t image_size)
{
    uint8_t payload[8] = {0};

    uint32_t next_offset;

    if (!s_stream.initialized || image_size == 0U ||
        image_size > STREAM_MAX_IMAGE_BYTES || (image_size & 1U) != 0U) {
        return false;
    }
    memset(s_stream.prefix, 0, sizeof(s_stream.prefix));
    s_stream.prefix_mask = 0U;
    s_stream.running_crc = 0U;
    s_stream.expected_crc = 0U;
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    memset(&s_stream.public, 0, sizeof(s_stream.public));
    s_stream.public.target_id = target_id;
    s_stream.public.image_size = image_size;
    s_stream.public.state = F280_FW_STREAM_ENTERING;
    retain_status_locked();
    xSemaphoreGive(s_stream.lock);
    if (!enter_bootloader()) goto fail;

    set_state(F280_FW_STREAM_ERASING, F280_UPD_STATUS_OK);
    put_be32(payload, image_size);
    put_be32(&payload[4], 0U);
    if (!send_and_wait(F280_UPD_PACKET_BEGIN, payload, sizeof(payload),
                       STREAM_ERASE_TIMEOUT_MS, &next_offset) ||
        next_offset != 0U) {
        goto fail;
    }
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    s_stream.public.active = true;
    s_stream.public.state = F280_FW_STREAM_PROGRAMMING;
    retain_status_locked();
    xSemaphoreGive(s_stream.lock);
    return true;

fail:
    f280_fw_stream_abort();
    return false;
}

bool f280_fw_stream_write(uint32_t protocol_offset,
                          const uint8_t *data, size_t length)
{
    uint32_t position = protocol_offset;
    size_t consumed = 0U;

    if (!s_stream.initialized || data == NULL || length == 0U ||
        !s_stream.public.active ||
        protocol_offset > s_stream.public.image_size + STREAM_PREFIX_BYTES ||
        length > s_stream.public.image_size + STREAM_PREFIX_BYTES -
                 protocol_offset) {
        return false;
    }
    while (consumed < length && position < STREAM_PREFIX_BYTES) {
        uint8_t mask = (uint8_t)(1U << position);
        if ((s_stream.prefix_mask & mask) != 0U &&
            s_stream.prefix[position] != data[consumed]) {
            goto fail;
        }
        s_stream.prefix[position] = data[consumed++];
        s_stream.prefix_mask |= mask;
        position++;
    }
    if (s_stream.prefix_mask == 0x3FU) {
        if (get_be32(s_stream.prefix) != s_stream.public.image_size) goto fail;
        s_stream.expected_crc = get_be16(&s_stream.prefix[4]);
    }
    while (consumed < length) {
        uint32_t image_offset = position - STREAM_PREFIX_BYTES;
        uint32_t next = s_stream.public.next_offset;
        uint16_t count;

        if (image_offset > next) {
            if (!advance_erased(image_offset)) goto fail;
            next = s_stream.public.next_offset;
        }
        count = (uint16_t)(length - consumed);
        if (count > F280_UPD_BLOCK_BYTES) count = F280_UPD_BLOCK_BYTES;
        if (image_offset < next && image_offset + count > next) {
            count = (uint16_t)(next - image_offset);
        }
        if (!send_block(image_offset, &data[consumed], count)) goto fail;
        if (image_offset == next) {
            s_stream.running_crc = crc16_update(
                s_stream.running_crc, &data[consumed], count);
            xSemaphoreTake(s_stream.lock, portMAX_DELAY);
            s_stream.public.next_offset += count;
            retain_status_locked();
            xSemaphoreGive(s_stream.lock);
        }
        consumed += count;
        position += count;
    }
    return true;

fail:
    f280_fw_stream_abort();
    return false;
}

bool f280_fw_stream_finish(void)
{
    uint8_t payload[8] = {0};
    uint32_t next_offset;
    bool ok;

    if (!s_stream.initialized || !s_stream.public.active ||
        s_stream.prefix_mask != 0x3FU) {
        f280_fw_stream_abort();
        return false;
    }
    if (!advance_erased(s_stream.public.image_size) ||
        s_stream.running_crc != s_stream.expected_crc) {
        f280_fw_stream_abort();
        return false;
    }
    set_state(F280_FW_STREAM_VERIFYING, F280_UPD_STATUS_OK);
    put_be32(payload, s_stream.public.image_size);
    put_be32(&payload[4], 0U);
    ok = send_and_wait(F280_UPD_PACKET_FINISH, payload, sizeof(payload),
                       STREAM_ERASE_TIMEOUT_MS, &next_offset);
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    s_stream.public.active = false;
    s_stream.public.state = ok ? F280_FW_STREAM_COMPLETE :
                                 F280_FW_STREAM_FAILED;
    retain_status_locked();
    xSemaphoreGive(s_stream.lock);
    return ok;
}

void f280_fw_stream_abort(void)
{
    uint8_t payload[8] = {0};
    if (!s_stream.initialized) return;
    if (s_stream.public.target_id != 0U) {
        comm_can_transmit_eid((uint32_t)s_stream.public.target_id |
                              ((uint32_t)F280_UPD_PACKET_ABORT << 8),
                              payload, 0U);
    }
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    s_stream.public.active = false;
    s_stream.public.state = F280_FW_STREAM_FAILED;
    retain_status_locked();
    xSemaphoreGive(s_stream.lock);
}

bool f280_fw_stream_handle_can(uint32_t eid, const uint8_t *data,
                               uint8_t length)
{
    uint8_t id = (uint8_t)eid;
    uint8_t packet = (uint8_t)(eid >> 8);
    if (!s_stream.initialized || packet != F280_UPD_PACKET_RESPONSE ||
        id != s_stream.public.target_id || data == NULL ||
        length != F280_UPD_RESPONSE_FRAME_BYTES) {
        return false;
    }
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    memcpy(s_stream.response, data, F280_UPD_RESPONSE_FRAME_BYTES);
    xSemaphoreGive(s_stream.lock);
    xSemaphoreGive(s_stream.response_sem);
    return true;
}

void f280_fw_stream_get_status(f280_fw_stream_status_t *status)
{
    if (status == NULL || !s_stream.initialized) return;
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    *status = s_stream.public;
    xSemaphoreGive(s_stream.lock);
}

bool f280_fw_stream_get_retained_status(f280_fw_stream_status_t *status)
{
    bool valid;

    if (status == NULL || !s_stream.initialized) return false;
    xSemaphoreTake(s_stream.lock, portMAX_DELAY);
    valid = retained_status_valid();
    if (valid) {
        *status = s_retained_status.status;
    }
    xSemaphoreGive(s_stream.lock);
    return valid;
}
