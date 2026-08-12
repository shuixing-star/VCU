#ifndef F280_FW_STREAM_H
#define F280_FW_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    F280_FW_STREAM_IDLE = 0,
    F280_FW_STREAM_ENTERING,
    F280_FW_STREAM_ERASING,
    F280_FW_STREAM_PROGRAMMING,
    F280_FW_STREAM_VERIFYING,
    F280_FW_STREAM_COMPLETE,
    F280_FW_STREAM_FAILED
} f280_fw_stream_state_t;

typedef struct {
    f280_fw_stream_state_t state;
    uint8_t target_id;
    uint8_t last_status;
    uint32_t image_size;
    uint32_t next_offset;
    uint32_t retry_count;
    bool active;
} f280_fw_stream_status_t;

bool f280_fw_stream_init(void);
bool f280_fw_stream_begin(uint8_t target_id, uint32_t image_size);
bool f280_fw_stream_write(uint32_t protocol_offset,
                          const uint8_t *data, size_t length);
bool f280_fw_stream_finish(void);
void f280_fw_stream_abort(void);
bool f280_fw_stream_handle_can(uint32_t eid, const uint8_t *data,
                               uint8_t length);
void f280_fw_stream_get_status(f280_fw_stream_status_t *status);
bool f280_fw_stream_get_retained_status(
        f280_fw_stream_status_t *status);

#endif
