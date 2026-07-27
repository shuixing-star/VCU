#ifndef MX_LOG_H_
#define MX_LOG_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool enabled;
    bool task_running;
    uint32_t period_ms;
    uint32_t samples_written;
    uint32_t duplicate_skips;
    uint32_t file_errors;
    uint32_t flush_count;
    uint32_t slow_flush_count;
    uint32_t max_sample_write_ms;
    uint32_t max_flush_ms;
    int last_error_errno;
    const char *last_error_operation;
    const char *active_path;
} mx_log_public_state;

void mx_log_init(void);
bool mx_log_start(uint32_t hz);
void mx_log_stop(void);
bool mx_log_get_state(mx_log_public_state *state);

#endif /* MX_LOG_H_ */
