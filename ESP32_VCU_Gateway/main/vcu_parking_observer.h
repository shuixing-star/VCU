#ifndef VCU_PARKING_OBSERVER_H
#define VCU_PARKING_OBSERVER_H
#include <stdbool.h>
#include <stdint.h>
/* Passive wire classification, NOT a K2 acceptance acknowledgement. */
enum { VCU_PARK_CMD_STOP, VCU_PARK_CMD_DRIVE, VCU_PARK_CMD_BRAKE,
       VCU_PARK_CMD_PARK, VCU_PARK_CMD_RESET, VCU_PARK_CMD_INVALID };
#define VCU_PARK_OBS_FRESH_MS 100U
typedef struct {
    bool command_seen, status_seen;
    uint8_t command[8], command_len, command_kind, status[8];
    uint32_t command_ms, status_ms, command_count, command_changes;
    uint32_t brake_count, park_count, invalid_count, bad_status_count;
    uint32_t last_brake_ms, last_park_ms, hold_entries;
} vcu_parking_observer_t;
/* frame_kind: 0 = GED command, 2 = GED status2; caller routes exact node IDs.
 * Caller owns its existing gateway lock. No transmission, allocation or I/O. */
void vcu_parking_observe(vcu_parking_observer_t *s, unsigned frame_kind,
                         const uint8_t *data, uint8_t len, uint32_t now_ms);
#endif
