#include "vcu_parking_observer.h"
#include <string.h>
static uint16_t read_le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static void increment(uint32_t *value) { if (*value != UINT32_MAX) (*value)++; }
static unsigned classify(const uint8_t *d, uint8_t len) {
    if (len != 8 || (d[4] & 0xF0U)) return VCU_PARK_CMD_INVALID;
    unsigned c=d[4];
    if ((c & 8U) && (!(c & 1U) || (c & 6U) ||
        read_le16(d)!=32000 || read_le16(d+2)!=32000 || read_le16(d+5)))
        return VCU_PARK_CMD_INVALID;
    if (c & 4U) return VCU_PARK_CMD_RESET;
    if (!(c & 1U)) return VCU_PARK_CMD_STOP;
    if (c & 8U) return VCU_PARK_CMD_PARK;
    return read_le16(d+5) ? VCU_PARK_CMD_BRAKE : VCU_PARK_CMD_DRIVE;
}
void vcu_parking_observe(vcu_parking_observer_t *s, unsigned kind,
                         const uint8_t *d, uint8_t len, uint32_t now) {
    if (!s || !d || len>8) return;
    if (kind==0) {
        if (!s->command_seen || len!=s->command_len ||
            memcmp(s->command,d,len<7?len:7)) increment(&s->command_changes);
        s->command_kind=(uint8_t)classify(d,len);
        s->command_seen=true; s->command_len=len; s->command_ms=now;
        memset(s->command,0,sizeof(s->command));memcpy(s->command,d,len);
        increment(&s->command_count);
        if (s->command_kind==VCU_PARK_CMD_BRAKE) {
            increment(&s->brake_count);s->last_brake_ms=now;
        } else if (s->command_kind==VCU_PARK_CMD_PARK) {
            increment(&s->park_count);s->last_park_ms=now;
        } else if (s->command_kind==VCU_PARK_CMD_INVALID) increment(&s->invalid_count);
    } else if (kind==2) {
        if (len!=8) {increment(&s->bad_status_count);return;}
        if ((d[2]&0x19U)==0x19U && (!s->status_seen || (s->status[2]&0x19U)!=0x19U))
            increment(&s->hold_entries);
        s->status_seen=true;s->status_ms=now;memcpy(s->status,d,8);
    }
}
