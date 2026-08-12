#ifndef F280_FW_BRIDGE_H
#define F280_FW_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*f280_fw_reply_fn)(unsigned char *data, unsigned int length);

bool f280_fw_bridge_init(void);

/* Return true when the forwarded packet belongs to the F280 updater. */
bool f280_fw_bridge_handle(uint8_t target_id,
                           const uint8_t *packet,
                           unsigned int length,
                           f280_fw_reply_fn reply);

#endif
