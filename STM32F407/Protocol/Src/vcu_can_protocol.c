#include "vcu_can_protocol.h"

bool vcu_can_protocol_validate_frame(const vcu_can_frame_t *frame)
{
    if (frame == 0 || frame->data_length > VCU_CAN_MAX_DATA_LENGTH) {
        return false;
    }

    if (frame->extended_id) {
        return frame->id <= 0x1FFFFFFFUL;
    }

    return frame->id <= 0x7FFU;
}

bool vcu_can_protocol_version_supported(uint8_t version)
{
    return version == VCU_CAN_PROTOCOL_VERSION;
}
