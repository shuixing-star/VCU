#ifndef F280_FW_PROTOCOL_H
#define F280_FW_PROTOCOL_H

/*
 * Private ESP32-to-F28069 update transport carried in 29-bit CAN frames.
 * The low eight EID bits are the target F280 node ID, matching VESC CAN.
 * Every multi-octet field is big-endian. No C struct is serialized.
 */
#define F280_UPD_PROTOCOL_VERSION        1U
#define F280_UPD_ENTER_MAGIC             0x46325550UL /* "F2UP" */
#define F280_UPD_BLOCK_BYTES             224U
#define F280_UPD_DATA_OCTETS_PER_FRAME   7U

#define F280_UPD_PACKET_ENTER            0x70U
#define F280_UPD_PACKET_HELLO            0x71U
#define F280_UPD_PACKET_BEGIN            0x72U
#define F280_UPD_PACKET_BLOCK_BEGIN      0x73U
#define F280_UPD_PACKET_ABORT            0x78U
#define F280_UPD_PACKET_BLOCK_DATA       0x74U
#define F280_UPD_PACKET_BLOCK_END        0x75U
#define F280_UPD_PACKET_FINISH           0x76U
#define F280_UPD_PACKET_RESPONSE         0x77U

#define F280_UPD_STATUS_OK               0U
#define F280_UPD_STATUS_BAD_COMMAND      1U
#define F280_UPD_STATUS_BAD_STATE        2U
#define F280_UPD_STATUS_BAD_SIZE         3U
#define F280_UPD_STATUS_BAD_OFFSET       4U
#define F280_UPD_STATUS_BAD_CRC          5U
#define F280_UPD_STATUS_FLASH_ERROR      6U
#define F280_UPD_STATUS_BAD_IMAGE        7U
#define F280_UPD_STATUS_UNAUTHORIZED     8U

/* BLOCK_BEGIN: offset:u32, length:u16, CRC16:u16. */
#define F280_UPD_BLOCK_BEGIN_FRAME_BYTES 8U
/* BLOCK_DATA: sequence:u8 followed by one to seven image octets. */
#define F280_UPD_BLOCK_DATA_MIN_BYTES    2U
#define F280_UPD_BLOCK_DATA_MAX_BYTES    8U
/* RESPONSE: request:u8, status:u8, next_offset:u32, detail:u16. */
#define F280_UPD_RESPONSE_FRAME_BYTES    8U

#endif
