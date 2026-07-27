# MX App Data Packet

This packet is sent by the firmware with `commands_send_app_data`, so it reaches
VESC Tool as `COMM_CUSTOM_APP_DATA`.

## Terminal commands

- `mx_stream_once`: send one status packet.
- `mx_stream_start [hz]`: stream status packets at 1 to 50 Hz. Default is 10 Hz.
- `mx_stream_stop`: stop status streaming.

The stream is independent from polling. A normal test sequence is:

```text
mx_cfg 0x101 0x102 sid 1
mx_start 200
mx_stream_start 10
mx_status
```

## Status packet layout

All integer fields use the existing VESC `buffer_append_*` big-endian encoding.
Floats use `buffer_append_float32_auto`.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | uint8 | packet type, `MX_CAN485_APP_PACKET_STATUS` = 1 |
| 1 | uint8 | packet version, `MX_CAN485_APP_PACKET_VERSION` = 1 |
| 2 | uint8 | online, 0 or 1 |
| 3 | uint8 | has sample, 0 or 1 |
| 4 | uint8 | MX address |
| 5 | uint16 | fault code |
| 7 | int16 | temperature, deg C |
| 9 | float32_auto | voltage, V |
| 13 | float32_auto | motor 1 current, A |
| 17 | float32_auto | motor 2 current, A |
| 21 | int32 | motor 1 speed, rpm |
| 25 | int32 | motor 2 speed, rpm |
| 29 | uint32 | poll TX count |
| 33 | uint32 | valid RX count |
| 37 | uint32 | timeout count |
| 41 | uint32 | bad frame count |
| 45 | uint32 | RX overflow count |

Total payload length is 49 bytes, before the VESC protocol's
`COMM_CUSTOM_APP_DATA` command byte.
