# MX QML Panel

`qml/MxCan485Panel.qml` is a read-only VESC Tool panel for the MX CAN485 data
stream.

## Firmware side

Use these terminal commands after flashing the current firmware:

```text
mx_cfg 0x101 0x102 sid 1
mx_start 200
mx_stream_start 10
```

The panel listens to `COMM_CUSTOM_APP_DATA` packets sent by `mx_stream_start`.
It decodes packet type `1`, version `1`, as documented in
`docs/mx-app-data-packet.md`.

## VESC Tool side

Open the QML UI page in VESC Tool and load:

```text
qml/MxCan485Panel.qml
```

The panel displays:

- MX online/stream state
- fault code
- temperature
- voltage
- motor 1 and motor 2 current
- motor 1 and motor 2 RPM
- stream, TX, RX, timeout, bad-frame, and overflow counters

The panel is display-only. It does not send polling, logging, stream or motor
control commands.

Use terminal commands to control acquisition:

```text
vcu_start [poll_ms] [stream_hz] [log_hz]
vcu_stop
vcu_status
```
