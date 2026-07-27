# VCU Phase 1 MX Data Acquisition Plan

## Goal

Build the first VCU data-acquisition slice on top of VESC Express firmware.
The system currently focuses on collecting data, not controlling the MX controller.

Main data sources:

- MX controller through CAN-to-RS485 gateway.
- u-blox M8N GPS through UART/NMEA.
- SD card CSV logging.
- VESC Tool/QML display through custom app data.

STM32F407-related work is explicitly out of scope for this phase.

## Architecture

Runtime ownership:

- `mx_rs485_protocol.h/.c`: deterministic MX Modbus RTU request/response encoding and parsing.
- `mx_can485.h/.c`: CAN-to-RS485 gateway adapter, polling task, stream task, and terminal diagnostics.
- `vcu_gps.h/.c`: GPS snapshot adapter over existing `ublox.c` and `nmea.c`; no raw GPS parsing here.
- `mx_log.h/.c`: main VCU CSV logging to SD. Rows contain runtime time, optional UTC time, MX data, and latest GPS snapshot.
- `vcu_control.h/.c`: operator commands that start/stop the acquisition bundle and print combined status.
- `qml/MxCan485Panel.qml`: VESC Tool display-only panel for streamed MX data.

## Confirmed MX Link

Current confirmed communication parameters:

```text
tx_id   = 0x101
rx_id   = 0x102
id type = SID
mx addr = 1
```

Useful commands:

```text
mx_cfg 0x101 0x102 sid 1
mx_poll 1
mx_start 200
mx_stop
mx_status
mx_parse [response bytes...]
mx_reset
```

Confirmed status has shown:

```text
last result : ok
online      : 1
rx ok       : incrementing
rx bad      : 0
rx timeout  : 0
```

## GPS

GPS uses existing VESC u-blox/NMEA code:

```text
ublox_init(false, 500, UART_NUM, UART_RX, UART_TX)
nmea_get_state()
```

Current UART mapping in `hw_xp_t.h`:

```text
UART_NUM : 0
UART_TX  : GPIO21
UART_RX  : GPIO20
```

Useful commands:

```text
gps_status
nmea_info
ublox_reinit
```

GPS state policy:

- `ublox_ready`: u-blox init/handshake succeeded.
- `has_gga`: at least one GGA message decoded.
- `has_rmc`: at least one RMC message decoded.
- `fix_valid`: GGA reports a non-zero fix type.
- `tod_valid`: RMC time-of-day is valid.
- `date_valid`: RMC date is valid.
- `time_valid`: date and time-of-day are both valid.

GPS is not logged as a separate standalone CSV. It is appended to the main VCU/MX CSV row as the latest GPS snapshot.

## Main CSV Log

Log directory:

```text
/sdcard/mx_log/
```

File names:

```text
mx_001.csv
mx_002.csv
...
```

Commands:

```text
mx_log_start [hz]
mx_log_stop
mx_log_status
vcu_start [poll_ms] [stream_hz] [log_hz]
vcu_stop
vcu_status
```

Current CSV fields:

```text
runtime_ms
utc_valid`nutc_time
mx_addr
online
has_sample
fault
temp_c
voltage_v
m1_current_a
m2_current_a
m1_rpm
m2_rpm
tx_count
rx_ok
rx_timeout
rx_bad
rx_overflow
gps_ready
gps_has_gga
gps_has_rmc
gps_fix_valid
gps_fix_type
gps_sats
gps_lat
gps_lon
gps_height_m
gps_speed_mps
```

Timestamp policy:

- `runtime_ms` is always present and comes from firmware runtime.
- UTC fields are filled only when `gps.time_valid` is true.
- If UTC is unavailable, `utc_valid = 0` and all `utc_*` fields are `-1`.
- GPS position/speed fields are latest parser values; if no fix is available they remain parser defaults and should be interpreted using `gps_fix_valid`.

## SD Card

Current SD SPI mapping:

```text
MOSI = GPIO6
MISO = GPIO5
SCK  = GPIO4
CS   = GPIO7
```

Useful commands:

```text
sd_status
sd_mount [freq_khz]
sd_unmount
sd_test
```

GPIO3 blue LED was disabled because the board schematic ties GPIO3 near SD card detect/pull-up circuitry.

## VCU Start/Stop

Default command:

```text
vcu_start 200 10 5
```

Meaning:

```text
poll_ms   = 200
stream_hz = 10
log_hz    = 5
```

Stop:

```text
vcu_stop
```

Status:

```text
vcu_status
```

## Hardware Test Sequence

```text
sd_status
sd_test
mx_cfg 0x101 0x102 sid 1
mx_start 200
mx_status
gps_status
vcu_start 200 10 5
vcu_status
```

Let it run, then:

```text
vcu_stop
vcu_status
```

Check the newest CSV under `/sdcard/mx_log/`.

## Next Steps

1. Compile and flash the current firmware.
2. Run the hardware test sequence above.
3. Inspect one CSV file for column alignment and values.
4. If CSV is correct, keep the main log schema stable before adding new sensors.
5. Later: add GPS fields to QML only if display needs them; logging already has the data.


