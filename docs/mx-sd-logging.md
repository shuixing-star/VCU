# MX SD Logging

This slice logs parsed MX controller samples to CSV without using the VESC Tool
generic logging protocol. Each MX row also includes the latest GPS snapshot from
the onboard u-blox/NMEA parser.

## Terminal Commands

```text
mx_log_start [hz]
mx_log_stop
mx_log_status
vcu_start [poll_ms] [stream_hz] [log_hz]
vcu_stop
vcu_status
```

Default rate is 5 Hz. The accepted range is 1 to 50 Hz. The logger only writes
a row when `rx_ok` changes, so it skips duplicate samples if the log task runs
faster than MX polling.

Files are written under:

```text
/sdcard/mx_log/
```

with incrementing names:

```text
mx_001.csv
mx_002.csv
...
```

## CSV Fields

```text
runtime_ms
utc_valid
utc_time
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
`$([Environment]::NewLine)
## First Hardware Test

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

Let it run for 30 seconds, then:

```text
vcu_stop
vcu_status
```

Power down before removing the card, then check the newest CSV in
`mx_log`.



