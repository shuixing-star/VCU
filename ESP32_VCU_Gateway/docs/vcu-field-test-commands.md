# VCU Field Test Commands

## Flash

Run from `E:\Work\2026\VCU\ESP32_VCU_Gateway`:

```powershell
idf.py -B build_k2 -p COM18 flash
```


## Upgrade In VESC Tool

Build output firmware:

```text
E:\Work\2026\VCU\ESP32_VCU_Gateway\build_k2\ESP32_VCU_Gateway.bin
```

Typical Tool upgrade flow:

1. Connect the board in VESC Tool.
2. Open the firmware update page.
3. Choose custom firmware / custom file.
4. Select `build_k2\ESP32_VCU_Gateway.bin`.
5. Upload / write firmware.
6. Reconnect after reboot.
7. Run `vcu_drive_status` and check the new fields.

Expected new field:

```text
accel       : 2550 rpm/s
can ramp    : 255
```

## Basic Checks

```text
mx_can_status
vcu_rc_status
vcu_drive_status
```

`vcu_rc_status` should show:

```text
throttle    : ch3 sign=+1
steering    : ch1 sign=-1
arm         : ch7 raw=... >= 1500 req=...
reset       : ch8 raw=... <= 500 req=...
```

Vehicle targets with absolute value below 200 rpm are commanded as zero. Targets at or above 200 rpm are not boosted to a minimum speed.
Controller feedback with absolute vehicle-normalized value below 200 rpm is also reported and logged as zero. Use `mx_can_raw` when the unfiltered wire value is needed.

## CAN ID Setup

The firmware defaults to the two controller IDs used by this vehicle:

```text
left : status 1801E002, command 1801E601, direction -1
right: status 1801E001, command 1801E600, direction +1
```

Normally no `mx_can_ids` command is needed after startup. Use the command below only to override the defaults for diagnostics.
The VCU uses vehicle-normalized signs: forward is positive on both physical sides. The left controller command and raw RPM are inverted at the CAN adapter; the right controller sign is unchanged. `mx_can_raw` remains the source of unmodified wire bytes.
The `mx_can_ids` argument order is physical left first, then physical right. Direction signs remain fixed when IDs are overridden.

Single controller:

```text
mx_can_ids 1801E002 1801E601
```

Two controllers:

```text
mx_can_ids <left_status> <left_command> <right_status> <right_command>
```

Check controller status:

```text
mx_can_status
```

## Drive Runtime

Start RC drive:

```text
vcu_drive_start
```

Start with custom CAN command period, for example 10 ms:

```text
vcu_drive_start 10
```

Start with custom period and acceleration, for example 5 ms and max 2550 rpm/s:

```text
vcu_drive_start 5 2550
```

Stop drive:

```text
vcu_drive_stop
```

Check runtime:

```text
vcu_drive_status
```

Current defaults:

```text
max target  : 4800 rpm
min moving  : 400 rpm
period      : 5 ms
accel       : 2550 rpm/s
```

## RC Switches

```text
CH7 high       = ARM
CH7 mid/low    = KILL
CH8 down/low   = RESET, edge-triggered once
```

CH8 reset condition:

```text
ch8 raw <= 500
```

After CH8 reset, drive is stopped. Start again with:

```text
vcu_drive_start
```

## Logging

Start log at 20 Hz:

```text
mx_log_start 20
```

Check log:

```text
mx_log_status
```

The logger buffers samples and flushes them to the SD card about once per second. In the status output, `file errors` must remain zero. `max flush ms` and `slow flushes` show SD-card write stalls that can affect other tasks.

Stop log:

```text
mx_log_stop
```

Log files:

```text
mx_log/vcu_001.csv
mx_log/vcu_002.csv
```

## Recommended Field Sequence

```text
mx_can_status
vcu_rc_status
mx_log_start 20
vcu_drive_start
vcu_drive_status
```

Then:

1. Set CH7 high.
2. Keep sticks neutral for a moment.
3. Move CH1 slowly for forward/reverse.
4. Move CH3 slowly for steering.
5. Pull CH8 down once to test reset.
6. Run `vcu_drive_status` and confirm `reset tx` increased.

## Dual-motor speed synchronization

Synchronization is enabled by default. It is a low-bandwidth, pure P correction
outside the motor controllers' internal speed loops. It preserves the theoretical
left/right target difference produced by throttle and steering.

Default parameters:

```text
Kp = 0.20
error deadzone = 40 rpm
correction limits = 150 / 200 / 250 rpm
limit thresholds = 4000 / 4400 rpm
weak-field fallback threshold = 4000 rpm
target slew rate = 2550 rpm/s
large error = 250 rpm for 800 ms
longitudinal derate = 100 rpm per trigger
```

Runtime commands:

```text
vcu_sync_status
vcu_sync_enable 0|1
vcu_sync_kp 0.20
vcu_sync_deadzone 40
vcu_sync_limits 150 200 250
vcu_sync_thresholds 4000 4400 4000
vcu_sync_slew 2550
vcu_sync_derate 250 800 100 200
```

The synchronization error is calculated from the difference relative to the
theoretical turn command. After the deadzone, the P correction only reduces the
magnitude of the faster motor target; it never increases the slower motor target.
Reverse travel uses the same magnitude rule. Synchronization is deliberately
inactive during an in-place turn or whenever the two theoretical targets have
opposite signs.

If no controller weak-field status bit is available, average absolute motor speed
at or above 4000 rpm is treated as weak field. A speed difference at or above
250 rpm for 800 ms reduces the longitudinal target by 100 rpm, then starts a new
800 ms timing interval. Steering difference is retained during this derating.

Both final targets pass through the VCU 2550 rpm/s slew limiter. Existing emergency
stop, neutral, CAN timeout, and controller fault paths bypass this limiter and send
zero immediately.

CSV logging uses the stable basic 11-column format: time, RC input, final left/right
targets, actual left/right RPM, faults, and voltages. Synchronization diagnostics are
kept out of the high-rate SD-card path; run `vcu_sync_status` to inspect live values.