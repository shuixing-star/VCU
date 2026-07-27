# VCU GPS M8N Bring-Up

The firmware uses the existing `ublox.c` and `nmea.c` modules for u-blox M8N.
The VCU layer only reads their parsed state and exposes it in terminal status
and the main MX/VCU CSV log.

## Wiring

Current `hw_xp_t.h` UART settings:

```text
UART_NUM : 0
UART_TX  : GPIO21
UART_RX  : GPIO20
Baud     : 115200 after initialization
```

Connect:

```text
M8N TX -> ESP GPIO20
M8N RX -> ESP GPIO21
M8N GND -> ESP GND
M8N VCC -> module-rated supply
```

## Startup Flow

`main.c` calls:

```text
nmea_init()
ublox_init(false, 500, UART_NUM, UART_RX, UART_TX)
vcu_gps_init()
```

The u-blox init tries the configured baud rate, then common u-blox defaults,
sets an automotive dynamic model, configures a 500 ms measurement period, and
enables GGA/GSV/RMC NMEA output.

## Terminal Commands

```text
gps_status
nmea_info
ublox_reinit
```

Use `gps_status` first. Important fields:

```text
ublox ready : 1 means the init handshake succeeded
gga count   : increments when position messages are decoded
rmc count   : increments when date/time/speed messages are decoded
fix valid   : 1 means GGA reports a non-zero fix type
tod valid   : 1 means RMC time-of-day is valid
date valid  : 1 means RMC date is valid
time valid  : 1 means both date and time-of-day are valid
satellites  : satellites used in the fix
hdop        : horizontal dilution of precision
gga age     : should stay fresh while messages arrive
rmc age     : should stay fresh while messages arrive
```

If `ublox ready` is `0`, check power, UART crossing, and run:

```text
ublox_reinit
gps_status
```

If `ublox ready` is `1` but `fix valid` is `0`, the UART link is alive but the
receiver does not yet have a position fix. Move the antenna outdoors or wait
for satellite lock.

## Logging

GPS fields are written into the main MX/VCU CSV log. The log has two time
groups:

```text
runtime_ms
utc_valid
utc_time
```

If UTC is unavailable, `utc_valid` is `0` and `utc_time` is empty.

The GPS data columns start at:

```text
gps_ready,gps_has_gga,gps_has_rmc,gps_fix_valid,gps_fix_type,...
```

The logger writes only when a new MX sample arrives, so GPS values are sampled
at MX log row time rather than at every raw NMEA update.

