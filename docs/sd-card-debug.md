# SD Card Debug

The firmware uses SDSPI, not 4-bit SDMMC.

Default `VESC Express T` pins:

| Signal | GPIO |
| --- | ---: |
| MOSI / DI / CMD | 6 |
| MISO / DO / DAT0 | 5 |
| SCK / CLK | 4 |
| CS / DAT3 | 7 |

## Terminal commands

```text
sd_status
sd_unmount
sd_mount
sd_mount 400
sd_test
```

Use `sd_mount 400` to retry at a slow 400 kHz clock. If that works but the
default mount fails, the wiring/card holder is marginal at the default speed.

`sd_status` prints:

- mounted state
- configured pins
- last mount stage
- SPI init result
- FAT/SDSPI mount result
- card name, sector size, and capacity when mounted

`sd_test` writes and reads `/sdcard/sd_debug.txt`.

## Useful first test sequence

```text
sd_status
sd_unmount
sd_mount 400
sd_status
sd_test
```

If `spi init` is OK but `mount` fails, focus on SD module wiring, pull-ups,
power, card format, and SPI speed. If `spi init` fails, focus on GPIO conflicts
or the selected hardware target.
