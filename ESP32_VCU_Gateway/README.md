# ESP32 VCU Gateway

ESP32 companion firmware for the vehicle control unit.

## Responsibilities

- Host commands and terminal access
- CSV logging
- UBlox GPS on UART0 (`GPIO21` TX, `GPIO20` RX)
- MX RS485 protocol over the CAN485 bridge
- CAN command and telemetry gateway to the STM32F407 controller

The STM32F407 owns CRSF input interpretation, safety state, tracked-vehicle
mixing, target slew, speed synchronization, and traction motor commands.
The ESP32 gateway rejects direct transmission of the two traction motor command
CAN IDs.

## Current target

- MCU: ESP32-C3
- Hardware definition: `hw_xp_t`
- STM32 gateway CAN IDs: standard `0x680` through `0x686`
- CAN485 bridge default IDs: standard `0x101` request and `0x102` response

## Build

```powershell
idf.py -B build_k2 build
```

After the directory rename, ESP-IDF generates
`build_k2/ESP32_VCU_Gateway.bin`.
