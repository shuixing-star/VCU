# Changelog

## [1.2.3] - 2026-07-27

### Added

- MX RS485 protocol support and the CAN485 bridge.
- ESP32-to-STM32 CAN command and telemetry gateway.
- ESP32 UBlox GPS integration and STM32 telemetry logging.

### Changed

- Moved CRSF reception, driver-intent interpretation, tracked-vehicle mixing,
  safety state, speed synchronization, and motor CAN control from ESP32 to
  STM32F407.
- Enabled the CAN485 bridge and STM32 gateway concurrently on the ESP32 target.
- Renamed the ESP32 project from `VESC_K2_TEST` to `ESP32_VCU_Gateway`.

### Safety

- ESP32 direct transmission of the two traction motor command IDs is blocked.
- STM32 uses timestamped CRSF/CAN input, paired motor command transmission,
  startup zero output, feedback timeouts, scheduling-overrun detection, and
  latched reset sequencing.
