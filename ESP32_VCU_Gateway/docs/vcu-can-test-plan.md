# VCU CAN Test Firmware Plan

## System map
- `main/vcu_crsf.*`: owns CRSF CRC, channel decoding, and link age.
- `main/mx_can_protocol.*`: owns MX CAN IDs, flags, byte order, and ranges.
- `main/vcu_mx_can_test.*`: owns MX controller status capture and manual disabled-zero CAN diagnostics.
- Existing `main/comm_can.*`: remains the ESP-IDF TWAI transport.

## Safety boundary
Current runtime does not automatically transmit drive commands. Manual diagnostics may send one disabled-zero frame with both axes disabled. Future runtime must default to zero target with both axes disabled. Enable requires a recent valid CRSF frame, explicit arm channel, startup-neutral controls, and healthy 500 kbit/s CAN. CRSF timeout, disarm, invalid input, task overrun, or CAN fault must disable both axes. The drive's 1.5 second timeout is only a final backstop.

## Thin slices
- [x] Portable CRSF parser and MX CAN codec with host tests.
- [x] UART adapter and terminal-only channel diagnostics; no CAN transmission.
- [x] CAN status decoding and manual disabled-zero frame command.
- [x] RC mapping preview with deadband and differential mixing; no runtime CAN TX.
- [ ] Disarmed/arming/armed/fault state machine; wheels-up test.
- [ ] Low-speed vehicle test and calibrated limits.

## Field checks
- CRSF receiver validated on hardware: link OK, valid frames incrementing, CRC and UART errors at zero.
- CAN check passed on hardware: `mx_can_status` receives 0x1801E001 and `mx_can_send_zero` sends one disabled-zero frame.
- Next RC check: run `vcu_drive_preview` while moving CH1 and CH3; confirm sign and axis mapping before any enable logic.

## Decisions required before runtime wiring
- ESP32 target; CRSF RX GPIO/baud; TWAI GPIOs and transceiver.
- CRSF mapping: CH1 throttle, CH3 steering, CH7 arm/kill.
- Physical axis mapping: axis1 left, axis2 right.
- Mode, maximum target, ramp, deadband, mix, timeout, and command period.
- Fault latching/recovery policy.

## Logging ownership

The generic VESC logging and SD mount infrastructure remains enabled. The old `mx_log` application is not started in CRSF test mode because its rows are tied to the retired CAN485 and GPS data sources. A later slice will keep the same SD storage layer while replacing the row schema with CRSF input, requested CAN command, controller status, safety state, and fault-reason fields.



