# Changelog

## [Source snapshot] - 2026-10-17

- Import the current field-test VCU source from `HASO_10KW/VCU`, including
  STM32F407 control/CAN recovery and the ESP32 diagnostic logging chain.
- Include parking/brake observations, K2 speed diagnostics, automatic fault
  records, current-trace companion CSVs, and the flight recorder and tools.
- Preserve the existing RENG Tool recording/configuration interface. This
  snapshot does not import the separate VCU checkout's LogUI experiment.
- Exclude generated firmware/build output; this is a source upload, not a
  newly built or hardware-validated firmware release.
- Offline validation passed: all 8 STM32 host suites, flight-recorder/export
  checks, parking CSV chain, 15 automatic fault-record scenarios, and all 8
  firmware-stream contract checks.
- Known test limitation: `test_vcu_can_errors_terminal.py` and
  `test_vcu_tx_probe.py` fail to compile their extracted host harnesses because
  they lack the newer parking-observer type/dependencies. These scripts are
  preserved as part of the source snapshot; a complete firmware build was
  not performed for this upload.

## [Unreleased] - 2026-09-28

- Adapt mx_log CSV to the existing Log Analysis reader on the user-selected HASO_10KW/VCU ESP32 baseline.
- Preserve existing CAN diagnostics, configuration layout, and flight recorder; no LogUI package required.
- Verify the AutoRecovery 5000 rpm / CH6 / K2 239/240 telemetry with the actual ESP receive and CSV code offline.
- STM32 baseline remains unchanged; hardware scan/SD recording not verified.
- See docs/log-analysis-recording.md for the exact baseline and usage.

## [vcu-log-1.0.0] - 2026-09-21

- Added an independent STM32 bare-metal diagnostic build and ESP32-C3 flight
  recorder with 10 Hz snapshots, per-pair CAN outcomes, and a 5 s pre / 3 s
  post-trigger frozen capture. The new CAN stream is bounded at 600 frames/s.
- Separated mailbox admission, observed TXOK and feedback receipt. Existing
  CAN diagnostic delivery is nonblocking in the dedicated logging build.
- Added CSV export through the existing SD/file transport, raw fault/clear
  frames, clock provenance, missing-value handling and drop/usage counters.
- Added local-only capture inspection, simulated examples and offline tests.
- Confirmed left/right addresses 239/240; corrected the ESP32 raw monitor table.
- Preserved historical firmware and existing edits. No motor-control algorithm,
  K2/RENG controller firmware, protection threshold or running parameter change.
- Offline builds/tests only; no hardware connection, flash or motor operation.
- See `docs/vcu-flight-log.md` for limits, resource calculations and operation.

## [Unreleased] - 2026-09-16

### Controller-owned speed ramp

- Removed the STM32 VCU target slew limiter. Synchronized targets pass through
  online gates and magnitude limits directly to the motor CAN encoder.
- Legacy RUN acceleration remains wire/API compatible but is ignored; the
  runtime field reports zero. STOP, link-loss and fault gates are preserved.
- Added CAN-level step/reversal/zero tests and removed the unused slew source
  from the Keil target.

### Diagnostic follow-up

- Added guarded CRSF ring access with first-invalid-index capture and a release
  barrier before the foreground publishes its tail. Added corrupt-index tests.
- Added an exception-entry HardFault snapshot on a dedicated stack. The current
  diagnostic build disables Cortex-M4 store buffering to aid fault localization;
  this changes timing and does not establish the original root cause.


### Changed

- STM32 VCU now runs the existing application in a bare-metal foreground loop
  driven by the TIM7 1 ms tick. Removed FreeRTOS sources/includes from the
  active Keil target and disabled FreeRTOS in CubeMX configuration.
- Preserved CAN/CRSF reception, control timing, safety gates and telemetry.
  CAN probe task-switch bits remain zero without changing the wire layout.
- Bare-metal outputs use MDK-ARM/BareMetal/STM32F407_BareMetal.*; historical
  RTOS source and outputs remain available. This migration does not establish
  the root cause of the previously observed memory corruption.

## [Unreleased] - 2026-09-15

### Fixed

- STM32 CAN error events are retained until hardware TXOK confirmation.
  Previously admission to a mailbox cleared the pending event even if a later
  timeout aborted it. Added a real-port/link/reporter regression proving the
  original event is transmitted after a failed attempt and bus recovery.

- STM32 motor TX now reserves two mailboxes and waits within a shared 2 ms
  budget for capacity and TXOK completion. Pending requests are aborted on
  timeout instead of retrying stale commands indefinitely. Persistent failures
  still use the existing zero-output and fault-latch path.
- Telemetry keeps at most one pending mailbox, expires after 2 ms when polled,
  and skips congestion without counting it as a new hardware error. Abort
  writes preserve other mailboxes' completion flags; mailbox reuse updates
  ownership so a stale telemetry timeout cannot cancel a motor command.
- Missing ACK for the startup zero no longer traps the control task in
  Error_Handler; initialization failures still do, and runtime safety gates
  remain active.

- STM32 error telemetry now has a reserved 5 ms slot in a seven-slot cycle.
  Ordinary telemetry repeats every 35 ms; no slot submits both ordinary and
  error frames behind the two motor commands. This prevents error-log
  starvation when only one TX mailbox is free. Failed submissions are retried.

- ESP32 `vcu_can_errors` now accepts the terminal command-name argument;
  no arguments display the log, and `clear` clears the cached error history.

- STM32 motor/controller faults no longer permanently latch the VCU or discard
  RUN intent after the controller clears its fault. Recovery requires fresh,
  fault-free feedback from each faulted controller and the existing 500 ms
  neutral arming dwell. Offline feedback cannot acknowledge recovery.
- Explicit STOP and internal control/CAN TX/scheduling fault latches retain
  their existing restart requirements. No automatic controller RESET is sent.

### Tests

- Added host regressions for motor-fault recovery through the actual protocol,
  motor feedback, RC, speed synchronization, slew and CAN command layers.
- Added `STM32F407/tests/run_host_tests.py` to run the three STM32 host suites.

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
