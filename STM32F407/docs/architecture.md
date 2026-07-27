# STM32 VCU software structure

The STM32F407 is the sole owner of real-time vehicle control. The ESP32 is a
logging, host-communication, and diagnostics gateway connected over CAN.

## Dependency direction

```text
Core (CubeMX generated)
  -> App
      -> Protocol
      -> BSP
          -> STM32 HAL
```

`Protocol` must remain independent of STM32 HAL so its pack/unpack functions can
be tested on a host computer.

## Active module ownership

| Module | Responsibility |
| --- | --- |
| `vcu_app` | Drain ISR queues, route CAN frames, schedule control and telemetry |
| `vcu_crsf_port` | USART3/PB11, 420000-8N1 receive-only ISR and timestamped byte ring |
| `vcu_crsf_uart` / `vcu_crsf` | CRSF frame parsing, channels, CRC counters and 150 ms link timeout |
| `vcu_rc_drive` | Channel normalization, arm/reset extraction and tracked-vehicle mixing |
| `vcu_drive_runtime` | RUN ownership, neutral dwell, safety state, real-time slew and fault latch |
| `vcu_speed_sync` | Two-track speed correction and weak-field derating |
| `vcu_target_slew` | Acceleration-limited target generation using measured elapsed time |
| `vcu_motor_can` | Motor IDs, direction signs, status filtering and paired command transactions |
| `vcu_can_port` | CAN1 queues, paired/non-blocking TX, bus-off recovery and error counters |
| `vcu_esp_link` | Accept only guarded RUN/STOP/RESET requests from ESP32 |
| `vcu_control_telemetry` | Round-robin STM32 state telemetry to ESP32 |
| `vcu_gateway_protocol` | Shared deterministic ESP32/STM32 payload contract |
| `mx_can_protocol` | Deterministic motor-controller payload contract |

The older `vcu_inputs`, `vcu_driver_intent`, `vcu_safety`, `vcu_actuators`,
`vcu_can_gateway`, and `vcu_telemetry` skeletons remain on disk for later local
I/O work, but are deliberately excluded from the active Keil target. The old
`bsp_CAN.c` and `bsp_UART.c` teaching drivers are also excluded; re-adding them
would create duplicate CAN callbacks or a duplicate USART3 ISR.

## Active control flow

```text
CRSF receiver
  -> USART3 PB11 timestamped bytes
  -> CRSF channels and link state
  -> arm/reset + throttle/steering normalization
  -> left/right tracked-vehicle mix
  -> 500 ms neutral arming dwell
  -> speed synchronization + measured-time target slew
  -> paired left/right MX CAN command transaction
```

Motor CAN status frames are timestamped in the CAN ISR. Buffered frames retain
their actual arrival times, so a delayed main loop cannot make old CRSF or motor
feedback appear fresh. A control-loop gap greater than the configured period
plus 20 ms latches `SCHEDULE_OVERRUN`, sends a disabled zero target, and
requires an explicit RESET before RUN can be accepted again.

## CAN contract

| Direction | CAN ID | Frame |
| --- | ---: | --- |
| motor -> STM32 | `0x1801E002` extended | left status |
| motor -> STM32 | `0x1801E001` extended | right status |
| STM32 -> motor | `0x1801E601` extended | left command |
| STM32 -> motor | `0x1801E600` extended | right command |
| STM32 -> ESP32 | `0x680` standard | heartbeat/state |
| STM32 -> ESP32 | `0x681` standard | CRSF input |
| STM32 -> ESP32 | `0x682` standard | base/final targets |
| STM32 -> ESP32 | `0x683` standard | left motor |
| STM32 -> ESP32 | `0x684` standard | right motor |
| STM32 -> ESP32 | `0x685` standard | diagnostics |
| ESP32 -> STM32 | `0x686` standard | guarded RUN/STOP/RESET |

The standard gateway IDs are the current project allocation and must be checked
against the final vehicle-wide CAN database before production release.

Left/right motor frames are prepared first and require two free bxCAN mailboxes.
If the second enqueue fails, the first request is aborted. RESET is deliberately
split across two control periods: paired disabled-zero first, paired RESET
second. This avoids the four-frame burst exceeding the three hardware mailboxes.

## Timing and safety contract

- Default control period: 5 ms; configurable RUN period: 5 to 100 ms.
- Target slew uses actual elapsed time, not the configured period constant.
- CRSF link timeout: 150 ms.
- Motor status safety timeout: 500 ms.
- Speed-sync feedback timeout: 50 ms; invalid feedback fails closed.
- Speed synchronization update: 20 ms.
- Arm requires throttle and steering within +/-50 per mille for 500 ms.
- Startup sends a paired zero-speed, output-disabled transaction immediately
  after CAN initialization.
- CAN automatic retransmission and automatic bus-off recovery are enabled.
- Motor fault, control failure, CAN TX failure, and scheduling overrun are
  latched; RESET sends safe-zero and reset transactions before clearing them.
- ESP32 cannot directly own a traction command. Its lowest CAN EID TX entry
  rejects both motor command IDs, including raw host forwarding and Lisp calls.

## Confirmed hardware assumptions

- USART3 is CRSF receive-only on PB11 at 420000 baud; PB10 is not driven.
- ESP32 and STM32 communicate over CAN1 on PB8/PB9 at 500 kbit/s.
- ESP32 UART0 GPIO21/GPIO20 is reserved for the UBlox GPS.
- One brake analog input represents simultaneous left and right braking.
- The speed-select outputs and `KS_RELAY` MCU pin are unused.
- The future throttle-output path includes the DAC-side signal chain,
  AMC1311DWVR isolation amplifier, and external op-amp gain stage.

USART3 is owned directly by `vcu_crsf_port` and is not generated by the current
`.ioc`. If CubeMX is regenerated, do not also generate `USART3_IRQHandler`;
either keep the BSP-owned vector or deliberately migrate ownership to Core.

## Not included in this slice

This slice is the complete migration of the former ESP32
CRSF-to-tracked-motor-CAN control chain. It does not yet implement the STM32
local analog throttle, shared brake analog input, DAC/PWM throttle waveform,
reverse/hold/parking hard lines, or control-source arbitration. Those need
confirmed pin activity levels and analog scaling before being enabled.

## Verification

The active Keil target builds with ARMCC 5.06 update 5 at zero errors and zero
warnings. Pure protocol/synchronization/slew regression sources are kept in the
ESP repository `tests` directory; hardware validation still requires CAN trace,
CRSF loss, feedback timeout, bus-off, reset sequencing, and main-loop stall
fault-injection tests on the vehicle bench.
