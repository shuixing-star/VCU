# STM32 VCU software structure

The STM32F407 is the sole owner of real-time vehicle control. The ESP32 is a
logging, host-communication, and diagnostics gateway connected over CAN.

## Dependency direction

```text
Core (CubeMX generated)
  -> Bare-metal main loop
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
| `vcu_app` | Bounded ISR-queue drain, CAN routing, control scheduling and telemetry |
| `vcu_crsf_port` | USART3/PB11, 420000-8N1 receive-only ISR and timestamped byte ring |
| `vcu_crsf_uart` / `vcu_crsf` | CRSF frame parsing, channels, CRC counters and 150 ms link timeout |
| `vcu_rc_drive` | Channel normalization, arm/reset extraction and tracked-vehicle mixing |
| `vcu_drive_runtime` | RUN ownership, neutral dwell, safety state, direct target output, motor-fault recovery and internal fault latch |
| `vcu_speed_sync` | Two-track speed correction and weak-field derating |
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
Bare-metal main loop (once per observed TIM7 1 ms tick)
  -> vcu_app_step
  -> CRSF receiver
  -> USART3 PB11 timestamped bytes
  -> CRSF channels and link state
  -> arm/reset + throttle/steering normalization
  -> left/right tracked-vehicle mix
  -> 500 ms neutral arming dwell
  -> speed synchronization + direct target output
  -> paired left/right MX CAN command transaction
```

Motor CAN status frames are timestamped in the CAN ISR. Buffered frames retain
their actual arrival times, so a delayed foreground loop cannot make old CRSF or motor
feedback appear fresh. A control-loop gap greater than the configured period
plus 20 ms latches `SCHEDULE_OVERRUN`, sends a disabled zero target, and
requires an explicit RESET before RUN can be accepted again.

## CAN contract

| Direction | CAN ID | Frame |
| --- | ---: | --- |
| STM32 -> motor | `0x0C01EFD0` extended | left command, protocol section 2.2 |
| motor -> STM32 | `0x1801D0EF` extended | left electrical status, section 2.3 |
| motor -> STM32 | `0x1802D0EF` extended | left state/fault status, section 2.3 |
| STM32 -> motor | `0x0C01F0D0` extended | right command, controller F0 (240), VCU D0 |
| motor -> STM32 | `0x1801D0F0` extended | right electrical status, controller F0 (240) |
| motor -> STM32 | `0x1802D0F0` extended | right state/fault status, controller F0 (240) |
| STM32 -> ESP32 | `0x680` standard | heartbeat/state |
| STM32 -> ESP32 | `0x681` standard | CRSF input |
| STM32 -> ESP32 | `0x682` standard | base/final targets |
| STM32 -> ESP32 | `0x683` standard | left motor |
| STM32 -> ESP32 | `0x684` standard | right motor |
| STM32 -> ESP32 | `0x685` standard | diagnostics |
| ESP32 -> STM32 | `0x686` standard | guarded RUN/STOP/RESET |
| STM32 -> ESP32 | `0x687` standard | queued CAN error event |

The standard gateway IDs are the current project allocation and must be checked
against the final vehicle-wide CAN database before production release.

Motor baud rate and the two configurable IDs per controller are centralized in
`App/Inc/vcu_motor_config.h`. `DRIVER_RECEIVE_ID` is the command ID and
`DRIVER_SEND_ID` is the electrical-status ID. The state/fault ID is derived by
adding `0x00010000` to `DRIVER_SEND_ID`, matching the two section 2.3 frames.
The default remains 500 kbit/s to match the existing STM32/ESP32 bus; 250
kbit/s is supported when every node is configured to the document default. The
RENG controller addresses are EF (239) for left and F0 (240) for right;
the VCU address remains D0 for both. The controller CAN ID must match this
allocation and takes effect after the controller restarts.

Commands use the section 2.2 single-drive layout; the section 2.4 dual-motor
examples are not applicable to this controller firmware. Signed torque and speed
commands are little-endian offset-binary values with a wire zero of 32000.

The observed section 2.3 electrical feedback uses the same 32000 zero offset for
bus current, phase current, and speed. Voltage remains ordinary little-endian
unsigned 0.1 V units. The captured frame `FA 03 00 7D 00 7D 00 7D` decodes to
101.8 V, 0 A, 0 A, and 0 rpm.

State-frame controller and motor temperatures use a +40 wire offset, covering
the documented -40 to 210 C range in raw values 0 to 250. The captured bytes
`4F 3C` therefore decode to 39 C controller and 20 C motor temperature.

There is no application handshake with the motor controllers. A controller is
online only after both section 2.3 feedback frames are valid and fresh and the
state-frame heartbeat has advanced. Left/right motor command frames are prepared
first and require two free bxCAN mailboxes.
If the second enqueue fails, the first request is aborted. RESET is deliberately
split across two control periods: paired disabled-zero first, paired RESET
second. This avoids the four-frame burst exceeding the three hardware mailboxes.

## Timing and safety contract

- Bare-metal main loop runs once per observed 1 ms tick; missed ticks are skipped.
- Main code and nested ISRs share the 4096-byte MSP stack. No RTOS heap or task hooks.
- Startup retains a 512-byte C-library heap reservation; control uses static storage.
- CAN processing is limited to 16 frames per foreground iteration; CRSF draining is limited
  to four 64-byte chunks per iteration to preserve a bounded control execution time.
- HAL 1 ms timebase uses TIM7, leaving TIM6 available for a future DAC trigger.
- Default control period: 5 ms; configurable RUN period: 5 to 100 ms.
- VCU does not apply a speed ramp. Motor controllers own acceleration/deceleration.
- Legacy RUN acceleration is accepted for compatibility but ignored; runtime reports 0.
- CRSF link timeout: 150 ms.
- Motor electrical/state/heartbeat safety timeout: 500 ms.
- Speed-sync feedback timeout: 150 ms; invalid dual-controller feedback fails closed. With exactly one controller online, synchronization is disabled and the offline side target is forced to zero.
- Speed synchronization update: 20 ms.
- Arm requires throttle and steering within +/-50 per mille for 500 ms.
- Startup sends a paired zero-speed, output-disabled transaction immediately
  after CAN initialization.
- CAN automatic retransmission and automatic bus-off recovery are enabled.
- Motor/controller fault feedback inhibits both outputs while retaining RUN intent.
  Every controller observed with a fault must report fault-free status with fresh
  electrical/state feedback and an advancing heartbeat before recovery. An offline
  controller or an expired cached clear cannot clear its pending fault. All defined
  motor fault bits follow this rule, regardless of the protocol classification.
- Motor-fault recovery sends no automatic RESET to the controllers. The VCU leaves
  FAULT after valid clear feedback and then applies the existing RC link, arm and
  500 ms throttle/steering neutral dwell checks. Targets restart from zero through
  normal target output. An explicit STOP remains stopped until RUN is requested.
- Control failure, CAN TX failure, and scheduling overrun remain latched;
  RESET sends safe-zero and reset transactions before clearing them. A CAN TX
  failure during motor-fault inhibition becomes an internal latched fault.
- FAULT describes both an active external motor fault and an internal latched
  failure; `fault_latched` specifically indicates the latter. Disabled zero-target
  command pairs continue to be attempted each control period in either case.
- ESP32 cannot directly own a traction command. Its lowest CAN EID TX entry
  rejects both motor command IDs, including raw host forwarding and Lisp calls.

## Bounded CAN transmission

All TX APIs are called by the foreground loop, never an ISR. Single-frame telemetry
is best effort: only one background mailbox may be outstanding, reserving two
for the motor pair. Congestion skips a status frame without inventing a hardware
CAN error. The foreground loop polls the port each 1 ms iteration and before transmissions;
a background request pending for 2 ms is cancelled. Error-event transmission
failures do not recursively create more error events.

Motor pair submission has a shared 2 ms HAL-tick budget for mailbox availability
and transmission completion. Short contention and automatic retransmission are
allowed inside that budget. Success now means both requests completed with TXOK,
not merely that HAL accepted their payloads. A failed/expired pair requests abort
of its remaining mailboxes and permits up to 1 ms of abort cleanup. Outstanding
abort requests block reuse until hardware confirms release. Runtime then retains
its existing zero-output attempt and fault-latch behavior; there is no unbounded
software queue or automatic return to run after a persistent TX failure.

These are software budgets measured with the 1 ms HAL time base, not a verified
worst-case wall-clock guarantee under IRQ preemption. Normal 5 ms motor command
scheduling is retained. TX work can occupy the foreground loop while it waits; CAN RX, CRSF
and the HAL tick IRQs remain enabled. Physical timing and bus loading must be
validated on the vehicle. Do not enable TX completion interrupts without changing
the polling design: they would consume TXOK before the foreground observes it.

CAN_TSR abort writes contain only ABRQ bits because completion bits are
write-one-to-clear. A read/modify/write abort can otherwise erase another
mailbox's TXOK. Background ownership is cleared when HAL reuses a completed
mailbox for a motor command, so a stale background timer cannot abort that new
command. Startup zero is attempted, but a missing controller ACK during boot
must not send the foreground loop into Error_Handler; RX/diagnostics and normal
runtime arming/fault gates continue to operate.

## CAN error capture

Error frames require hardware TXOK before the reporter removes its pending
record. Diagnostic admission alone is not success: timeout/abort returns false
and the same event is retried on a later error slot. Diagnostics use the same
2 ms transmit and 1 ms abort-cleanup budgets, once per 35 ms error slot. This
adds a bounded foreground wait during error reporting. Hardware TXOK confirms bus
transmission, not that the ESP32 application processed and stored the event.
The event-pipeline regression links the real capture port, ESP link, protocol,
and telemetry reporter; an admitted-but-failed event survives bus recovery.

Telemetry uses seven 5 ms slots: six ordinary status frames and one reserved
CAN error event slot. Each status type repeats every 35 ms. At most one
telemetry/error frame is submitted per slot, leaving room for the two motor
commands already submitted by the control loop. Previously the error frame
was a fourth submission after those two commands and ordinary telemetry,
which could indefinitely starve errors when all three mailboxes stayed busy.
An error rejected or not confirmed by the CAN diagnostic API remains pending
for the next reserved slot. Hardware completion is not an ESP32 application
delivery acknowledgement.
ESP32 keeps this log in RAM; restart/clear loses history and does not request
a replay from STM32.

The STM32 snapshots every reported bxCAN error into a bounded queue. Each event
contains its sequence, source, last-error code, warning/passive/bus-off and
TX/RX-overflow flags, HAL state, TEC, REC, and a 10 ms timestamp. Events are
forwarded on standard ID `0x687` after the bus can transmit again; forwarding
failures do not recursively create more diagnostic events. The ESP32 retains
the newest 64 events.

Use `vcu_can_errors clear` before a test, reproduce the loaded-wheel failure,
then run `vcu_can_errors`. `vcu_drive_status` shows the total, retained count,
sequence gaps, and latest event. A total larger than the retained count means
older ESP32 records were overwritten; a nonzero sequence-gap count means the
STM32 queue overflowed or diagnostic frames were lost.

## Confirmed hardware assumptions

- USART3 is CRSF receive-only on PB11 at 420000 baud; PB10 is not driven.
- ESP32 and STM32 communicate over CAN1 on PB8/PB9 at 500 kbit/s.
- CAN RX0/SCE run at NVIC priority 6; CRSF USART3 runs at priority 7.
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
Production release also requires an IWDG policy in which only a verified healthy control path
feeds the watchdog; it is intentionally not enabled during this migration.

## Verification

Protocol 2.2 command/2.3 feedback byte-vector tests and motor CAN heartbeat
runtime test are kept in `tests`. The changed application modules also pass a
strict host-GCC syntax build. Hardware validation still requires CAN trace,
CRSF loss, feedback timeout, bus-off, reset sequencing, and foreground-loop stall
fault-injection tests on the vehicle bench.

### Host regression command

From the HASO_10KW workspace root, run:

```text
python VCU/STM32F407/tests/run_host_tests.py
```

The runner uses GCC with `-Wall -Wextra -Werror` and runs protocol vectors, motor
CAN feedback/heartbeat tests, and the drive-runtime recovery suite. The runtime
suite uses real CAN status decoding, RC mixing, synchronization and command
encoding with only the CAN transmit port mocked. It covers all 17 defined fault
bits on each side, persistent/simultaneous faults, missing and stale feedback,
frozen heartbeat, STOP/link/arm gates, interrupted neutral dwell, single-controller
operation, and retained internal fault/reset behavior. It performs no hardware I/O.

The telemetry host regression models two occupied CAN mailboxes and verifies
error delivery, retry/order under full mailboxes, and normal telemetry progress
under a sustained error backlog. It does not model physical bus timing.

The CAN-port host suite compiles the real port with a HAL/register model. It
covers transient and sustained congestion, delayed completion/retries, partial
pairs, TXOK versus empty mailboxes, bounded abort and reuse, telemetry expiry,
write-one-to-clear completion preservation, and tick wrap. The startup suite
verifies a failed initial zero does not disable later control and telemetry.
The model does not emulate electrical CAN arbitration, bit timing, or vehicle EMI.

## Bare-metal entry and build (2026-09-16)

`main()` initializes HAL/TIM7, clocks, GPIO, USART1 and CAN1, then calls
`vcu_app_init()` once. The application initializes USART3 CRSF reception,
CAN filters/notifications and drive state. Each new HAL tick runs one
`vcu_app_step(now_ms)`; missed ticks are not replayed. Control remains 5 ms by
default, with direct target output and the existing overrun/fault/neutral gates.
The foreground and interrupts use the startup MSP stack (4 KiB).
There are no RTOS tasks, task heaps, PendSV context switches or OS delays.
TIM7 owns the tick; SysTick is disabled. CAN/CRSF ISRs capture/queue incoming
data and errors; foreground code parses and transmits.

The active Keil target excludes `freertos.c` and all FreeRTOS middleware;
these files remain as historical source. CubeMX configuration also has
FreeRTOS disabled. Build outputs are `MDK-ARM/BareMetal/STM32F407_BareMetal.*`,
separate from the old RTOS binary. See [baremetal.md](baremetal.md).
CAN error probe task-switch bits retain their protocol position and read zero.

Open `MDK-ARM/STM32F407_BareMetal.uvprojx` for delivery. This separate project
prevents an already-open legacy Keil session from saving its stale RTOS config
over the delivery entry point. Both projects use the same source and outputs.
