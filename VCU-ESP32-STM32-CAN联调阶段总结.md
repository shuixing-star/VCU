# VCU 当前任务总结

更新时间：2026-07-31

## 当前目标

本阶段目标是搭建 VCU 测试版链路：

- 使用 ESP32-C3 替代当前未引出 CAN 的 STM32 部分做早期车辆 CAN 控制测试。
- 去掉 GPS，改用 CRSF 接收机作为遥控输入。
- 使用 CAN 控制美鑫双控制器车辆运行。
- 后续逐步迁移到 STM32F407 作为实时控制核心，ESP32 作为日志、诊断、网关或工具侧模块。

## 工程位置

- 工作根目录：`E:\Work\2026\VCU`
- ESP32 工程：`E:\Work\2026\VCU\ESP32_VCU_Gateway`
- STM32 工程：`E:\Work\2026\VCU\STM32F407`
- ESP32 现场命令文档：`E:\Work\2026\VCU\ESP32_VCU_Gateway\docs\vcu-field-test-commands.md`
- STM32 架构文档：`E:\Work\2026\VCU\STM32F407\docs\architecture.md`

## ESP32 当前状态

ESP32 当前用于 CRSF 接收、CAN 控制、日志和现场测试。

已验证：

- CRSF 链路正常，`crsf_status` 显示 link ok，valid frames 持续增加，CRC/UART 错误为 0。
- CAN 通信正常，能收到控制器状态，也能发送控制帧。
- 双控制器命令采用扩展帧。
- 控制器只接受轴 2 数据，轴 1 不参与控制。
- ESP32 对左右控制器连续发送两包 axis2 速度命令。

## CRSF 通道约定

- CH1：前进/后退
- CH3：左转/右转
- CH7：ARM/KILL
- CH8：RESET，向下拨触发，边沿检测

当前 RC 状态查看命令：

```text
vcu_rc_status
```

运行状态查看命令：

```text
vcu_drive_status
```

## ESP32 驱动参数

当前默认：

- 最大目标转速：`4800 rpm`
- 最小运动转速：`400 rpm`
- CAN 控制周期：`5 ms`
- 默认加速度：`2550 rpm/s`
- CAN ramp 字段单位：`10 rpm/s`
- ramp 最大值：`255`

因此默认 `2550 rpm/s` 对应 CAN ramp：

```text
2550 / 10 = 255 = 0xFF
```

启动命令：

```text
vcu_drive_start
```

带自定义周期和加速度：

```text
vcu_drive_start 5 2550
```

停止：

```text
vcu_drive_stop
```

## 美鑫控制器 CAN 约定

当前 ESP32 默认双控制器 ID：

```text
left : status 0x1801E002, command 0x1801E601, direction -1
right: status 0x1801E001, command 0x1801E600, direction +1
```

注意：

- 这里的 left/right 是车辆物理左右。
- VCU 内部使用车辆归一化方向：前进为正。
- 左控制器物理方向在 CAN 适配层反向。
- 控制器命令帧为扩展帧。

axis2 速度命令格式：

- `data[0]`：控制标志
  - `0x41`：axis2 速度模式并使能
  - `0x40`：axis2 速度模式但不使能，安全零速
- `data[1..2]`：axis2 目标转速，int16，大端
- `data[3]`：ramp，加速度，单位 `10 rpm/s`

如果目标速度是 `4800 rpm`，默认加速度 `2550 rpm/s`，则发送数据类似：

```text
41 12 C0 FF 00 00 00 00
```

如果目标是 `-4800 rpm`：

```text
41 ED 40 FF 00 00 00 00
```

实际左右电机是否取反由对应控制器方向配置决定。

## ESP32 常用现场命令

```text
crsf_status
crsf_channels
mx_can_status
mx_can_raw
vcu_rc_status
vcu_drive_status
vcu_drive_start
vcu_drive_stop
mx_log_start 20
mx_log_status
mx_log_stop
```

查看或覆盖 CAN ID：

```text
mx_can_ids
mx_can_ids <left_status> <left_command> [right_status right_command]
```

发送安全零速诊断帧：

```text
mx_can_send_zero
```

查看最近 CAN 原始收发：

```text
mx_can_raw
```

## ESP32 日志系统

日志系统保留并已接入 VCU 数据。

日志命令：

```text
mx_log_start 20
mx_log_status
mx_log_stop
```

日志文件路径类似：

```text
mx_log/vcu_001.csv
mx_log/vcu_002.csv
```

当前日志字段已精简，去掉了阶段性调试字段，例如：

- `crsf_valid_frames`
- `arm_raw`
- `reset_raw`
- `arm_requested`
- `reset_requested`
- `left_mix_pm`
- `right_mix_pm`
- `drive_period_ms`
- `drive_accel_rpm_s`
- `drive_can_ramp`
- `drive_tx`
- `zero_tx`
- `reset_tx`
- `ignored_can_rx`
- `left_configured`
- `left_has_status`
- `left_age_ms`
- `right_configured`
- `right_has_status`
- `right_status_count`
- `right_age_ms`

保留核心字段用于看输入、目标、实际转速、故障、电压和 CAN 原始帧。

## 已观察到的问题

负转速附近曾出现短暂跳正值。

目前判断：

- 从日志和图表看，VCU 目标值没有对应跳变。
- 更像是控制器内部速度环/PID 或反馈处理导致的瞬态。
- 需要继续结合 `mx_can_raw` 和控制器侧参数确认。

控制器 fault `29`：

- 用户确认含义是控制器报 CAN 通讯故障。
- 现象上 VCU 能收到状态帧，所以不是物理 CAN 完全不通。
- 更可能是控制器没有持续收到它认可的控制命令，或命令 ID、使能位、周期、控制器参数存在不一致。

## STM32F407 当前状态

STM32F407 已根据新硬件外部晶振改为 12 MHz。

时钟配置：

- HSE：`12 MHz`
- PLLM：`12`
- PLLN：`336`
- PLLP：`2`
- PLLQ：`7`
- SYSCLK：`168 MHz`
- APB1：`42 MHz`
- APB2：`84 MHz`
- 48 MHz 域：`48 MHz`

CAN1 配置：

- 引脚：`PB8 = CAN1_RX`，`PB9 = CAN1_TX`
- 位速率：`500 kbit/s`
- Prescaler：`6`
- BS1：`11TQ`
- BS2：`2TQ`

接线提醒：

- `PB9 / CAN_TX` 接 CAN 收发器 `TXD`
- `PB8 / CAN_RX` 接 CAN 收发器 `RXD`
- MCU TX/RX 不直接接 CANH/CANL

## STM32 当前临时 CAN 心跳测试

注意：STM32 当前为了单独测试 CAN，已经把原来的标准网关 heartbeat 临时改成指定扩展帧。

当前 STM32 每 `100 ms` 发送：

- 帧格式：扩展帧
- 帧类型：数据帧
- ID：`0x1101`
- DLC：`1`
- 数据：`02`

改动位置：

```text
E:\Work\2026\VCU\STM32F407\Core\Src\freertos.c
```

关键宏：

```c
#define CAN_HEARTBEAT_PERIOD_MS 100U
#define CAN_HEARTBEAT_ID 0x1101U
#define CAN_HEARTBEAT_DATA 0x02U
```

当前 STM32 CAN 测试已经在逻辑分析仪/分析工具上看到帧。

## STM32 原设计网关协议

STM32 架构文档中原计划：

- STM32 是实时车辆控制核心。
- ESP32 是日志、主机通信和诊断网关。
- STM32 与 ESP32 的网关帧原本使用标准帧：

```text
0x680 heartbeat/state
0x681 CRSF input
0x682 targets
0x683 left motor
0x684 right motor
0x685 diagnostics
0x686 ESP32 -> STM32 guarded RUN/STOP/RESET
```

当前 `0x1101` 扩展心跳是临时 CAN 测试状态。下一个任务如果恢复 STM32/ESP32 网关，需要把这个临时心跳改回标准网关协议，或者显式保留为车辆上位系统需要的心跳。

## 编译和刷写

ESP32 编译/刷写路径：

```powershell
cd E:\Work\2026\VCU\ESP32_VCU_Gateway
idf.py -B build_k2 -p COM18 flash
```

注意不要在 `build_k2` 目录里面执行 `idf.py flash`，否则会报：

```text
CMakeLists.txt not found in project directory
```

如果遇到 ESP-IDF Python 环境不一致，使用当前环境：

```text
E:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe
```

或重新 `fullclean` 后用同一套 ESP-IDF 环境配置。

VESC Tool 升级 ESP32 固件时，选择构建产物：

```text
E:\Work\2026\VCU\ESP32_VCU_Gateway\build_k2\ESP32_VCU_Gateway.bin
```

STM32 工程目前是 Keil/MDK 工程：

```text
E:\Work\2026\VCU\STM32F407\MDK-ARM\STM32F407.uvprojx
```

本环境没有直接验证 MDK 编译。

## 下一个任务建议

1. 明确下一阶段控制核心：继续 ESP32 实车测试，还是恢复 STM32 作为主控。
2. 如果继续 ESP32 测车，优先看：
   - 控制器 fault 29 是否因命令帧周期/ID/使能/参数导致。
   - 负转速跳正值是否为控制器速度环问题。
   - 是否需要调整控制器 PID、加速度或速度滤波。
3. 如果转到 STM32 主控，优先恢复/确认：
   - STM32 CRSF 输入。
   - STM32 到美鑫控制器双扩展帧命令。
   - STM32 到 ESP32 标准网关遥测。
   - 是否保留当前 `0x1101` 扩展心跳。
4. 任何实车测试前保持默认安全顺序：
   - 先 `mx_can_status`
   - 再 `vcu_rc_status`
   - 开日志 `mx_log_start 20`
   - 最后 `vcu_drive_start`
   - CH7 ARM 前确认轮子架空或车辆安全固定

