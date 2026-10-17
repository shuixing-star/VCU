# VCU 故障日志链路 vcu-log-1.0.0

本次仅改 VCU STM32、ESP32 和日志工具。没有修改 K2/RENG 控制器固件、电机算法、保护阈值或速度轨迹，也没有连接、烧录、写参数或启动电机。现场 K2 是否为 v1.0.83 no_mos_temp、实际故障侧及完整内部故障名仍待现场确认。

## 工程与已有能力

- STM32 实际基线：`VCU/STM32F407/MDK-ARM/STM32F407_BareMetal.uvprojx`，2026-09-16 裸机调度，TIM7 毫秒时钟，默认控制周期 5 ms。旧 RTOS 工程不作为入口。
- 新日志构建入口：`VCU/STM32F407/MDK-ARM/STM32F407_FlightLog.uvprojx`，独立 `FlightLog/` 输出，启用 `VCU_FLIGHT_LOG_ENABLED`。
- ESP32 实际工程：`VCU/ESP32_VCU_Gateway`，ESP32-C3 / VESC Express T / `HW_VCU_STM32_GATEWAY`，ESP-IDF 5.5.4。原有 `conf_general.h` 为 7.00，CMake PROJECT_VER 为 6.0；两者原样保留，本次诊断功能独立标识为 `vcu-log-1.0.0`，不得拿这些编号当成电机控制器版本。
- 用户确认：左 239（EF），右 240（F0），VCU 地址 D0。STM32 编译地址不变；ESP32 旧原始报文诊断表的右侧地址修正为 F0。
- 原链路有 0x680..0x688 遥测/错误事件、64 条错误缓存、`mx_log` SD CSV，以及 `COMM_FILE_LIST` / `COMM_FILE_READ` 文件下载协议。缺少完整电流、原始报文历史、统一源时钟、独立发送结果和故障窗口；旧 CSV 对缺失值填零。本次增加故障记录器，原 `mx_log_start/stop/status` 保留兼容，诊断应使用本说明的新日志。

## 操作步骤

以下为后续得到硬件操作授权、部署两端日志固件后的步骤，本次没有执行硬件操作。

1. 准备可正常挂载的 SD 卡，启动 VCU 与 ESP32。日志自动在 RAM 中采集，不依赖 Wi-Fi，也不需要发送 RUN 或 `vcu_start`。
2. 从现有 ESP32 终端执行 `vcu_log_status`，确认 ready=1，samples、tx 随运行增加。ready 只表示本机记录器已启动；计数持续增加才证明 STM32 日志流已到达。若显示 unavailable，检查显示的空闲堆；须同时部署新 STM32 与 ESP32 固件才能获得完整字段。
3. 允许条件下等待至少 5 秒建立前窗，再按现场既定流程复现故障。非零原始故障位、STM32 FAULT、CAN 成对发送失败、已见过快照后断流超过 750 ms，均可触发。首次触发后继续收 3 秒，然后冻结并自动写盘。后续故障不会无限延长后窗。
4. `vcu_log_status` 查看 frozen=1、exported=1 及目录，例如 `/sdcard/mx_log/flight_1234ABCD_001/`。确认目录内有 `COMPLETE.txt`，才表示全部 CSV 已正常关闭。
5. 通过现有支持文件列表/读取的 VESC Tool 文件管理入口，下载 `mx_log/flight_.../` 整个目录；Wi-Fi 或 USB 连接都只是导出通道。也可在设备安全断电后取卡读取。写盘期间不要拔卡。目录使用现有 `file_basepath`，默认 `/sdcard/`。
6. 在 PC 上运行（不连接硬件）：

   ```powershell
   python E:/Work/2026/HASO_10KW/VCU/tools/inspect_vcu_log.py D:/captures/flight_1234ABCD_001
   ```

   得到 `timeline.csv` 和 `summary.txt`。工具校验原始记录 CRC、合并时间线，并报告实际覆盖范围。`snapshots.csv` 可直接用 Excel 查看；它将周期快照与事件分别写出，查看先后关系应按 `stm_capture_ms`/会话排序，或使用合并工具。
7. 无自动触发但希望保留当前过程时，执行 `vcu_log_export`；它只触发日志窗口，不发电机命令。SD 写入失败时 RAM 保持冻结，可先用既有 `sd_status` 检查挂载；确认卡已就绪后用 `sd_mount` 恢复挂载，再执行同一日志命令重试。每次尝试写到新目录，不覆盖已有文件；无 COMPLETE 的目录视为未完成。
8. 导出成功后执行 `vcu_log_rearm`，重新积累下一次前窗。未成功导出的已触发记录不允许被 rearm 清掉。默认只保留首次故障，需明确重新布防才能采集下一次。

## 数据与字段

### snapshots.csv

| 字段组 | 含义与单位 |
| --- | --- |
| esp_session、stm_boot_nonce、boot_nonce_valid | ESP32 启动随机标识、STM32 硬件 RNG 启动随机标识及有效位。随机 nonce 不是全局唯一 ID；STM32 RNG 不可用时明确标为无效，序号回退另作会话重置/歧义处理。 |
| esp_rx_dequeue_ms | ESP32 从 TWAI 驱动取出最后一个分片的本机运行毫秒；不是控制器采样时间，也不是硬件 RX 时间戳。 |
| stm_capture_ms、control_step_ms、rc_rx_ms | STM32 完成该轮控制/遥测后的采集时刻、对应控制预览计算时刻、该控制预览使用的 CRSF 接收时刻。均来自 STM32 HAL 毫秒时钟。 |
| snapshot_seq、record_kind、gateway_command、stm_build_id | 32 位采集序号；1=周期，2=状态事件；网关命令 0=无、1=RUN、2=STOP、3=RESET；日志构建 ID 0x20260921。 |
| throttle_raw、steering_raw、arm_raw、reset_raw | 参与驱动的原始 CH3/CH1/CH7/CH8。未参与驱动的其他 CRSF 通道不在本记录中。 |
| throttle_per_mille、steering_per_mille | VCU 实际使用的归一化值，单位千分比。 |
| left/right_base_rpm、left/right_target_rpm | 混控基准与同步/限幅后目标，车辆方向坐标，rpm。 |
| left/right_wire_target_rpm、command_flags、command_raw_hex | 本次尝试编码的控制器目标与完整 8 字节命令，不能仅凭这些列认定发送成功。左侧方向符号由原配置转换；负的左侧线上目标不代表混控出错。 |
| left/right_txok_target_rpm | 仅该侧硬件 TXOK 已观察到时填写，否则空白。 |
| tx_attempt_ms、tx_observed_done_ms、tx_attempt_seq | 成对发送开始、前台观察发送结果的 STM32 时刻及 32 位尝试序号。不是精确 CAN 帧结束采样。 |
| mailbox_admitted_mask、txok_mask、pair_success | bit0=左，bit1=右；分别表示进入硬件邮箱、观察到 TXOK、原成对发送 API 是否成功。允许出现 TXOK=3 而 pair_success=0（原超时判断窗口），不能合并这三项。 |
| left/right_voltage_dV | 协议电压，0.1 V。 |
| left/right_bus_current_dA、phase_current_reported_dA | 协议上报的总线电流及“相电流”标量，0.1 A。后者不是三相电流波形或高频故障快照。 |
| left/right_raw_speed_rpm | 控制器原始上报转速，保留控制器坐标，未应用 VCU 反馈滤波/死区。 |
| controller_temperature_reported_C、motor_temperature_reported_C | 协议编码的温度，摄氏度。只说明报文值，不证明传感器存在/有效；no_mos_temp 固件的语义必须结合现场版本核实。 |
| left/right_fault_bits_hex、fault_code | 前者为原始三个故障位字节；后者为现有 STM32 协议解析器派生的首要故障编号。没有控制器内部完整 VESC 故障名，不能将派生编号当成完整内部故障码。 |
| electrical_age_ms、state_age_ms、heartbeat_age_ms、online | STM32 接收并接受相应反馈后的年龄、心跳前进年龄及原在线判断。年龄 65535 表示未知或已饱和，在 CSV 留空；原始记录保留哨兵。在线不等于所有数据刚刚更新。 |
| electrical_raw_hex、state_raw_hex | 两种原始反馈各 8 字节，支持后续重新解析。无效报文仍可保留原始字节，解析失败的数值留空。 |
| drive_state、drive_reason、flags | 原 VCU 状态与原因。flags bit0=CRSF 在线、bit1=使能请求、bit2=复位请求、bit3=运行请求、bit4=运行输出条件成立。实际发出使能还要看命令 flags 和 TXOK。 |
| stm_esr、stm_tec、stm_rec、stm_bus_off | CAN ESR 原值及 TEC/REC/bus-off 位。 |
| stm_tx_errors、stm_rx_overflows、stm_rx_errors、stm_error_events_lost | 原 CAN 端口计数器；tx_errors 包含遥测/日志传输错误，不等同于电机指令失败次数。 |
| stm_log_dropped、stm_tx_log_dropped、stm_queue_used、stm_queue_highwater、crsf_overflows | STM32 快照队列、逐次发送结果队列、队列占用/峰值与 CRSF 溢出计数。 |
| left/right_command_id、record_raw_hex | 实际编译的 CAN 控制 ID（十进制），以及完整 176 字节记录（含 CRC）。 |

缺失或不能解析的测量数值留空；有效反馈报告的 0 才填写 0。旧值可保留，但须结合年龄/在线字段判断，不能当新采样。

### 其他文件

- `tx_results.csv`：每次成对发送（默认约 200 Hz）的独立结果，包含低 16 位发送时间、低 16 位尝试序号、左右邮箱准入/TXOK、pair_success、左右命令心跳及耗时。目标/输入以 10 Hz 完整快照及状态事件记录，并非每个 5 ms 周期都有完整目标快照。两次快照间的快速目标变化不能靠插值伪造。
- `raw_events.csv`：左右原始故障位变化帧（包括清零）、网关 RUN/STOP/RESET 原始命令、0x687 CAN 错误和 0x688 TX probe。ESP32 时间独立标注，controller_sample_ms 始终留空，因为协议没有提供。
- `metadata.csv`：两端日志构建信息、控制器版本未知标记、触发点、窗口/频率、RAM 使用、堆水位、序号缺口、重组丢失、各环形缓冲覆盖次数及占用。ESP32 TWAI 状态/TEC/REC/漏收/硬件溢出/发送失败/总线错误/处理队列丢失是导出时快照，不冒充故障时刻快照。
- `COMPLETE.txt`：成功关闭全部 CSV 后才创建。断电、拔卡或写错误时不保证持久化；不带此标记的目录可能只有部分文件。

状态依次为 STOPPED、DISARMED、ARMING_NEUTRAL、ARMED、FAULT（0..4）。原因 0..13 依次为 INIT、STOPPED、RESET、CRSF_LOST、MOTOR_TIMEOUT、MOTOR_FAULT、KILL、WAITING_NEUTRAL、NEUTRAL_DWELL、ARMED、RUNNING、CONTROL_FAILURE、CAN_TX_FAILURE、SCHEDULE_OVERRUN。

## 时间与丢包规则

没有给控制器伪造时钟或采样时间，也没有假设 STM32、ESP32 上电同步。每份快照携带 STM32 完整时刻和 ESP32 末分片取出时刻，二者关系还包含排队/分片延迟，不能把它们的差当精确时钟偏差。

`inspect_vcu_log.py` 仅在附近 2 秒快照属于同一 STM32 nonce 时，将 TX 低 16 位毫秒按最近完整 STM32 时间解回；明确标为 `STM32_low16_unwrapped_near_snapshot`。不满足条件则保留低 16 位和空的完整时间。原始控制器帧仅保留 ESP32 取出时间。合并时间线按 ESP32 取出时刻排序，源时钟另列，不混成控制器时间。

完整快照带序号、CRC16-CCITT（初值 FFFF）和格式版本。缺片/跨记录混片不会被当作有效测量。TX 16 位序号回绕正常；重复记录忽略，明显回退另计 reset/reordered，避免错误报告 65535 个丢包。正常环形覆盖与传输丢失分别计数。

## RAM 与带宽

- 完整快照 176 B，10 Hz；CAN 0x690 每帧携带 6 B 记录内容，加 token/分片号 2 B，30 帧/记录，约 300 帧/s。
- CAN 0x691 每次成对控制尝试一帧，默认 5 ms 周期约 200 帧/s。新日志合计正常约 500 帧/s，突发状态事件共用 6 帧/10 ms 配额，长期上限 600 帧/s，且每毫秒最多提交一帧，无补跑突发。
- 按标准 8 字节 CAN 帧含最坏填充/间隔保守取 135 bit，500 kbit/s 总线上新增日志常态约 13.5%，限速上限约 16.2%。既有两侧控制约 400 扩展帧/s，按 160 bit 估算 12.8%；既有遥测最多约 5.4%。三者上限合计约 34.4%，还需加控制器反馈和其他总线流量，现场必须核对总线负载。
- 日志 ID 0x690/0x691 的仲裁优先级低于现有控制命令/控制器反馈，提交时必须有三个空邮箱，保留两个控制邮箱。没有等待日志 TXOK 的忙等；原错误报告在新日志目标也改成轮询确认，TXOK 后才移除事件。
- STM32 使用 8×176 B 快照队列、32×8 B TX 结果队列及少量状态。缓存满即计数，不等待 ESP32。ESP32/Wi-Fi/SD 故障不会让 STM32 日志层等待；原 CAN 物理总线失联保护仍然生效，日志不会绕开它。
- ESP32 缓存：90×180 B 完整快照、32×180 B 状态事件、1792×12 B 发送结果、128×16 B 原始关键报文，加重组和计数总计 **45,796 B**。默认 5 秒前、3 秒后，快照/发送结果槽位有少量余量。状态突变/失败风暴超过独立事件容量仍可能覆盖，以导出的计数和实际时间范围为准。
- 分配前要求缓存之外另有 32 KiB 堆余量和 5 KiB 任务开销预算；单独写盘任务栈 4 KiB、优先级 2。实际空闲堆受现场 Wi-Fi/BLE/Lisp 配置影响，离线链接不能替代启动水位实测。分配失败不会改变 STM32 控制，终端明确报告不可用。
- 主控中断仍只接收/排队；无文件写入、网络发送或字符串格式化。ESP32 接收路径只做有界内存采集/重组；写 CSV 位于独立低优先级任务。修复了原 ESP32 接收处理队列满时覆盖未读槽位的问题，并计数。

## 可定位与缺口

能定位参与控制的遥控输入、混控/发送目标、对应地址、准入与发送完成差异、反馈缺失/年龄、原始故障位、VCU 停止/复位/恢复动作，以及 CAN 物理错误和日志自身丢失。可以判断故障前后是否出现目标突变、发送失败、反馈陈旧或先故障后停止等时序关系。

不能仅靠这些日志证明过流根因。没有控制器内部 iqref、FOC 状态、三相高频电流、PWM/ADC 同步故障快照、完整内部故障名或控制器采样时间。协议中的低频相电流标量不能代替这些信息。K2 v83 的 ±300 rpm 启动门槛是用户提供的版本信息，现场未确认版本前不能套用判断，也没有将其改进 VCU 或其他产品。

ESP32 重启会丢失尚未持久化的 RAM 窗口；STM32 不会因此停等日志。重启后新 ESP 会话重新积累。未满 5 秒就触发、通信阻塞、CPU 复位、丢包或事件风暴，都可能使实际前后窗口不足，工具会报告实际覆盖，不补零或伪造历史。

## 离线验证与维护入口

```powershell
python VCU/STM32F407/tests/run_host_tests.py --output-dir outputs/vcu_log_20260921/host_tests
python VCU/tests/run_log_tests.py --output-dir outputs/vcu_log_20260921/log_tests
```

第一组包含原控制/协议/CAN/CRSF/调度回归以及新准入/TXOK/异步诊断检查。第二组包含 STM32 采集与限速、阻塞丢弃恢复、分片/CRC/时钟回绕/会话重置、故障清零保留、5+3 秒窗口、低内存拒绝、SD 失败后保留与重试、实际 CSV 写出和离线时间线。生成数据全部标记 **SIMULATED_HOST_TEST_NOT_FIELD_DATA**，不作为现场故障证据。

模块归属：STM32 `vcu_flight_log.c` 负责采集/限速；`vcu_can_port.c` 只观察发送事实；`common/vcu_log_wire.*` 管分片和 CRC；`common/vcu_log_store.*` 管窗口缓存；ESP32 `vcu_flight_log.c` 管终端和文件导出；`vcu_log_motor_decode.*` 复用 STM32 双反馈帧解析器并隔离旧 ESP32 解析器符号；`tools/inspect_vcu_log.py` 做离线校验和时间线。

修改前源码备份：`outputs/vcu_log_20260921/before_logging.zip`；原先未提交差异另存 `preexisting.diff`。最终构建日志、测试日志、固件 SHA256 和模拟样例位于同一交付目录。实际硬件 RAM 水位、总线负载、SD 写入和 Wi-Fi/ESP 重启联调尚未进行，需要后续明确授权。


## 本次构建结果

STM32 最终通过 ARMCC 5.06 update 7 全量编译/链接。Keil GUI 批处理第二次启动停滞，因此增加 `VCU/tools/build_stm32_log.py`，读取独立 Keil 项目的源文件、宏和包含目录，以同一 Cortex-M4.fp.sp、C99、O3 与现有内存布局离线构建。该脚本不含烧录/调试连接操作。源文件中的既有控制策略和保护条件未改。

ESP32 最终在独立 `build_logging/` 中通过 ESP-IDF 5.5.4 构建，app 镜像 0x172660 字节，最小应用分区 0x190000，剩余 0x1d9a0（约 7%）。原 `build_k2/` 与 STM32 `BareMetal/STM32F407_BareMetal.*` 历史固件保留。

记录在同一时刻既是周期采样又是状态事件时，仍计入周期快照环，状态/原因/命令同时保留，避免事件挤占整个周期历史。数据包逐字节序列化，不依赖两种 CPU 的 C 结构体布局。
