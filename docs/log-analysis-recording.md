# AutoRecovery 配套 Log Analysis 记录适配

本次使用用户指定的 ESP32 工程：E:/Work/2026/HASO_10KW/VCU/ESP32_VCU_Gateway。
STM32 基线：outputs/VCU_AutoRecovery_5000_CH6_20260928/firmware/STM32F407_AutoRecovery_5000rpm_CH6_K2_239_240.hex。
STM32 原文件及控制源码未修改：保留 ±5000 rpm、CH6 制动、自动恢复及左右 239/240。

## 交付

firmware/ESP32_VCU_Gateway_LogAnalysis_AutoRecovery.bin 是 ESP32-C3 应用固件。
它不是 STM32 文件，也不是从地址 0 开始的整片镜像。通过原有 RENG Tool ESP32 应用固件升级流程使用。
本次没有连接硬件、烧录、擦除 NVS 或修改运行参数。不要因本次日志更新重刷 STM32。
之前从 E:/Work/2026/VCU/ESP32_VCU_Gateway 构建的文件不作为本次交付。

电脑端使用 E:/Work/2026/vesc_tool/dist/RENG_Tool_7.01_Windows_x64/reng_tool_7.01_sd_recording.exe。
无需安装 logui.vescpkg，也不需要新建面板。

## 使用

1. 选择本机 VESC Express T（local），进入 Log Analysis，等待状态返回。
2. 选择采样频率（默认20 Hz，1～50 Hz），点 Start recording。
3. 确认 samples 增加、errors 为0；文件写在 SD 卡 mx_log/vcu_NNN.csv。
4. 点 Stop recording，等 Stopped 后在 Source > Connected Device 点 Refresh，打开 mx_log。
5. 选中 CSV，Open 查看曲线，Save as 下载文件。旧的逗号CSV不自动转换。

启动、停止和查询仍使用 mx_log_start、mx_log_stop、mx_log_status。
上电不自动记录。断开电脑不会停止已经启动的 SD 记录。
记录条数增加只说明文件正在保存；stm32_online 和 fresh_mask 可判断遥测是否有效。
无遥测时不能把全零数据视为控制器真实数值。

## 数据与边界

仅修改 mx_log.c 的CSV输出，并增加 vcu_log_sample 字段转换模块和构建引用。
25列包括时间、在线/遥控状态、状态/原因、油门/转向、左右目标/实际转速、故障、
电压、温度、CAN溢出、数据新鲜度、反馈年龄和输出使能。
这版 STM32 的制动状态保留为 drive_state=5、drive_reason=14；未额外增加 CAN 报文。
CAN接收/转发、配置结构和NVS、硬件引脚、协议编解码、原有FlightLog功能均保留指定工程版本。

完整离线验证结果和文件哈希见 manifest.json，原修改文件见 before，补丁见 logging-only.patch。
chain_test/SIMULATED_*.csv 是模拟测试文件，不是实车采集记录。
实车 CAN 扫描恢复、SD写入及断线恢复仍需设备验证，不能仅据离线测试宣称已经恢复。
