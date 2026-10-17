# HardFault 定位版（2026-09-16）

本版本用于定位裸机运行时的异常写访问，不代表根因已修复。

## 串口队列核对与修改

- 两个数组分别为 256 个字节和 256 个 uint32_t 时间戳；掩码为 255。
- head/tail 是自然对齐的 volatile uint16_t。初始化前禁用 USART3 IRQ；ISR 只发布 head，前台只发布 tail。
- 保留读槽位前 DMB；新增读完字节和时间戳、发布 tail 前的 DMB。
- 访问槽位前检查当前 head/tail，不能只检查经掩码得到的 next。
- 首次越界记录到 g_vcu_crsf_ring_fault，随后停止该队列读写，直到重新初始化。
  不静默掩码或清空异常值。遥控数据不再更新后仍由既有 CRSF 超时保护处理。
- HAL_GetTick 使用 HAL 默认实现，只返回 uwTick，没有等待或动态分配。

## 故障快照

HardFault 汇编入口先保存异常入栈地址与 EXC_RETURN，再使用独立的 512 字节栈捕获现场。
快照保存在 g_vcu_fault，包含 CFSR/HFSR、有效位、原始栈地址及核心寄存器。
会处理浮点扩展栈帧，并检查栈地址范围；frame_readable 表示可读取，frame_plausible 仅表示通过有限合理性检查。
complete=0x4641554C 表示快照写完。若调试器在异常入口就停住，快照可能尚未生成。

本诊断版 main.c 的 VCU_FAULT_PRECISE_WRITES=1，关闭 Cortex-M4 默认写缓冲，以帮助将不精确写错误定位到具体指令。
此设置会改变执行时序，不是修复证据；完成诊断后应评估是否恢复为 0。

加载此次诊断固件后，清除旧绝对地址断点。复现故障并暂停后，在 Keil Watch 添加并展开：

```
g_vcu_fault
g_vcu_crsf_ring_fault
```

不要再用上一版的固定 SP 地址解释这一版的异常。

## 验证

- ARMCC 编译：0 errors，0 warnings。
- 检查反汇编：HardFault 入口读取 MSP/PSP 和 LR 后才切换栈；main 确实设置 ACTLR.DISDEFWBUF。
- 八组主机回归通过，新增 CRSF 队列回绕、满队列丢弃、时间戳配对、串口错误和损坏下标注入测试。
- 未连接、烧录或启动硬件。尚需用户复现以确定根因。

日志与修改前备份：outputs/fault_diagnostic_20260916。
