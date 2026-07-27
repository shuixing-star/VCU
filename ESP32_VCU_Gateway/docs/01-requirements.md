# Phase 1 — 需求汇总

## 1. 系统概述

O2M2 新 VCU (Vehicle Control Unit)，基于 STM32F4 LQFP-100，管理两台 EZkontrol PMSM 电机控制器。

| 项目 | 规格 |
|------|------|
| 系统电压 | 96V DC |
| MCU | STM32F407 (LQFP-100) |
| 电机控制器 | EZkontrol ×2 (PMSM, FOC, Cortex-M4) |
| 控制方式 | 硬线控制为主，CAN 读取状态为辅 |
| 视频 | 2路模拟摄像头 + AT7456E OSD 字符叠加 |
| 无线 | ESP32-C3 Mini (外接天线, WiFi/BT), UART 连接 |
| 充电管理 | VCU 不参与，充电器独立 |
| 后续扩展 | SD 卡, IMU, GPS |

---

## 2. 外部连接器定义 (8个, 沿用旧VCU)

### 2.1 Video Camera No1 / No2 (GX12-3B, 3Pin)

| Pin | 信号 | 方向(VCU视角) | 电平 | 说明 |
|-----|------|:--:|------|------|
| 1 | VIDEO_IN | IN | CVBS 1Vpp | 模拟视频输入 |
| 2 | +12V | OUT | 12V | 摄像头供电 |
| 3 | GND | — | 0V | 电源地 |

### 2.2 Remote Receiver (GX12-4B, 4Pin)

| Pin | 信号 | 方向 | 电平 | 说明 |
|-----|------|:--:|------|------|
| 1 | UART_FAULT_TX | OUT | 3.3V/5V | 故障/状态数据发送 |
| 2 | +5V | OUT | 5V | 遥控模块供电 |
| 3 | GND | — | 0V | |
| 4 | UART_FAULT_RX | IN | 3.3V/5V | 故障/状态数据接收 |

### 2.3 Local Control Terminal (GX16-9B, 9Pin)

| Pin | 信号 | 方向 | 电平 | 说明 |
|-----|------|:--:|------|------|
| 1 | +5V | OUT | 5V | 电位器供电 |
| 2 | THROTTLE | IN | 0-5V 模拟 | 油门(基础速度) |
| 3 | GND | — | 0V | |
| 4 | REVERSE(R) | IN | 开关量 | 右电机倒车 |
| 5 | BRAKE(R) | IN | 0-5V 模拟 | 右刹车(控制右转) |
| 6 | REVERSE(L) | IN | 开关量 | 左电机倒车 |
| 7 | PARKING(R+L) | IN | 开关量 | 驻车(左右联动) |
| 8 | BRAKE(L) | IN | 0-5V 模拟 | 左刹车(控制左转) |
| 9 | LOCAL/REMOTE KEY | IN | 开关量 | 本地/远程模式切换 |

> **操作逻辑**(履带车): THROTTLE 控制基础速度; BRAKE(L) 制动左履带→左转; BRAKE(R) 制动右履带→右转。3 路电位器(模拟 0-5V)。

### 2.4 EXT. RELAY (GX12-6B, 6Pin)

| Pin | 信号 | 方向 | 电平 | 说明 |
|-----|------|:--:|------|------|
| 1 | CAN_H | I/O | CAN 差分 | **(新增, 原 Reserved)** |
| 2 | UART_TX | OUT | 3.3V/5V | 扩展设备通信 (单向发送) |
| 3 | GND | — | 0V | |
| 4 | CAN_L | I/O | CAN 差分 | **(新增, 原 Reserved)** |
| 5 | +12V | OUT | 12V | 扩展设备供电 |
| 6 | GND | — | 0V | |

> EXT.RELAY 只有 UART_TX (单向)，无 UART_RX。

### 2.5 Motor Left (GX16-10B, 10Pin)

| Pin | 信号 | 方向 | 电平 | 对应EZkontrol CN1 | 说明 |
|-----|------|:--:|------|------|------|
| 1 | +5V | OUT | 5V | CN1-23 | 油门电位器供电 |
| 2 | THROTTLE | OUT | 0-5V 模拟 | CN1-12 | 油门信号 |
| 3 | GND | — | 0V | CN1-24 | |
| 4 | HOLD | OUT | 接地生效 | CN1-18 | 巡航 |
| 5 | BRAKE | OUT | 接地生效(低刹) | CN1-15 | 低刹 |
| 6 | SPEED_L | OUT | 三段式开关 | CN1-17 | 三速控制 |
| 7 | REVERSE | OUT | 接地生效 | CN1-16 | 倒车 |
| 8 | GND | — | 0V | | |
| 9 | PARKING | OUT | 接地生效 | CN1-19 | 驻车制动 |
| 10 | GND | — | 0V | | |

### 2.6 Motor Right (GX16-10B, 10Pin)

| Pin | 信号 | 方向 | 电平 | 对应EZkontrol CN1 | 说明 |
|-----|------|:--:|------|------|------|
| 1 | +5V | OUT | 5V | CN1-23 | |
| 2 | THROTTLE | OUT | 0-5V 模拟 | CN1-12 | |
| 3 | GND | — | 0V | CN1-24 | |
| 4 | HOLD | OUT | 开关量 | CN1-18 | |
| 5 | BRAKE | OUT | 开关量 | CN1-15 | |
| 6 | SPEED_L | OUT | 开关量 | CN1-17 | |
| 7 | REVERSE | OUT | 开关量 | CN1-16 | |
| 8 | GND | — | 0V | | |
| 9 | PARKING | OUT | 开关量 | CN1-19 | |
| 10 | GND | — | 0V | | |

### 2.7 Power (GX12-5B, 5Pin)

| Pin | 信号 | 方向 | 电平 | 说明 |
|-----|------|:--:|------|------|
| 1 | +96V | IN | 96V DC | 主电源正极 |
| 2 | -96V | IN | 0V (电池负) | 主电源负极 |
| 3 | Reserved | — | — | 待定 |
| 4 | KS_RELAY | — | 开关量 | 钥匙开关, 控制主接触器 |
| 5 | KS_RELAY | — | 开关量 | 钥匙开关, 控制主接触器 |

> KS_RELAY 是钥匙开关信号，控制主接触器线圈，非 MCU 控制。钥匙闭合 → 接触器吸合 → 96V 上电。

---

## 3. 内部接口

### 3.1 AT7456E OSD (SPI)

| 连接 | MCU引脚组 | 说明 |
|------|-----------|------|
| SPI (MOSI,MISO,SCK,CS) | SPI1 | 字符叠加数据 |
| VSYNC/HSYNC | GPIO | 视频同步信号 |
| VIDEO_IN | — | 来自摄像头切换后的模拟视频 |
| VIDEO_OUT | — | 叠加后输出至显示器 |

### 3.2 ESP32 (UART)

| 连接 | MCU引脚组 | 说明 |
|------|-----------|------|
| UART TX/RX | USART3 | 数据透传 |
| 可选: RTS/CTS | GPIO | 流控(待定) |
| 供电 | 3.3V | |

### 3.3 预留扩展接口 (板载, 非连接器)

| 外设 | 接口 | 说明 |
|------|------|------|
| SD Card | SPI2 | 数据记录 |
| IMU | I2C1 | 姿态传感器 |
| GPS | USART4 | 定位 |
| SWD | 4-pin 焊盘 | 板载调试, 不外露 |

### 3.4 CAN 总线

| 信号 | 连接 | 说明 |
|------|------|------|
| CAN_TX/RX | STM32F4 CAN1 | |
| CAN_H/L | MCP2562 收发器 → EXT.RELAY Pin1/4 | |
| 终端电阻 | 120Ω (板载, 可跳线选择) | |
| 挂载节点 | 电机控制器(左) + 电机控制器(右) + BMS | |

---

## 4. 电源树输入条件

| 电压轨 | 来源 | 负载 |
|--------|------|------|
| **96V** | Power 连接器 Pin1/2 | DC-DC 96→12V 模块输入 |
| **12V** | DC-DC 输出 | 摄像头×2, 扩展设备(EXT.RELAY), Buck 12→5V 输入 |
| **5V** | Buck 输出 | STM32F4, AT7456E, 油门电位器, Remote Receiver, Motor 接口 5V 输出 |
| **3.3V** | LDO 输出 | ESP32, SD, IMU, GPS |

**不在 VCU 盒子内的电源:**
- 96V→24V DC-DC (灯, USB 适配器) — 电池侧
- 充电回路 — 独立

---

## 5. 信号电平转换需求

STM32F4 GPIO 为 3.3V，以下控制信号**接地生效**，使用开漏输出 (N-MOS) 即可：

| 信号 | STM32侧 | 方式 | 说明 | 数量 |
|------|---------|------|------|:--:|
| BRAKE | 3.3V GPIO | N-MOS 开漏, 拉低到 GND | 低刹 | ×2 |
| REVERSE | 3.3V GPIO | N-MOS 开漏, 拉低到 GND | 倒车 | ×2 |
| HOLD | 3.3V GPIO | N-MOS 开漏, 拉低到 GND | 巡航 | ×2 |
| PARKING | 3.3V GPIO | N-MOS 开漏, 拉低到 GND | 驻车 | ×2 |
| SPEED_L | 待定 | 三段式开关 | 三速 | ×2 |
| KS_RELAY | 3.3V GPIO | 继电器驱动 | 主接触器 | ×2 |
| Local Control 输入 (ADC) | 0-5V | 电阻分压 → 3.3V, ESD 防护 | THROTTLE, BRAKE(R), BRAKE(L) | ×3 |
| Local Control 输入 (GPIO) | 开关量 | 上拉+ESD 防护 → 3.3V | REVERSE(R/L), PARKING, MODE | ×4 |
| UART 对外 | 3.3V TX | 电平匹配 | Remote Recv UART (双向), EXT.RELAY UART_TX (单向) | ×2 |

> BRAKE/REV/HOLD/PARKING 为接地生效，GPIO 高阻时释放(悬空)，GPIO 低时 N-MOS 导通拉低到 GND。

---

## 6. 功能逻辑需求

| 功能 | 输入 | 处理 | 输出 |
|------|------|------|------|
| 基础速度 | THROTTLE 电位器 (0-5V) | ADC 读取, 映射曲线 | 左/右 THROTTLE (DAC 0-5V) |
| 转向控制 | BRAKE(R), BRAKE(L) 电位器 (0-5V) | ADC 读取, 履带车差速逻辑 | 左/右 BRAKE (低刹, 接地生效) |
| 倒车 | REVERSE(R), REVERSE(L) 开关 | 方向逻辑 | 左/右 REVERSE (接地生效) |
| 驻车 | PARKING(R+L) 开关 | 联动 | 左/右 PARKING (接地生效) |
| 三速 | — (控制逻辑内部) | 档位判断 | 左/右 SPEED_L (三段式) |
| 巡航 | — | 保持车速 | 左/右 HOLD (接地生效) |
| 模式切换 | LOCAL/REMOTE KEY | 切换控制源 | 内部状态 |

> 履带车转向: THROTTLE 设定前进速度; BRAKE(L) 越大=左履带越慢=左转越急; BRAKE(R) 越大=右履带越慢=右转越急。
| OSD | 车速/状态/故障码 | AT7456E 渲染 | 视频叠加输出 |
| 遥控 | UART_FAULT_RX 数据 | 解析控制指令 | 替代本地控制 |
| 无线 | ESP32 UART 数据 | 调参/数据导出 | — |

---

## 7. 确认事项

- [x] EZkontrol 控制信号电平 — 实测验证, 原理图预留电平转换
- [x] EZkontrol CN1 REV=0 即 Forward — 不用管
- [x] 油门输出: **DAC 直出 0-5V**
- [x] STM32F4 型号: **STM32F407** (LQFP-100)
- [x] ESP32 型号: **ESP32-C3 Mini** (外接天线, 不带板载天线)
- [x] 扩展设备 UART_TX 协议 — 先不管
