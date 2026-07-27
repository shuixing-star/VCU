#ifndef __BSP__UART_H
#define __BSP__UART_H
/***********************************************************************************************************************************
 ** 【代码编写】  魔女开发板团队
 ** 【最后版本】  2026-03-18-01
 ** 【淘    宝】  https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** 【文件名称】  bsp_UART.h
 **
 ** 【 功  能 】  串口通信底层驱动（UART1~UART6）
 **               通信格式：波特率可配 - 无校验 - 8位数据 - 1位停止位
 **
 ** 【 约  定 】  本文件所有串口均采用异步通信方式。
 **               2024年起统一使用 "UART" 命名，不再使用旧版 "USART"。
 **
 ** 【适用环境】  适配芯片：STM32F4系列
 **               适配软件：兼容 STM32CubeIDE（GCC）、Keil MDK（ARMCC）
 **               适配库：HAL库 / 标准库
 **
 ** 【串口引脚】  各串口默认初始化引脚如下：
 **               1- UART1  TX_PA9   RX_PA10
 **                  特别说明：魔女开发板板载USB转TTL，已硬件连接UART1，使用烧录USB即可与电脑串口通信
 **               2- UART2  TX_PA2   RX_PA3
 **               3- UART3  TX_PB10  RX_PB11
 **               4- UART4  TX_PC10  RX_PC11
 **               5- UART5  TX_PC12  RX_PD2
 **               6- UART6  TX_PC6   RX_PC7
 **
 ** 【移植说明】  1- 使用CubeMX工程时，无需额外配置UART
 **               2- 支持HAL库、标准库工程直接使用
 **               3- 复制UART文件夹至目标工程目录
 **               4- 添加头文件路径：Keil -> Option -> C/C++ -> Include Paths
 **               5- 添加工程文件：将 bsp_UART.c 加入工程
 **               6- 引用头文件：#include "bsp_uart.h"
 **
 ** 【代码使用】  每组串口封装7个函数（初始化1个 + 发送3个 + 接收3个），以UART1为例：
 **               1、初始化串口：          void      UART1_Init(uint32_t ulBaudrate);                                    // 配置GPIO、波特率、8N1格式、中断
 **               2、发送指定长度数据：    void      UART1_SendData(uint8_t *puData, uint16_t usNum);                    // 参数：数据指针、字节数
 **               3、发送格式化字符串：    void      UART1_SendString(const char *pcString, ...);                        // 用法：与printf类似
 **               4、发送AT指令并等待应答：uint8_t   UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);  // 适用：ESP8266、蓝牙等AT指令模块; 参数：指令、期待应答、超时时间(ms); 返回：0=失败 1=成功
 **               5、获取接收数据长度：    uint16_t  UART1_GetRxNum(void);                                               // 返回值>0表示收到一帧新数据
 **               6、获取接收数据指针：    uint8_t*  UART1_GetRxData(void);                                              // 返回接收缓冲区首地址
 **               7、清空接收状态：        void      UART1_ClearRx(void); **                                             // 数据处理完成后必须调用，准备下一次接收
 **
 ** 【更新记录】  2026-03-17  优化printf重定向，兼容GCC/ARMCC双编译器
 **               2024-10-22  中断服务函数改用宏定义，提升可读性，优化注释
 **               2024-07-02  新增Modbus_CRC16校验计算与校验函数
 **               2024-06-30  优化UART初始化，增加奇偶校验位配置
 **               2024-06-20  优化AT指令函数名称与执行机制
 **               2024-06-07  新增AT指令等待应答函数UARTx_WaitACK()
 **               2024-06-07  优化UARTx_SendString()函数实现
 **               2024-06-06  优化代码适配，支持标准库与HAL库双环境
 **               2024-04-04  UARTx_SendString()取消static缓存，降低RAM占用，适配LVGL
 **               2024-03-27  修改xUSATR_TypeDef结构体，优化缓冲区存储方式
 **               2024-02-04  简化接收函数命名，使用Rx替代Received
 **               2024-01-09  合并静态变量至结构体，调用更直观
 **               2024-01-03  完善函数接口与注释说明
 **               2023-12-25  封装接收函数，取消全局接收变量
 **               2023-12-23  完成HAL库驱动支持
 **               2023-01-27  增加配置宏定义，完善注释规范
 **               2021-12-16  优化接收机制，通过字节数判断数据接收完成
 **               2021-11-03  完善发送/接收函数返回值处理逻辑
 **               2021-08-14  新增收发缓冲区大小配置宏，优化资源占用
 **               2021-08-14  发送函数重命名：UARTx_Printf → UARTx_SendString
 **               2020-09-02  文件创建，实现UART1中断收发、DMA收发功能
 **               2021-06-04  完成UART1~UART6全串口收发功能开发
 **               2021-06-09  统一文件格式，规范注释排版
 **               2021-06-22  完善使用说明与功能注释
 **
************************************************************************************************************************************/
#include "stdio.h"        // 引用C语言标准库，它定义了标准输入输出函数; 如：printf、scanf、sprintf、fopen 等; 
#include "stdarg.h"       // 引用C语言标准库，它定义了处理可变参数的宏; 如：va_start、va_arg、va_end、va_list 等;
#include "string.h"       // 引用C语言标准库，它定义了操作字符串的函数; 如：strcpy、strcmp、strlen、memset、memcpy 等;




#ifdef USE_STDPERIPH_DRIVER
    #include "stm32f4xx.h"                        // 引用 标准库的底层支持文件
#endif

#ifdef USE_HAL_DRIVER
    #include "stm32f4xx_hal.h"                    // 引用 HAL库的底层支持文件
#endif



/*****************************************************************************
 ** 移植配置修改区
 ** 备注：除非有特殊要求，否则，下面参数已通用：RS485、蓝牙模块、ESP8266模块、串口屏等
****************************************************************************/
// 串口开关
#define UART1_EN                       1          // 串口1，0=关、1=启用;  倘若没用到UART1, 置0，就不会开辟UART1发送缓存、接收缓存，省一点资源;
#define UART2_EN                       1          // 串口2，0=关、1=启用;  同上;
#define UART3_EN                       1          // 串口3，0=关、1=启用;  同上;
#define UART4_EN                       1          // 串口4，0=关、1=启用;  同上;
#define UART5_EN                       1          // 串口5，0=关、1=启用;  同上;
#define UART6_EN                       1          // 串口5，0=关、1=启用;  同上;
// 发送缓冲区大小
#define UART1_TX_BUF_SIZE           2048          // 配置每组UART发送环形缓冲区数组的大小，单位：字节数; 
#define UART2_TX_BUF_SIZE            512          // -- 只有在前面串口开关在打开状态，才会定义具体的缓冲区数组
#define UART3_TX_BUF_SIZE            512          // -- 默认值：512，此值已能适配大部场景的通信; 如果与ESP8266之类的设备通信，可适当增大此值。
#define UART4_TX_BUF_SIZE            512          // -- 值范围：1~65535; 注意初始化后，不要超过芯片最大RAM值。
#define UART5_TX_BUF_SIZE            512          // -- 注意此值是一个环形缓冲区大小，决定每一帧或多帧数据进入发送前的总缓存字节数，先进先出。
#define UART6_TX_BUF_SIZE            512          //
// 接收缓冲区大小
#define UART1_RX_BUF_SIZE           2048          // 配置每组UART接收缓冲区的大小，单位：字节; 此值影响范围：中断里的接收缓存大小，接收后数据缓存的大小
#define UART2_RX_BUF_SIZE           1024          // --- 当接收到的一帧数据字节数，小于此值时，数据正常；
#define UART3_RX_BUF_SIZE           1024          // --- 当接收到的一帧数据字节数，超过此值时，超出部分，将在中断中直接弃舍，直到此帧接收结束(发生空闲中断); 
#define UART4_RX_BUF_SIZE           1024          // 
#define UART5_RX_BUF_SIZE           1024          // 
#define UART6_RX_BUF_SIZE           1024          // 
// 结束-配置修改



/*****************************************************************************
 ** 声明全局函数
 **
 ** 每个串口的函数:
 ** 初始化    1个   波特率-None-8-1
 ** 发送      3个   发送指定长度数据、字符串、AT指令 
 ** 接收      3个   获取字节数、获取数据、清0
****************************************************************************/
// UART1
void      UART1_Init(uint32_t ulBaudrate);                                      // 初始化串口1; GPIO引脚PA9+PA10、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART1_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART1_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART1_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART1_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART1_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART2                                                                        
void      UART2_Init(uint32_t ulBaudrate);                                      // 初始化串口2; GPIO引脚PA2+PA3、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART2_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART2_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART2_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART2_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART2_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART2_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART3                                                                        
void      UART3_Init(uint32_t ulBaudrate);                                      // 初始化串口3; GPIO引脚PB10+PB11、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART3_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART3_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART3_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART3_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART3_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART3_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART4                                                            
void      UART4_Init(uint32_t ulBaudrate);                                      // 初始化串口4; GPIO引脚PC10+PC11、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART4_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART4_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART4_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART4_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART4_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART4_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART5                                                                        
void      UART5_Init(uint32_t ulBaudrate);                                      // 初始化串口5; GPIO引脚PC12+PD2、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART5_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART5_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART5_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART5_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART5_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART5_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)
// UART6                                                                        
void      UART6_Init(uint32_t ulBaudrate);                                      // 初始化串口6; GPIO引脚PC6+PC7、中断优先级、通信参数：波特率可设、8位数据、无校验、1个停止位
void      UART6_SendData(uint8_t *puData, uint16_t usNum);                      // 发送指定数据; 参数：数据地址、字节数
void      UART6_SendString(const char *pcString, ...);                          // 发送字符串;   参数：字符串地址; 使用方法如同printf
uint8_t   UART6_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs);    // 本函数，针对ESP8266、蓝牙模块等AT固件，用于等待返回期待的信息; 参数：AT指令字符串、期待返回信息字符串、ms等待超时; 返回：0-执行失败、1-执行成功 
uint16_t  UART6_GetRxNum(void);                                                 // 获取接收到的最新一帧字节数
uint8_t * UART6_GetRxData(void);                                                // 获取接收到的数据 (缓存的地址)
void      UART6_ClearRx(void);                                                  // 清理接收到的数据 (清理最后一帧字节数，因为它是判断接收的标志)     

// 辅助函数：Modbus_CRC16校验
uint16_t  Modbus_GetCRC16(uint8_t *_pcData, uint16_t _usLen);                   // 计算CRC16值; 参数：原始数据、原始数据字节数; 高位在前，低位在后
uint8_t   Modbus_CheckCRC16(uint8_t *_pcData, uint16_t _usLen);                 // 对带ModbusCRC16校验的数据段进行校验; 返回：0-错误、1-匹配正确;




#endif

