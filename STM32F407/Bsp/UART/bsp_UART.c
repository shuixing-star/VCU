/***********************************************************************************************************************
【文件名称】  bsp_UART.c

【功能描述】  STM32F407 串口底层驱动实现文件，包含UART1~UART6的GPIO初始化、通信参数配置、中断管理
              及收发数据、printf重定向、Modbus CRC16校验等功能函数
              
【适用平台】  主控芯片：STM32F407
              编译环境：Keil MDK、STM32CubeIDE
              库版本：标准库(STD)、HAL库 双兼容
              
【硬件引脚】  UART1_TX-PA9  UART1_RX-PA10
              UART2_TX-PA2  UART2_RX-PA3
              UART3_TX-PB10 UART3_RX-PB11
              UART4_TX-PC10 UART4_RX-PC11
              UART5_TX-PC12 UART5_RX-PD2
              UART6_TX-PC6  UART6_RX-PC7
              
【通信参数】  波特率可配置 | 无校验 | 8位数据位 | 1位停止位

【核心特性】  1. 接收：字节接收中断 + 帧空闲中断，自动识别完整数据帧
              2. 发送：环形缓冲区 + 发送中断，非阻塞式数据发送
              3. 兼容：标准库/HAL库自动适配，无需修改代码
              4. 易用：支持printf重定向、格式化字符串发送、AT指令收发
              5. 校验：内置Modbus CRC16查表法校验函数
              
【版本信息】  2025-11-11  初始版本，完善UART驱动及CRC16函数注释
              2026-03-18  优化printf重定向，兼容GCC/ARMCC双编译器，关闭半主机模式
              
【开发团队】  魔女开发板团队

【淘宝店铺】  https://demoboard.taobao.com

【备注说明】  1. 本文件通过宏定义开关UART1~UART6，按需启用节省资源
              2. 中断服务函数已实现，使用时需屏蔽工程中同名中断函数
              3. 收发缓冲区大小可在bsp_UART.h中通过宏定义修改
              4. 采用寄存器+库函数混合配置，兼顾移植性与执行效率
              5. printf默认重定向至UART1，阻塞式发送保证调试输出稳定
***********************************************************************************************************************/
#include "bsp_UART.h"             // 头文件




/*****************************************************************************
 ** 声明本地变量
****************************************************************************/
typedef struct
{
    uint16_t  usRxNum;            // 新一帧数据，接收到多少个字节数据
    uint8_t  *puRxData;           // 新一帧数据，数据缓存; 存放的是空闲中断后，从临时接收缓存复制过来的完整数据，并非接收过程中的不完整数据;

    uint8_t  *puTxFiFoData;       // 发送缓冲区，环形队列; 为了方便理解阅读，没有封装成队列函数
    uint16_t  usTxFiFoHead ;      // 环形缓冲区的队头(写入)
    uint16_t  usTxFiFoTail ;      // 环形缓冲区的队尾(读出)
} xUSATR_TypeDef;





/******************************************************************************
 * 函  数： delay_ms
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟168MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
            3、编译优化级别：Level 3(-O3)
 * 参  数： uint32_t  ms  毫秒值
 * 返回值： 无
 ******************************************************************************/
static volatile uint32_t ulTimesMS;    // 使用volatile声明，防止变量被编译器优化
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS)
        ulTimesMS--;                   // 操作外部变量，防止空循环被编译器优化掉
}





//////////////////////////////////////////////////////////////   UART-1   ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART1_EN

static xUSATR_TypeDef  xUART1 = { 0 };                      // 定义 UART1 的收发结构体
static uint8_t uaUART1RxData[UART1_RX_BUF_SIZE];            // 定义 UART1 的接收缓存
static uint8_t uaUART1TxFiFoData[UART1_TX_BUF_SIZE];        // 定义 UART1 的发送缓存

/******************************************************************************
 * 函  数： UART1_Init
 * 功  能： 初始化USART1的通信引脚、协议参数、中断优先级
 *          引脚：TX-PA10、RX-PA11
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t  ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART1_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    /* 使能 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);           // 使能 GPIOA  时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);          // 使能 USART1 时钟
    /* 配置 引脚的复用功能 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);       // 配置PA9复用功能： USART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);      // 配置PA10复用功能：USART1
    /* 重置 UART */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);          // 使能重置
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);         // 取消重置
    /* 配置 TX引脚 */
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;                     // 引脚编号
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // 引脚方向: 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // 输出模式：推挽
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // 上下拉：上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // 输出速度：50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    // 配置R X引脚
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                    // 引脚编号
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 UART */
    USART_InitTypeDef USART_InitStructure;                          // 声明USART初始化结构体
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // 设置波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 设置字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 设置一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART1, &USART_InitStructure);                       // 初始化USART
    /* 配置 中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                  // 开启 接收中断
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                  // 开启 空闲中断
    // 配置中断优先级
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // 声明 中断优先级结构体
    NVIC_InitStructure .NVIC_IRQChannel = USART1_IRQn;              // 指定中断通道
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // 设置抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // 设置响应优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 // 初始化NVIC
    /* 配置完成，使能UART */
    USART_Cmd(USART1, ENABLE);                                      // 使能USART1
#endif

#ifdef USE_HAL_DRIVER                                               // HAL库 配置
    /* 使能 时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();                                   // 使能GPIOA
    __HAL_RCC_USART1_CLK_ENABLE();                                  // 使能USART1
    /* 重置 UART */
    __HAL_RCC_USART1_FORCE_RESET();                                 // 使能重置
    __HAL_RCC_USART1_RELEASE_RESET();                               // 取消重置
    // 配置 引脚
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_9 | GPIO_PIN_10;               // 引脚 TX-PA9、RX-PA10
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;                    // 引脚复用功能
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                         // 初始化引脚工作模式
    /* 计算波特率 */
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);        // 波特率公式计算; USART1挂载在APB2, 时钟为系统时钟的2分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                                // 整数部分
    fraction = (temp - mantissa) * 16;                              // 小数部分
    USART1 -> BRR  = mantissa << 4 | fraction;                      // 设置波特率
    // 配置 USART
    USART1 -> CR1  =   0;                                           // 清0
    USART1 -> CR1 |=   0x01 << 2;                                   // 接收使能[02]: 0=失能、1=使能
    USART1 -> CR1 |=   0x01 << 3;                                   // 发送使能[03]：0=失能、1=使能
    USART1 -> CR1 |=   0x00 << 9;                                   // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    USART1 -> CR1 |=   0x00 << 10;                                  // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    USART1 -> CR1 |=   0x00 << 12;                                  // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    USART1 -> CR2  =   0;                                           // 数据清0
    USART1 -> CR2 &= ~(0x03 << 12);                                 // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    USART1 -> CR3  =   0;                                           // 数据清0
    USART1 -> CR3 &= ~(0x01 << 6);                                  // DMA接收[6]: 0=禁止、1=使能
    USART1 -> CR3 &= ~(0x01 << 7);                                  // DMA发送[7]: 0=禁止、1=使能
    /* 配置 中断 */
    USART1 -> CR1 &= ~(0x01 << 7);                                  // 关闭发送中断
    USART1 -> CR1 |=   0x01 << 5;                                   // 使能接收中断: 接收缓冲区非空
    USART1 -> CR1 |=   0x01 << 4;                                   // 使能空闲中断：超过1字节时间没收到新数据
    USART1 -> SR   = ~(0x00F0);                                     // 清理中断
    /* 配置 中断优先级 */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);                        // 设置指定中断的响应优先级;  参数：中断请求编号、抢占级、子优先级
    HAL_NVIC_EnableIRQ(USART1_IRQn);                                // 使能、启用指定的中断
    /* 配置完成，使能UART */
    USART1 -> CR1 |=   0x01 << 13;                                  // 使能UART开始工作
#endif

    /* 关联缓冲区 */
    xUART1.puRxData = uaUART1RxData;                                // 关联接收缓冲区的地址
    xUART1.puTxFiFoData = uaUART1TxFiFoData;                        // 关联发送缓冲区的地址

    /* 输出提示 */
    printf("\r\r\r===========  STM32F407VE 外设 初始化报告 ===========\r");                   // 输出到串口助手
    SystemCoreClockUpdate();                                                                  // 更新一下系统运行频率变量
    printf("系统时钟频率             %d MHz\r", SystemCoreClock / 1000000);                   // 输出到串口助手
    printf("UART1 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate); // 输出到串口助手
}

/******************************************************************************
 * 函  数： USART1_IRQHandler
 * 功  能： USART1的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
******************************************************************************/
void USART1_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART1_RX_BUF_SIZE];                           // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到外部缓存：xUARTx.puRxData[ ]

    /* 发送中断：把环形缓冲的数据，逐字节发出 */
    if ((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))  // 检查发送寄存器空中断使能，且发送寄存器为空; TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART1->DR = xUART1.puTxFiFoData[xUART1.usTxFiFoTail++];         // 从FIFO队列中取出一个数据，放入USART的发送寄存器(硬件会自动发出)，然后将FIFO的尾指针递增，指向下一个要发送的数据
        if (xUART1.usTxFiFoTail == UART1_TX_BUF_SIZE)                    // 检查FIFO尾指针是否到达了FIFO队列的末尾
            xUART1.usTxFiFoTail = 0;                                     // 将尾指针重置为0，实现环形队列的功能
        if (xUART1.usTxFiFoTail == xUART1.usTxFiFoHead)                  // 检查FIFO尾指针是否追上了头指针，即所有数据是否都已发送完毕
            USART1->CR1 &= ~USART_CR1_TXEIE;                             // 关闭发送寄存器空中断，防止中断服务程序被不必要地调用
        return;
    }

    /* 接收中断：逐个字节接收，存放到临时缓存 */
    if (USART1->SR & USART_SR_RXNE)                                      // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if (cnt == UART1_RX_BUF_SIZE)                                    // 当前帧已接收的字节量，已满缓存区的大小; 为避免溢出，本包后面接收到的数据直接舍弃;
        {
            printf("警告：UART1单帧接收量，已超出接收缓存大小！\r修改文件bsp_UART.h的UART1_RX_BUF_SIZE值,可解决此问题！\r");
            USART1->DR;                                                  // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART1->DR ;                                     // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位；
        return;
    }

    /* 空闲中断：判断一帧数据结束，整理数据到外部备读 */
    if (USART1->SR & USART_SR_IDLE)                                      // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART1.puRxData, rxTemp, UART1_RX_BUF_SIZE);              // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART1.usRxNum  = cnt;                                           // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                         // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART1_RX_BUF_SIZE);                            // 接收数据缓存数组，清零; 准备下一次的接收
        USART1 ->SR;
        USART1 ->DR;                                                     // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： UART1_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t*  puData   需发送数据的地址
 *          uint16_t  usNum    发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART1_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART1.puTxFiFoData[xUART1.usTxFiFoHead++] = puData[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART1.usTxFiFoHead == UART1_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART1.usTxFiFoHead = 0;
    }                                                            // 为了方便阅读理解，这里没有把此部分封装成队列函数，可以自行封装

    if ((USART1->CR1 & USART_CR1_TXEIE) == 0)                    // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        USART1->CR1 |= USART_CR1_TXEIE;                          // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART1_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART1_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                             // 新建一个可变参数列表
    va_start(ap, pcString);                                 // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                  // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                             // 清空可变参数列表
    UART1_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART1_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART1_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART1_ClearRx();                                              // 清0
    UART1_SendString(pcAT);                                       // 发送AT指令字符串
    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART1_GetRxNum())                                     // 判断是否接收到数据
        {
            UART1_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART1_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}


/******************************************************************************
 * 函  数： UART1_SendStringForDMA
 * 功  能： UART通过DMA发送数据，省了占用中断的时间
 *         【适合场景】字符串，字节数非常多，
 *         【不 适 合】1:只适合发送字符串，不适合发送可能含0的数值类数据; 2-时间间隔要足够
 * 参  数： char strintTemp  要发送的字符串首地址
 * 返回值： 无
 * 备  注:  本函数为保留函数，留作用户参考。为了方便移植，本文件对外不再使用本函数。
 ******************************************************************************/
#if 0
void UART1_SendStringForDMA(char *stringTemp)
{
    static uint8_t Flag_DmaTxInit = 0;                // 用于标记是否已配置DMA发送
    uint32_t   num = 0;                               // 发送的数量，注意发送的单位不是必须8位的
    char *t = stringTemp ;                            // 用于配合计算发送的数量

    while (*t++ != 0)  num++;                         // 计算要发送的数目，这步比较耗时，测试发现每多6个字节，增加1us，单位：8位

    while (DMA1_Channel4->CNDTR > 0);                 // 重要：如果DMA还在进行上次发送，就等待; 得进完成中断清标志，F4不用这么麻烦，发送完后EN自动清零
    if (Flag_DmaTxInit == 0)                          // 是否已进行过USAART_TX的DMA传输配置
    {
        Flag_DmaTxInit  = 1;                          // 设置标记，下次调用本函数就不再进行配置了
        USART1 ->CR3   |= 1 << 7;                     // 使能DMA发送
        RCC->AHBENR    |= 1 << 0;                     // 开启DMA1时钟  [0]DMA1   [1]DMA2

        DMA1_Channel4->CCR   = 0;                     // 失能， 清0整个寄存器, DMA必须失能才能配置
        DMA1_Channel4->CNDTR = num;                   // 传输数据量
        DMA1_Channel4->CMAR  = (uint32_t)stringTemp;  // 存储器地址
        DMA1_Channel4->CPAR  = (uint32_t)&USART1->DR; // 外设地址

        DMA1_Channel4->CCR |= 1 << 4;                 // 数据传输方向   0:从外设读   1:从存储器读
        DMA1_Channel4->CCR |= 0 << 5;                 // 循环模式       0:不循环     1：循环
        DMA1_Channel4->CCR |= 0 << 6;                 // 外设地址非增量模式
        DMA1_Channel4->CCR |= 1 << 7;                 // 存储器增量模式
        DMA1_Channel4->CCR |= 0 << 8;                 // 外设数据宽度为8位
        DMA1_Channel4->CCR |= 0 << 10;                // 存储器数据宽度8位
        DMA1_Channel4->CCR |= 0 << 12;                // 中等优先级
        DMA1_Channel4->CCR |= 0 << 14;                // 非存储器到存储器模式
    }
    DMA1_Channel4->CCR  &= ~((uint32_t)(1 << 0));     // 失能，DMA必须失能才能配置
    DMA1_Channel4->CNDTR = num;                       // 传输数据量
    DMA1_Channel4->CMAR  = (uint32_t)stringTemp;      // 存储器地址
    DMA1_Channel4->CCR  |= 1 << 0;                    // 开启DMA传输
}
#endif

/******************************************************************************
 * 函  数： UART1_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART1_GetRxNum(void)
{
    return xUART1.usRxNum ;
}

/******************************************************************************
 * 函  数： UART1_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 缓存地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART1_GetRxData(void)
{
    return xUART1.puRxData ;
}

/******************************************************************************
 * 函  数： UART1_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART1_ClearRx(void)
{
    xUART1.usRxNum = 0 ;
}
#endif  // endif UART1_EN





//////////////////////////////////////////////////////////////   UART-2   ///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART2_EN

static xUSATR_TypeDef  xUART2 = { 0 };                      // 定义 UART2 的收发结构体
static uint8_t uaUART2RxData[UART2_RX_BUF_SIZE];            // 定义 UART2 的接收缓存
static uint8_t uaUART2TxFiFoData[UART2_TX_BUF_SIZE];        // 定义 UART2 的发送缓存

/******************************************************************************
 * 函  数： UART2_Init
 * 功  能： 初始化USART2的通信引脚、协议参数、中断优先级
 *          引脚：TX-PA2、RX-PA3
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t  ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART2_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    /* 使能 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);           // 使能 GPIOA  时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);          // 使能 USART2 时钟
    /* 配置 引脚的复用功能 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);       // 配置PA2复用功能：USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);       // 配置PA3复用功能：USART2
    /* 重置 UART */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);          // 使能重置
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);         // 取消重置
    /* 配置 TX引脚 */
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                     // 引脚编号：TX_PA2
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // 引脚方向: 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // 输出模式：推挽
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // 上下拉：上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // 输出速度：50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 RX引脚 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;                     // 引脚编号：RX_PA3
    GPIO_Init(GPIOA, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 UART */
    USART_InitTypeDef USART_InitStructure;                          // 声明USART初始化结构体
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // 设置波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 设置字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 设置一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART2, &USART_InitStructure);                       // 初始化USART
    /* 配置 中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  // 开启 接收中断
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);                  // 开启 空闲中断
    /* 配置 中断优先级 */
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // 声明 中断优先级结构体
    NVIC_InitStructure .NVIC_IRQChannel = USART2_IRQn;              // 指定中断通道
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // 设置抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // 设置响应优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 // 初始化NVIC
    /* 配置完成，使能UART */
    USART_Cmd(USART2, ENABLE);                                      // 使能USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL库 配置
    /* 使能 时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();                                   // 使能GPIOA
    __HAL_RCC_USART2_CLK_ENABLE();                                  // 使能USART2
    /* 重置 UART */
    __HAL_RCC_USART2_FORCE_RESET();                                 // 使能重置
    __HAL_RCC_USART2_RELEASE_RESET();                               // 取消重置
    /* 配置 GPIO引脚 */
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_2 | GPIO_PIN_3;                // 引脚 TX-PA2、RX-PA3
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;                    // 引脚复用功能
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);                         // 初始化引脚工作模式
    /* 计算波特率 */
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // 波特率公式计算; USART2挂载在APB1, 时钟为系统时钟的4分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                                // 整数部分
    fraction = (temp - mantissa) * 16;                              // 小数部分
    USART2 -> BRR  = mantissa << 4 | fraction;                      // 设置波特率
    /* 配置 UART */
    USART2 -> CR1  =   0;                                           // 清0
    USART2 -> CR1 |=   0x01 << 2;                                   // 接收使能[02]: 0=失能、1=使能
    USART2 -> CR1 |=   0x01 << 3;                                   // 发送使能[03]：0=失能、1=使能
    USART2 -> CR1 |=   0x00 << 9;                                   // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    USART2 -> CR1 |=   0x00 << 10;                                  // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    USART2 -> CR1 |=   0x00 << 12;                                  // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    USART2 -> CR2  =   0;                                           // 数据清0
    USART2 -> CR2 &= ~(0x03 << 12);                                 // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    USART2 -> CR3  =   0;                                           // 数据清0
    USART2 -> CR3 &= ~(0x01 << 6);                                  // DMA接收[6]: 0=禁止、1=使能
    USART2 -> CR3 &= ~(0x01 << 7);                                  // DMA发送[7]: 0=禁止、1=使能
    /* 配置 中断 */
    USART2 -> CR1 &= ~(0x01 << 7);                                  // 关闭发送中断
    USART2 -> CR1 |=   0x01 << 5;                                   // 使能接收中断: 接收缓冲区非空
    USART2 -> CR1 |=   0x01 << 4;                                   // 使能空闲中断：超过1字节时间没收到新数据
    USART2 -> SR   = ~(0x00F0);                                     // 清理中断
    /* 配置 中断优先级 */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);                        // 设置指定中断的响应优先级;  参数：中断请求编号、抢占级、子优先级
    HAL_NVIC_EnableIRQ(USART2_IRQn);                                // 使能、启用指定的中断
    /* 配置完成，使能UART */
    USART2 -> CR1 |=   0x01 << 13;                                  // 使能UART开始工作
#endif
    /* 关联缓冲区 */
    xUART2.puRxData = uaUART2RxData;                                // 获取接收缓冲区的地址
    xUART2.puTxFiFoData = uaUART2TxFiFoData;                        // 获取发送缓冲区的地址
    /* 输出提示 */
    printf("UART2 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： USART2_IRQHandler
 * 功  能： USART2的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
void USART2_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART2_RX_BUF_SIZE];                           // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到外部缓存：xUARTx.puRxData[ ]

    /* 发送中断：把环形缓冲的数据，逐字节发出 */
    if ((USART2->SR & USART_SR_TXE) && (USART2->CR1 & USART_CR1_TXEIE))  // 检查发送寄存器空中断使能，且发送寄存器为空; TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART2->DR = xUART2.puTxFiFoData[xUART2.usTxFiFoTail++];         // 从FIFO队列中取出一个数据，放入USART的发送寄存器(硬件会自动发出)，然后将FIFO的尾指针递增，指向下一个要发送的数据
        if (xUART2.usTxFiFoTail == UART2_TX_BUF_SIZE)                    // 检查FIFO尾指针是否到达了FIFO队列的末尾
            xUART2.usTxFiFoTail = 0;                                     // 将尾指针重置为0，实现环形队列的功能
        if (xUART2.usTxFiFoTail == xUART2.usTxFiFoHead)                  // 检查FIFO尾指针是否追上了头指针，即所有数据是否都已发送完毕
            USART2->CR1 &= ~USART_CR1_TXEIE;                             // 关闭发送寄存器空中断，防止中断服务程序被不必要地调用
        return;
    }

    /* 接收中断：逐个字节接收，存放到临时缓存 */
    if (USART2->SR & USART_SR_RXNE)                                      // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if (cnt == UART2_RX_BUF_SIZE)                                    // 当前帧已接收的字节量，已满缓存区的大小; 为避免溢出，本包后面接收到的数据直接舍弃;
        {
            printf("警告：UART2单帧接收量，已超出接收缓存大小！\r修改文件bsp_UART.h的UART2_RX_BUF_SIZE值,可解决此问题！\r");
            USART2->DR;                                                  // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART2->DR ;                                     // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位；
        return;
    }

    /* 空闲中断：判断一帧数据结束，整理数据到外部备读 */
    if (USART2->SR & USART_SR_IDLE)                                      // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART2.puRxData, rxTemp, UART2_RX_BUF_SIZE);              // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART2.usRxNum  = cnt;                                           // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                         // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART2_RX_BUF_SIZE);                            // 接收数据缓存数组，清零; 准备下一次的接收
        USART2 ->SR;
        USART2 ->DR;                                                     // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： UART2_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* puData     需发送数据的地址
 *          uint8_t  usNum      发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART2_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART2.puTxFiFoData[xUART2.usTxFiFoHead++] = puData[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART2.usTxFiFoHead == UART2_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART2.usTxFiFoHead = 0;
    }

    if ((USART2->CR1 & USART_CR1_TXEIE) == 0)                    // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        USART2->CR1 |= USART_CR1_TXEIE;                          // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART2_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART2_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                             // 新建一个可变参数列表
    va_start(ap, pcString);                                 // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                  // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                             // 清空可变参数列表
    UART2_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART2_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART2_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART2_ClearRx();                                              // 清0
    UART2_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART2_GetRxNum())                                     // 判断是否接收到数据
        {
            UART2_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART2_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART2_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART2_GetRxNum(void)
{
    return xUART2.usRxNum ;
}

/******************************************************************************
 * 函  数： UART2_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART2_GetRxData(void)
{
    return xUART2.puRxData ;
}

/******************************************************************************
 * 函  数： UART2_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART2_ClearRx(void)
{
    xUART2.usRxNum = 0 ;
}
#endif  // endif UART2_EN





//////////////////////////////////////////////////////////////   USART-3   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART3_EN

static xUSATR_TypeDef  xUART3 = { 0 };                      // 定义 UART3 的收发结构体
static uint8_t uaUart3RxData[UART3_RX_BUF_SIZE];            // 定义 UART3 的接收缓存
static uint8_t uaUart3TxFiFoData[UART3_TX_BUF_SIZE];        // 定义 UART3 的发送缓存

/******************************************************************************
 * 函  数： UART3_Init
 * 功  能： 初始化USART3的通信引脚、协议参数、中断优先级
 *          引脚：TX-PB10、RX-PB11
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART3_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    /* 使能 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);           // 使能 GPIOB  时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);          // 使能 USART3 时钟
    /* 配置 引脚的复用功能 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);      // 配置PB10复用功能：USART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);      // 配置PB11复用功能：USART3
    /* 重置 UART */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);          // 使能重置
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);         // 取消重置
    /* 配置 TX引脚 */
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                    // 引脚编号
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // 引脚方向: 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // 输出模式：推挽
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // 上下拉：上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // 输出速度：50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 RX引脚 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;                    // 引脚编号
    GPIO_Init(GPIOB, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 UART */
    USART_InitTypeDef USART_InitStructure;                          // 声明USART初始化结构体
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // 设置波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 设置字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 设置一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART3, &USART_InitStructure);                       // 初始化USART
    /* 配置 中断 */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                  // 开启 接收中断
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);                  // 开启 空闲中断
    /* 配置 中断优先级 */
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // 声明 中断优先级结构体
    NVIC_InitStructure .NVIC_IRQChannel = USART3_IRQn;              // 指定中断通道
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // 设置抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // 设置响应优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 // 初始化NVIC
    /* 配置完成，使能UART */
    USART_Cmd(USART3, ENABLE);                                      // 使能USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL库 配置
    /* 使能 时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();                                   // 使能GPIOB
    __HAL_RCC_USART3_CLK_ENABLE();                                  // 使能USART3
    /* 重置 UART */
    __HAL_RCC_USART3_FORCE_RESET();                                 // 使能重置
    __HAL_RCC_USART3_RELEASE_RESET();                               // 取消重置
    /* 配置 GPIO引脚 */
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;              // 引脚 TX-PB10、RX-PB11
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;                    // 引脚复用功能
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                         // 初始化引脚工作模式
    /* 计算波特率 */
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // 波特率公式计算; USART3挂载在APB1, 时钟为系统时钟的4分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                                // 整数部分
    fraction = (temp - mantissa) * 16;                              // 小数部分
    USART3 -> BRR  = mantissa << 4 | fraction;                      // 设置波特率
    /* 配置 UART */
    USART3 -> CR1  =   0;                                           // 清0
    USART3 -> CR1 |=   0x01 << 2;                                   // 接收使能[02]: 0=失能、1=使能
    USART3 -> CR1 |=   0x01 << 3;                                   // 发送使能[03]：0=失能、1=使能
    USART3 -> CR1 |=   0x00 << 9;                                   // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    USART3 -> CR1 |=   0x00 << 10;                                  // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    USART3 -> CR1 |=   0x00 << 12;                                  // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    USART3 -> CR2  =   0;                                           // 数据清0
    USART3 -> CR2 &= ~(0x03 << 12);                                 // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    USART3 -> CR3  =   0;                                           // 数据清0
    USART3 -> CR3 &= ~(0x01 << 6);                                  // DMA接收[6]: 0=禁止、1=使能
    USART3 -> CR3 &= ~(0x01 << 7);                                  // DMA发送[7]: 0=禁止、1=使能
    // 配置中断
    USART3 -> CR1 &= ~(0x01 << 7);                                  // 关闭发送中断
    USART3 -> CR1 |=   0x01 << 5;                                   // 使能接收中断: 接收缓冲区非空
    USART3 -> CR1 |=   0x01 << 4;                                   // 使能空闲中断：超过1字节时间没收到新数据
    USART3 -> SR   = ~(0x00F0);                                     // 清理一次中断标志
    /* 配置 中断优先级 */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);                        // 设置指定中断的响应优先级;  参数：中断请求编号、抢占级、子优先级
    HAL_NVIC_EnableIRQ(USART3_IRQn);                                // 使能、启用指定的中断
    /* 配置完成，使能UART */
    USART3 -> CR1 |=   0x01 << 13;                                  // 使能UART开始工作
#endif
    /* 关联缓冲区 */
    xUART3.puRxData = uaUart3RxData;                                // 获取接收缓冲区的地址
    xUART3.puTxFiFoData = uaUart3TxFiFoData;                        // 获取发送缓冲区的地址
    /* 输出提示 */
    printf("UART3 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： USART3_IRQHandler
 * 功  能： USART3的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
void USART3_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART3_RX_BUF_SIZE];                           // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到外部缓存：xUARTx.puRxData[ ]

    /* 发送中断：把环形缓冲的数据，逐字节发出 */
    if ((USART3->SR & USART_SR_TXE) && (USART3->CR1 & USART_CR1_TXEIE))  // 检查发送寄存器空中断使能，且发送寄存器为空; TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART3->DR = xUART3.puTxFiFoData[xUART3.usTxFiFoTail++];         // 从FIFO队列中取出一个数据，放入USART的发送寄存器(硬件会自动发出)，然后将FIFO的尾指针递增，指向下一个要发送的数据
        if (xUART3.usTxFiFoTail == UART3_TX_BUF_SIZE)                    // 检查FIFO尾指针是否到达了FIFO队列的末尾
            xUART3.usTxFiFoTail = 0;                                     // 将尾指针重置为0，实现环形队列的功能
        if (xUART3.usTxFiFoTail == xUART3.usTxFiFoHead)                  // 检查FIFO尾指针是否追上了头指针，即所有数据是否都已发送完毕
            USART3->CR1 &= ~USART_CR1_TXEIE;                             // 关闭发送寄存器空中断，防止中断服务程序被不必要地调用
        return;
    }

    /* 接收中断：逐个字节接收，存放到临时缓存 */
    if (USART3->SR & USART_SR_RXNE)                                      // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if (cnt == UART3_RX_BUF_SIZE)                                    // 当前帧已接收的字节量，已满缓存区的大小; 为避免溢出，本包后面接收到的数据直接舍弃;
        {
            printf("警告：UART3单帧接收量，已超出接收缓存大小！\r修改文件bsp_UART.h的UART3_RX_BUF_SIZE值,可解决此问题！\r");
            USART3->DR;                                                  // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART3->DR ;                                     // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }

    /* 空闲中断：判断一帧数据结束，整理数据到外部备读 */
    if (USART3->SR & USART_SR_IDLE)                                      // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART3.puRxData, rxTemp, UART3_RX_BUF_SIZE);              // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART3.usRxNum  = cnt;                                           // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                         // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART3_RX_BUF_SIZE);                            // 接收数据缓存数组，清零; 准备下一次的接收
        USART3 ->SR;
        USART3 ->DR;                                                     // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： UART3_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* puData   需发送数据的地址
 *          uint8_t  usNum      发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART3_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                           // 把数据放入环形缓冲区
    {
        xUART3.puTxFiFoData[xUART3.usTxFiFoHead++] = puData[i];    // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART3.usTxFiFoHead == UART3_TX_BUF_SIZE)              // 如果指针位置到达缓冲区的最大值，则归0
            xUART3.usTxFiFoHead = 0;
    }

    if ((USART3->CR1 & USART_CR1_TXEIE) == 0)                      // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        USART3->CR1 |= USART_CR1_TXEIE;                            // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART3_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART3_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                              // 新建一个可变参数列表
    va_start(ap, pcString);                                  // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                   // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                              // 清空可变参数列表
    UART3_SendData((uint8_t *)mBuffer, strlen(mBuffer));     // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART3_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART3_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART3_ClearRx();                                              // 清0
    UART3_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART3_GetRxNum())                                     // 判断是否接收到数据
        {
            UART3_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART3_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART3_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART3_GetRxNum(void)
{
    return xUART3.usRxNum ;
}

/******************************************************************************
 * 函  数： UART3_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART3_GetRxData(void)
{
    return xUART3.puRxData ;
}

/******************************************************************************
 * 函  数： UART3_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART3_ClearRx(void)
{
    xUART3.usRxNum = 0 ;
}
#endif  // endif UART3_EN





//////////////////////////////////////////////////////////////   UART-4   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART4_EN

static xUSATR_TypeDef  xUART4 = { 0 };                      // 定义 UART4 的收发结构体
static uint8_t uaUart4RxData[UART4_RX_BUF_SIZE];            // 定义 UART4 的接收缓存
static uint8_t uaUart4TxFiFoData[UART4_TX_BUF_SIZE];        // 定义 UART4 的发送缓存

/******************************************************************************
 * 函  数： UART4_Init
 * 功  能： 初始化UART4的通信引脚、协议参数、中断优先级
 *          引脚：TX-PC10、RX-PC11
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART4_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    /* 使能 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           // 使能 GPIOC 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);           // 使能 UART4 时钟
    /* 配置 引脚复用功能 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);       // 配置PC10复用功能：UART4
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);       // 配置PC11复用功能：UART4
    /* 重置 UART */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);           // 使能重置
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);          // 取消重置
    /* 配置 TX引脚 */
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;                    // 引脚编号
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // 引脚方向: 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // 输出模式：推挽
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // 上下拉：上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // 输出速度：50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 RX引脚 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;                    // 引脚编号
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 UART */
    USART_InitTypeDef USART_InitStructure;                          // 声明USART初始化结构体
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // 设置波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 设置字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 设置一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(UART4, &USART_InitStructure);                        // 初始化USART
    /* 配置 中断 */
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);                   // 开启 接收中断
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);                   // 开启 空闲中断
    /* 配置 中断优先级 */
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // 声明 中断优先级结构体
    NVIC_InitStructure .NVIC_IRQChannel = UART4_IRQn;               // 指定中断通道
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // 设置抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // 设置响应优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 // 初始化NVIC
    /* 配置完成，使能UART */
    USART_Cmd(UART4, ENABLE);                                       // 使能USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL库 配置
    /* 使能 时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();                                   // 使能GPIOC
    __HAL_RCC_UART4_CLK_ENABLE();                                   // 使能UART4
    /* 重置 UART */
    __HAL_RCC_UART4_FORCE_RESET();                                  // 使能重置
    __HAL_RCC_UART4_RELEASE_RESET();                                // 取消重置
    /* 配置 GPIO引脚 */
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;              // 引脚 TX-PC10、RX-PC11
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;                     // 引脚复用功能
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                         // 初始化引脚工作模式
    /* 计算波特率 */
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // 波特率公式计算; UART4挂载在APB1, 时钟为系统时钟的4分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                                // 整数部分
    fraction = (temp - mantissa) * 16;                              // 小数部分
    UART4 -> BRR  = mantissa << 4 | fraction;                       // 设置波特率
    /* 配置 UART */
    UART4 -> CR1  =   0;                                            // 清0
    UART4 -> CR1 |=   0x01 << 2;                                    // 接收使能[02]: 0=失能、1=使能
    UART4 -> CR1 |=   0x01 << 3;                                    // 发送使能[03]：0=失能、1=使能
    UART4 -> CR1 |=   0x00 << 9;                                    // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    UART4 -> CR1 |=   0x00 << 10;                                   // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    UART4 -> CR1 |=   0x00 << 12;                                   // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    UART4 -> CR2  =   0;                                            // 数据清0
    UART4 -> CR2 &= ~(0x03 << 12);                                  // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    UART4 -> CR3  =   0;                                            // 数据清0
    UART4 -> CR3 &= ~(0x01 << 6);                                   // DMA接收[6]: 0=禁止、1=使能
    UART4 -> CR3 &= ~(0x01 << 7);                                   // DMA发送[7]: 0=禁止、1=使能
    /* 配置 中断 */
    UART4 -> CR1 &= ~(0x01 << 7);                                   // 关闭发送中断
    UART4 -> CR1 |=   0x01 << 5;                                    // 使能接收中断: 接收缓冲区非空
    UART4 -> CR1 |=   0x01 << 4;                                    // 使能空闲中断：超过1字节时间没收到新数据
    UART4 -> SR   = ~(0x00F0);                                      // 清理中断
    /* 配置 中断优选级 */
    HAL_NVIC_SetPriority(UART4_IRQn, 1, 1);                         // 设置指定中断的响应优先级;  参数：中断请求编号、抢占级、子优先级
    HAL_NVIC_EnableIRQ(UART4_IRQn);                                 // 使能、启用指定的中断
    /* 配置完成，打开串口 */
    UART4 -> CR1 |=   0x01 << 13;                                   // 使能UART开始工作
#endif
    /* 关联缓冲区 */
    xUART4.puRxData = uaUart4RxData;                                // 获取接收缓冲区的地址
    xUART4.puTxFiFoData = uaUart4TxFiFoData;                        // 获取发送缓冲区的地址
    /* 输出提示 */
    printf("UART4 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： UART4_IRQHandler
 * 功  能： UART4的中断处理函数
 *          接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
void UART4_IRQHandler(void)
{
    static uint16_t cnt = 0;                                           // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART4_RX_BUF_SIZE];                         // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到外部缓存：xUARTx.puRxData[ ]
    /* 发送中断：把环形缓冲的数据，逐字节发出 */
    if ((UART4->SR & USART_SR_TXE) && (UART4->CR1 & USART_CR1_TXEIE))  // 检查发送寄存器空中断使能，且发送寄存器为空; TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        UART4->DR = xUART4.puTxFiFoData[xUART4.usTxFiFoTail++];        // 从FIFO队列中取出一个数据，放入USART的发送寄存器(硬件会自动发出)，然后将FIFO的尾指针递增，指向下一个要发送的数据
        if (xUART4.usTxFiFoTail == UART4_TX_BUF_SIZE)                  // 检查FIFO尾指针是否到达了FIFO队列的末尾
            xUART4.usTxFiFoTail = 0;                                   // 将尾指针重置为0，实现环形队列的功能
        if (xUART4.usTxFiFoTail == xUART4.usTxFiFoHead)                // 检查FIFO尾指针是否追上了头指针，即所有数据是否都已发送完毕
            UART4->CR1 &= ~USART_CR1_TXEIE;                            // 关闭发送寄存器空中断，防止中断服务程序被不必要地调用
        return;
    }
    /* 接收中断：逐个字节接收，存放到临时缓存 */
    if (UART4->SR & USART_SR_RXNE)                                     // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if (cnt == UART4_RX_BUF_SIZE)                                  // 当前帧已接收的字节量，已满缓存区的大小; 为避免溢出，本包后面接收到的数据直接舍弃;
        {
            printf("警告：UART4单帧接收量，已超出接收缓存大小！\r修改文件bsp_UART.h的UART4_RX_BUF_SIZE值,可解决此问题！\r");
            UART4->DR;                                                 // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = UART4->DR ;                                    // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }
    /* 空闲中断：判断一帧数据结束，整理数据到外部备读 */
    if (UART4->SR & USART_SR_IDLE)                                     // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART4.puRxData, rxTemp, UART4_RX_BUF_SIZE);            // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART4.usRxNum  = cnt;                                         // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                       // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART4_RX_BUF_SIZE);                          // 接收数据缓存数组，清零; 准备下一次的接收
        UART4 ->SR;
        UART4 ->DR;                                                    // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： UART4_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* puData   需发送数据的地址
 *          uint8_t  usNum    发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART4_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART4.puTxFiFoData[xUART4.usTxFiFoHead++] = puData[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART4.usTxFiFoHead == UART4_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART4.usTxFiFoHead = 0;
    }

    if ((UART4->CR1 & USART_CR1_TXEIE) == 0)                     // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        UART4->CR1 |= USART_CR1_TXEIE;                           // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART4_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART4_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                                // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                              // 新建一个可变参数列表
    va_start(ap, pcString);                                  // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                   // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                              // 清空可变参数列表
    UART4_SendData((uint8_t *)mBuffer, strlen(mBuffer));     // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART4_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART4_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART4_ClearRx();                                              // 清0
    UART4_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART4_GetRxNum())                                     // 判断是否接收到数据
        {
            UART4_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART4_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART4_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART4_GetRxNum(void)
{
    return xUART4.usRxNum ;
}

/******************************************************************************
 * 函  数： UART4_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART4_GetRxData(void)
{
    return xUART4.puRxData ;
}

/******************************************************************************
 * 函  数： UART4_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART4_ClearRx(void)
{
    xUART4.usRxNum = 0 ;
}
#endif  // endif UART4_EN




//////////////////////////////////////////////////////////////   UART-5   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART5_EN

static xUSATR_TypeDef  xUART5 = { 0 };                      // 定义 UART5 的收发结构体
static uint8_t uaUart5RxData[UART5_RX_BUF_SIZE];            // 定义 UART5 的接收缓存
static uint8_t uaUart5TxFiFoData[UART5_TX_BUF_SIZE];        // 定义 UART5 的发送缓存

/******************************************************************************
 * 函  数： UART5_Init
 * 功  能： 初始化UART5的通信引脚、协议参数、中断优先级
 *          引脚：TX-PC12、RX-PD2
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART5_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    /* 使能 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           // 使能 GPIOC 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);           // 使能 GPIOD 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);           // 使能 UART5 时钟
    /* 配置 引脚复用功能 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);       // 配置PC12复用功能：UART5
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);        // 配置PD2复用功能 ：UART5
    /* 重置 UART */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);           // 使能重置
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);          // 取消重置
    /* 配置 TX引脚 */
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;                    // 引脚编号
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // 引脚方向: 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // 输出模式：推挽
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // 上下拉：上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // 输出速度：50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 RX引脚 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;                     // 引脚编号
    GPIO_Init(GPIOD, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 UART */
    USART_InitTypeDef USART_InitStructure;                          // 声明USART初始化结构体
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // 设置波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 设置字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 设置一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(UART5, &USART_InitStructure);                        // 初始化USART
    /* 配置 中断 */
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);                   // 开启 接收中断
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);                   // 开启 空闲中断
    /* 配置 中断优先级 */
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // 声明 中断优先级结构体
    NVIC_InitStructure .NVIC_IRQChannel = UART5_IRQn;               // 指定中断通道
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // 设置抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // 设置响应优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 // 初始化NVIC
    /* 配置完成，使能UART */
    USART_Cmd(UART5, ENABLE);                                       // 使能USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL库 配置
    /* 使能 时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();                                   // 使能GPIOC
    __HAL_RCC_GPIOD_CLK_ENABLE();                                   // 使能GPIOD
    __HAL_RCC_UART5_CLK_ENABLE();                                   // 使能UART5
    // 重置 UART
    __HAL_RCC_UART5_FORCE_RESET();                                  // 使能重置
    __HAL_RCC_UART5_RELEASE_RESET();                                // 取消重置
    /* 配置 TX引脚 */
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_12 ;                           // 引脚 TX-PC12
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;                     // 引脚复用功能
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                         // 初始化引脚
    /* 配置 RX引脚 */
    GPIO_InitStruct.Pin   = GPIO_PIN_2 ;                            // 引脚 RX-PD2
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);                         // 初始化引脚工作模式
    /* 计算波特率 */
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 4) / (ulBaudrate * 16);        // 波特率公式计算; UART5挂载在APB1, 时钟为系统时钟的4分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                                // 整数部分
    fraction = (temp - mantissa) * 16;                              // 小数部分
    UART5 -> BRR  = mantissa << 4 | fraction;                       // 设置波特率
    /* 配置 UART */
    UART5 -> CR1  =   0;                                            // 清0
    UART5 -> CR1 |=   0x01 << 2;                                    // 接收使能[02]: 0=失能、1=使能
    UART5 -> CR1 |=   0x01 << 3;                                    // 发送使能[03]：0=失能、1=使能
    UART5 -> CR1 |=   0x00 << 9;                                    // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    UART5 -> CR1 |=   0x00 << 10;                                   // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    UART5 -> CR1 |=   0x00 << 12;                                   // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    UART5 -> CR2  =   0;                                            // 数据清0
    UART5 -> CR2 &= ~(0x03 << 12);                                  // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    UART5 -> CR3  =   0;                                            // 数据清0
    UART5 -> CR3 &= ~(0x01 << 6);                                   // DMA接收[6]: 0=禁止、1=使能
    UART5 -> CR3 &= ~(0x01 << 7);                                   // DMA发送[7]: 0=禁止、1=使能
    /* 配置 中断 */
    UART5 -> CR1 &= ~(0x01 << 7);                                   // 关闭发送中断
    UART5 -> CR1 |=   0x01 << 5;                                    // 使能接收中断: 接收缓冲区非空
    UART5 -> CR1 |=   0x01 << 4;                                    // 使能空闲中断：超过1字节时间没收到新数据
    UART5 -> SR   = ~(0x00F0);                                      // 清理中断
    /* 配置 中断优先级 */
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);                         // 设置指定中断的响应优先级;  参数：中断请求编号、抢占级、子优先级
    HAL_NVIC_EnableIRQ(UART5_IRQn);                                 // 使能、启用指定的中断
    /* 配置完成，打开串口 */
    UART5 -> CR1 |=   0x01 << 13;                                   // 使能UART开始工作
#endif

    /* 关联缓冲区 */
    xUART5.puRxData = uaUart5RxData;                                // 获取接收缓冲区的地址
    xUART5.puTxFiFoData = uaUart5TxFiFoData;                        // 获取发送缓冲区的地址
    /* 输出提示 */
    printf("UART5 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： UART5_IRQHandler
 * 功  能： UART5的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
void UART5_IRQHandler(void)
{
    static uint16_t cnt = 0;                                           // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART5_RX_BUF_SIZE];                         // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到外部缓存：xUARTx.puRxData[ ]

    /* 发送中断：把环形缓冲的数据，逐字节发出 */
    if ((UART5->SR & USART_SR_TXE) && (UART5->CR1 & USART_CR1_TXEIE))  // 检查发送寄存器空中断使能，且发送寄存器为空; TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        UART5->DR = xUART5.puTxFiFoData[xUART5.usTxFiFoTail++];        // 从FIFO队列中取出一个数据，放入USART的发送寄存器(硬件会自动发出)，然后将FIFO的尾指针递增，指向下一个要发送的数据
        if (xUART5.usTxFiFoTail == UART5_TX_BUF_SIZE)                  // 检查FIFO尾指针是否到达了FIFO队列的末尾
            xUART5.usTxFiFoTail = 0;                                   // 将尾指针重置为0，实现环形队列的功能
        if (xUART5.usTxFiFoTail == xUART5.usTxFiFoHead)                // 检查FIFO尾指针是否追上了头指针，即所有数据是否都已发送完毕
            UART5->CR1 &= ~USART_CR1_TXEIE;                            // 关闭发送寄存器空中断，防止中断服务程序被不必要地调用
        return;
    }

    /* 接收中断：逐个字节接收，存放到临时缓存 */
    if (UART5->SR & USART_SR_RXNE)                                     // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if (cnt == UART5_RX_BUF_SIZE)                                  // 当前帧已接收的字节量，已满缓存区的大小; 为避免溢出，本包后面接收到的数据直接舍弃;
        {
            printf("警告：UART5单帧接收量，已超出接收缓存大小！\r修改文件bsp_UART.h的UART5_RX_BUF_SIZE值,可解决此问题！\r");
            UART5->DR;                                                 // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = UART5->DR ;                                    // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }

    /* 空闲中断：判断一帧数据结束，整理数据到外部备读 */
    if (UART5->SR & USART_SR_IDLE)                                     // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART5.puRxData, rxTemp, UART5_RX_BUF_SIZE);            // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART5.usRxNum  = cnt;                                         // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                       // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART5_RX_BUF_SIZE);                          // 接收数据缓存数组，清零; 准备下一次的接收
        UART5 -> SR;
        UART5 -> DR;                                                   // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}

/******************************************************************************
 * 函  数： UART5_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t* pudata     需发送数据的地址
 *          uint8_t  usNum      发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART5_SendData(uint8_t *pudata, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                         // 把数据放入环形缓冲区
    {
        xUART5.puTxFiFoData[xUART5.usTxFiFoHead++] = pudata[i];  // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART5.usTxFiFoHead == UART5_TX_BUF_SIZE)            // 如果指针位置到达缓冲区的最大值，则归0
            xUART5.usTxFiFoHead = 0;
    }

    if ((UART5->CR1 & USART_CR1_TXEIE) == 0)                     // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        UART5->CR1 |= USART_CR1_TXEIE;                           // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART5_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART5_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                             // 新建一个可变参数列表
    va_start(ap, pcString);                                 // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                  // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                             // 清空可变参数列表
    UART5_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART5_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART5_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART5_ClearRx();                                              // 清0
    UART5_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART5_GetRxNum())                                     // 判断是否接收到数据
        {
            UART5_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART5_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART5_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART5_GetRxNum(void)
{
    return xUART5.usRxNum ;
}

/******************************************************************************
 * 函  数： UART5_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART5_GetRxData(void)
{
    return xUART5.puRxData ;
}

/******************************************************************************
 * 函  数： UART5_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART5_ClearRx(void)
{
    xUART5.usRxNum = 0 ;
}
#endif  // endif UART5_EN




//////////////////////////////////////////////////////////////   USART-6   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if UART6_EN

static xUSATR_TypeDef  xUART6 = { 0 };                      // 定义 UART6 的收发结构体
static uint8_t uaUart6RxData[UART6_RX_BUF_SIZE];            // 定义 UART6 的接收缓存
static uint8_t uaUart6TxFiFoData[UART6_TX_BUF_SIZE];        // 定义 UART6 的发送缓存

/******************************************************************************
 * 函  数： UART6_Init
 * 功  能： 初始化USART6的通信引脚、协议参数、中断优先级
 *          引脚：TX-PC6、RX-PC7
 *          协议：波特率-None-8-1
 *          发送：发送中断
 *          接收：接收+空闲中断
 *
 * 参  数： uint32_t ulBaudrate  通信波特率
 * 返回值： 无
 ******************************************************************************/
void UART6_Init(uint32_t ulBaudrate)
{
#ifdef USE_STDPERIPH_DRIVER
    /* 使能 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);           // 使能 GPIOC  时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);          // 使能 USART6 时钟
    /* 配置 引脚复用功能 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);       // 配置PC6复用功能：USART6
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);       // 配置PC7复用功能：USART6
    /* 重置 UART */
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);          // 使能重置
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);         // 取消重置
    /* 配置 TX引脚 */
    GPIO_InitTypeDef  GPIO_InitStructure = {0};                     // GPIO 初始化结构体
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;                     // 引脚编号
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                   // 引脚方向: 复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  // 输出模式：推挽
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                   // 上下拉：上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               // 输出速度：50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 RX引脚 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;                     // 引脚编号
    GPIO_Init(GPIOC, &GPIO_InitStructure);                          // 初始化：把上述参数，更新到芯片寄存器
    /* 配置 UART */
    USART_InitTypeDef USART_InitStructure;                          // 声明USART初始化结构体
    USART_InitStructure.USART_BaudRate = ulBaudrate;                // 设置波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // 设置字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;          // 设置一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_Init(USART6, &USART_InitStructure);                       // 初始化USART
    /* 配置 中断 */
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);                  // 开启 接收中断
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);                  // 开启 空闲中断
    /* 配置 中断优先级 */
    NVIC_InitTypeDef  NVIC_InitStructure = {0};                     // 声明 中断优先级结构体
    NVIC_InitStructure .NVIC_IRQChannel = USART6_IRQn;              // 指定中断通道
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority = 1;      // 设置抢占优先级
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 1;             // 设置响应优先级
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);                                 // 初始化NVIC
    /* 配置完成，使能UART */
    USART_Cmd(USART6, ENABLE);                                      // 使能USART
#endif

#ifdef USE_HAL_DRIVER                                               // HAL库 配置
    /* 使能 时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();                                   // 使能GPIOC
    __HAL_RCC_USART6_CLK_ENABLE();                                  // 使能UART6
    /* 重置 UART5 */
    __HAL_RCC_USART6_FORCE_RESET();                                 // 使能重置
    __HAL_RCC_USART6_RELEASE_RESET();                               // 取消重置
    /* 配置 GPIO引脚 */
    GPIO_InitTypeDef    GPIO_InitStruct = {0};                      // 声明初始化要用到的结构体
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;                // 引脚 TX-PC6、RX-PC7
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;                        // 工作模式
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                            // 上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;              // 引脚速率
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;                    // 引脚复用功能
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);                         // 初始化引脚工作模式
    /* 计算波特率 */
    float    temp;
    uint16_t mantissa, fraction;
    SystemCoreClockUpdate();                                        // 更新系统运行频率全局值; 函数SystemCoreClock( )，在标准库、HAL库通用
    temp = (float)(SystemCoreClock / 2) / (ulBaudrate * 16);        // 波特率公式计算; USART6挂载在APB2, 时钟为系统时钟的2分频; 全局变量SystemCoreClock，在标准库、HAL库通用;
    mantissa = temp;				                                // 整数部分
    fraction = (temp - mantissa) * 16;                              // 小数部分
    USART6 -> BRR  = mantissa << 4 | fraction;                      // 设置波特率
    /* 配置 UART */
    USART6 -> CR1  =   0;                                           // 清0
    USART6 -> CR1 |=   0x01 << 2;                                   // 接收使能[02]: 0=失能、1=使能
    USART6 -> CR1 |=   0x01 << 3;                                   // 发送使能[03]：0=失能、1=使能
    USART6 -> CR1 |=   0x00 << 9;                                   // 奇偶校验[09]：0=偶Even、1=奇Odd;  注意：使用奇偶校验，下面两项要置1
    USART6 -> CR1 |=   0x00 << 10;                                  // 校验位  [10]：0=禁用、1=使能;     注意，使用奇偶校验，此位要置1 (否则无效、数据错乱)
    USART6 -> CR1 |=   0x00 << 12;                                  // 数据位  [12]：0=8位、 1=9位;      注意：使用奇偶校验，此位要置1 (否则数据会发生错乱)
    USART6 -> CR2  =   0;                                           // 数据清0
    USART6 -> CR2 &= ~(0x03 << 12);                                 // 停止位[13:12]：00b=1个停止位、01b=0.5个停止位、10b=2个停止位、11b=1.5个停止位
    USART6 -> CR3  =   0;                                           // 数据清0
    USART6 -> CR3 &= ~(0x01 << 6);                                  // DMA接收[6]: 0=禁止、1=使能
    USART6 -> CR3 &= ~(0x01 << 7);                                  // DMA发送[7]: 0=禁止、1=使能
    /* 配置 中断 */
    USART6 -> CR1 &= ~(0x01 << 7);                                  // 关闭发送中断
    USART6 -> CR1 |=   0x01 << 5;                                   // 使能接收中断: 接收缓冲区非空
    USART6 -> CR1 |=   0x01 << 4;                                   // 使能空闲中断：超过1字节时间没收到新数据
    USART6 -> SR   = ~(0x00F0);                                     // 清理中断
    /* 配置 中断优先级 */
    HAL_NVIC_SetPriority(USART6_IRQn, 1, 1);                        // 设置指定中断的响应优先级;  参数：中断请求编号、抢占级、子优先级
    HAL_NVIC_EnableIRQ(USART6_IRQn);                                // 使能、启用指定的中断
    /* 配置完成，打开串口 */
    USART6 -> CR1 |=   0x01 << 13;                                  // 使能UART开始工作
#endif

    /* 关联缓冲区 */
    xUART6.puRxData = uaUart6RxData;                                // 获取接收缓冲区的地址
    xUART6.puTxFiFoData = uaUart6TxFiFoData;                        // 获取发送缓冲区的地址
    /* 输出提示 */
    printf("UART6 初始化配置         %d-None-8-1; 已完成初始化配置、收发配置\r", ulBaudrate);
}

/******************************************************************************
 * 函  数： USART6_IRQHandler
 * 功  能： USART6的接收中断、空闲中断、发送中断
 * 参  数： 无
 * 返回值： 无
 * 备  注： 本函数，当产生中断事件时，由硬件调用。
 *          如果使用本文件代码，在工程文件的其它地方，要注释同名函数，否则冲突。
 ******************************************************************************/
void USART6_IRQHandler(void)
{
    static uint16_t cnt = 0;                                             // 接收字节数累计：每一帧数据已接收到的字节数
    static uint8_t  rxTemp[UART6_RX_BUF_SIZE];                           // 接收数据缓存数组：每新接收１个字节，先顺序存放到这里，当一帧接收完(发生空闲中断), 再转存到全局变量：xUARTx.puRxData[xx]中；

    /* 发送中断：把环形缓冲的数据，逐字节发出 */
    if ((USART6->SR & USART_SR_TXE) && (USART6->CR1 & USART_CR1_TXEIE))  // 检查发送寄存器空中断使能，且发送寄存器为空; TXE(发送数据寄存器空)、TXEIE(发送缓冲区空中断使能)
    {
        USART6->DR = xUART6.puTxFiFoData[xUART6.usTxFiFoTail++];         // 从FIFO队列中取出一个数据，放入USART的发送寄存器(硬件会自动发出)，然后将FIFO的尾指针递增，指向下一个要发送的数据
        if (xUART6.usTxFiFoTail == UART6_TX_BUF_SIZE)                    // 检查FIFO尾指针是否到达了FIFO队列的末尾
            xUART6.usTxFiFoTail = 0;                                     // 将尾指针重置为0，实现环形队列的功能
        if (xUART6.usTxFiFoTail == xUART6.usTxFiFoHead)                  // 检查FIFO尾指针是否追上了头指针，即所有数据是否都已发送完毕
            USART6->CR1 &= ~USART_CR1_TXEIE;                             // 关闭发送寄存器空中断，防止中断服务程序被不必要地调用
        return;
    }

    /* 接收中断：逐个字节接收，存放到临时缓存 */
    if (USART6->SR & USART_SR_RXNE)                                      // 检查RXNE(读数据寄存器非空标志位); RXNE中断清理方法：读DR时自动清理；
    {
        if (cnt >= UART6_RX_BUF_SIZE)                                    // 当前帧已接收的字节量，已满缓存区的大小; 为避免溢出，本包后面接收到的数据直接舍弃;
        {
            printf("警告：USART6单帧接收量，已超出接收缓存大小！\r修改文件bsp_UART.h的UART6_RX_BUF_SIZE值,可解决此问题！\r");
            USART6->DR;                                                  // 读取数据寄存器的数据，但不保存．主要作用：读DR时自动清理接收中断标志；
            return;
        }
        rxTemp[cnt++] = USART6->DR ;                                     // 把新收到的字节数据，顺序存放到RXTemp数组中；注意：读取DR时自动清零中断位
        return;
    }

    /* 空闲中断：判断一帧数据结束，整理数据到外部备读 */
    if (USART6->SR & USART_SR_IDLE)                                      // 检查IDLE(空闲中断标志位); IDLE中断标志清理方法：序列清零，USART1 ->SR;  USART1 ->DR;
    {
        memcpy(xUART6.puRxData, rxTemp, UART6_RX_BUF_SIZE);              // 把本帧接收到的数据，存入到结构体的数组成员xUARTx.puRxData中, 等待处理; 注意：复制的是整个数组，包括0值，以方便字符串输出时尾部以0作字符串结束符
        xUART6.usRxNum  = cnt;                                           // 把接收到的字节数，存入到结构体变量xUARTx.usRxNum中；
        cnt = 0;                                                         // 接收字节数累计器，清零; 准备下一次的接收
        memset(rxTemp, 0, UART6_RX_BUF_SIZE);                            // 接收数据缓存数组，清零; 准备下一次的接收
        USART6 ->SR;
        USART6 ->DR;                                                     // 清零IDLE中断标志位!! 序列清零，顺序不能错!!
        return;
    }

    return;
}


/******************************************************************************
 * 函  数： UART6_SendData
 * 功  能： UART通过中断发送数据
 *         【适合场景】本函数可发送各种数据，而不限于字符串，如int,char
 *         【不 适 合】注意h文件中所定义的发缓冲区大小、注意数据压入缓冲区的速度与串口发送速度的冲突
 * 参  数： uint8_t  *puData   需发送数据的地址
 *          uint8_t   usNum    发送的字节数 ，数量受限于h文件中设置的发送缓存区大小宏定义
 * 返回值： 无
 ******************************************************************************/
void UART6_SendData(uint8_t *puData, uint16_t usNum)
{
    for (uint16_t i = 0; i < usNum; i++)                           // 把数据放入环形缓冲区
    {
        xUART6.puTxFiFoData[xUART6.usTxFiFoHead++] = puData[i];    // 把字节放到缓冲区最后的位置，然后指针后移
        if (xUART6.usTxFiFoHead == UART6_TX_BUF_SIZE)              // 如果指针位置到达缓冲区的最大值，则归0
            xUART6.usTxFiFoHead = 0;
    }

    if ((USART6->CR1 & USART_CR1_TXEIE) == 0)                      // 检查USART寄存器的发送缓冲区空置中断(TXEIE)是否已打开
        USART6->CR1 |= USART_CR1_TXEIE;                            // 打开TXEIE中断
}

/******************************************************************************
 * 函  数： UART6_SendString
 * 功  能： 发送字符串
 *          用法请参考printf，及示例中的展示
 *          注意，最大发送字节数为512-1个字符，可在函数中修改上限
 * 参  数： const char *pcString, ...   (如同printf的用法)
 * 返回值： 无
 ******************************************************************************/
void UART6_SendString(const char *pcString, ...)
{
    char mBuffer[512] = {0};;                               // 开辟一个缓存, 并把里面的数据置0
    va_list ap;                                             // 新建一个可变参数列表
    va_start(ap, pcString);                                 // 列表指向第一个可变参数
    vsnprintf(mBuffer, 512, pcString, ap);                  // 把所有参数，按格式，输出到缓存; 参数2用于限制发送的最大字节数，如果达到上限，则只发送上限值-1; 最后1字节自动置'\0'
    va_end(ap);                                             // 清空可变参数列表
    UART6_SendData((uint8_t *)mBuffer, strlen(mBuffer));    // 把字节存放环形缓冲，排队准备发送
}

/******************************************************************************
 * 函    数： UART6_SendAT
 * 功    能： 发送AT命令, 并等待返回信息
 * 参    数： char     *pcString      AT指令字符串
 *            char     *pcAckString   期待的指令返回信息字符串
 *            uint16_t  usTimeOut     发送命令后等待的时间，毫秒
 *
 * 返 回 值： 0-执行失败、1-执行正常
 ******************************************************************************/
uint8_t UART6_SendAT(char *pcAT, char *pcAckString, uint16_t usTimeOutMs)
{
    UART6_ClearRx();                                              // 清0
    UART6_SendString(pcAT);                                       // 发送AT指令字符串

    while (usTimeOutMs--)                                         // 判断是否起时(这里只作简单的循环判断次数处理）
    {
        if (UART6_GetRxNum())                                     // 判断是否接收到数据
        {
            UART6_ClearRx();                                      // 清0接收字节数; 注意：接收到的数据内容 ，是没有被清0的
            if (strstr((char *)UART6_GetRxData(), pcAckString))   // 判断返回数据中是否有期待的字符
                return 1;                                         // 返回：0-超时没有返回、1-正常返回期待值
        }
        delay_ms(1);                                              // 延时; 用于超时退出处理，避免死等
    }
    return 0;                                                     // 返回：0-超时、返回异常，1-正常返回期待值
}

/******************************************************************************
 * 函  数： UART6_GetRxNum
 * 功  能： 获取最新一帧数据的字节数
 * 参  数： 无
 * 返回值： 0=没有接收到数据，非0=新一帧数据的字节数
 ******************************************************************************/
uint16_t UART6_GetRxNum(void)
{
    return xUART6.usRxNum ;
}

/******************************************************************************
 * 函  数： UART6_GetRxData
 * 功  能： 获取最新一帧数据 (数据的地址）
 * 参  数： 无
 * 返回值： 数据的地址(uint8_t*)
 ******************************************************************************/
uint8_t *UART6_GetRxData(void)
{
    return xUART6.puRxData ;
}

/******************************************************************************
 * 函  数： UART6_ClearRx
 * 功  能： 清理最后一帧数据的缓存
 *          主要是清0字节数，因为它是用来判断接收的标准
 * 参  数： 无
 * 返回值： 无
 ******************************************************************************/
void UART6_ClearRx(void)
{
    xUART6.usRxNum = 0 ;
}
#endif  // endif UART6_EN






/////////////////////////////////////////////////////////  ModbusCRC16 校验   /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CRC16查表法值表数组
// 查表法相比直接计算CRC值，大约可减少一半运算时间
static const uint8_t aModbusCRC16[] =
{
    // 高位
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    // 低位
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};



/**********************************************************************************************************
 * 函 数:  Modbus_GetCRC16
 * 功 能:  返回ModbusCRC16校验值
 * 备 注：
 *
 * 参 数:  uint8_t*  _pcData：参与计算校验值的数据
 *         uint16_t  _usLen ：数据字节数; 长度为数据的原字节数即可，不用加2;
 *
 * 返 回:  uint16  CRC16值，高位在前，低位在后
**********************************************************************************************************/
uint16_t  Modbus_GetCRC16(uint8_t *_pcData, uint16_t _usLen)
{
    uint8_t  ucCRCHi = 0xFF;                        // CRC值高位
    uint8_t  ucCRCLo = 0xFF;                        // CRC值低位
    uint16_t usIndex;                               // 索引

    while (_usLen--)                                // 开始逐字节查表
    {
        usIndex = ucCRCHi ^ *_pcData++;             // 注意，这里指针地址加1了
        ucCRCHi = ucCRCLo ^ aModbusCRC16[usIndex];
        ucCRCLo = aModbusCRC16[usIndex + 256];
    }
    //_pcData[0] = ucCRCHi;                         // 末尾第1字节，追加CRC16的高位
    //_pcData[1] = ucCRCLo;                         // 末尾第2字节，追加CRC16的低位
    return ucCRCHi << 8 | ucCRCLo ;                 // 返回CRC16值，高位在前，低位在后
}



/**********************************************************************************************************
 * 函 数:  Modbus_CheckCRC16
 * 功 能:  对带ModbusCRC16校验的数据段进行校验
 *
 * 参 数:  uint8_t*  _pcData：数据段地址
 *         uint16_t  _usLen ：数据段的长度(字节数); 长度是整包数据的字节数，即包含ModbusCRC16值的长度，不用减2;
 *
 * 返 回:  0-错误、1-正确
*******************************************************************************************/
uint8_t  Modbus_CheckCRC16(uint8_t *_pcData, uint16_t _usLen)
{
    uint8_t  ucCRCHi = 0xFF;                               // CRC值高位
    uint8_t  ucCRCLo = 0xFF;                               // CRC值低位
    uint16_t usIndex;                                      // 数组的索引

    _usLen -= 2;                                           // 字节数-2，不计算原数据末尾两字节(ModbusCRC16值)

    while (_usLen--)                                       // 开始逐字节查表获得ModbusCRC16值
    {
        usIndex = ucCRCHi ^ *_pcData++;                    // 注意，这里指针地址递增加1
        ucCRCHi = ucCRCLo ^ aModbusCRC16[usIndex];
        ucCRCLo = aModbusCRC16[usIndex + 256];
    }

    if ((ucCRCHi == *_pcData++) && (ucCRCLo == *_pcData))  // 把数据段的CRC16校验值，与原数据末尾的CRC16值相比较
        return 1;                                          // 成功匹配; 返回: 1

    return 0;                                              // 错误; 返回：0
}





/////////////////////////////////////////////////////////////  辅助功能   /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * 函  数： debug_showData
 * 功  能： 经printf，发送到串口助手上，方便观察
 * 参  数： uint8_t  *rxData   数据地址
 *          uint16_t  rxNum    字节数
 * 返回值： 无
 ******************************************************************************/
#if 0
void debug_showData(uint8_t *puRxData, uint16_t usRxNum)
{
    printf("字节数： %d \r", usRxNum);                   // 显示字节数
    printf("ASCII 显示数据: %s\r", (char *)puRxData);    // 显示数据，以ASCII方式显示，即以字符串的方式显示
    printf("16进制显示数据: ");                          // 显示数据，以16进制方式，显示每一个字节的值
    while (usRxNum--)                                    // 逐个字节判断，只要不为'\0', 就继续
        printf("0x%X ", *puRxData++);                    // 格式化
    printf("\r\r");                                      // 显示换行
}
#endif




//////////////////////////////////////////////////////////////  printf   //////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * 功  能： 实现 printf 串口重定向
 * 说  明： 1. 兼容 GCC（CubeIDE）与 ARMCC（Keil）编译器。
 *          2. 支持 标准库 和 HAL库
 *          3. 无需勾选MicroLIB、设置软件环境参数
 *          4. 使用了阻塞式发送，避免与中断/DMA发送冲突，以求调试真实输出
 *          5. 若需使用其他串口，修改 USART1 为相应UART即可
 *          6. 只要初始化了UART1, 复制下方代码到你的工程，即可使用printf
 * 参  数：
 * 返回值：
 * 备  注： 最后修改_2026年03月17日
 ******************************************************************************/
#include <stdio.h>                                        // 包含 printf/fputc 头文件

#ifdef __GNUC__			                                  // 用于 CubeIDE (GCC) 环境;                                                          

#define PUTCHAR_FUNCTION    int __io_putchar(int ch)      // 通常需要实现 __io_putchar 函数，该函数被 _write 系统调用间接使用。若工程未实现 _write，需自行提供（一般只需实现 __io_putchar 即可）。
                                                        
#else                                                     // 用于 Keil (ARMCC) 环境
                                                         
#pragma import(__use_no_semihosting)                      // 关闭半主机模式
struct __FILE                                             // 标准库所需的文件结构体
{                                                        
    int handle;                                          
};                                                       
FILE __stdout;                                            // 定义标准输出流 (printf使用); FILE 在stdio.h文件
void _sys_exit(int x)                                     // 半主机模式下的 _sys_exit 函数必须重定义,此处为空实现
{                                                        
    (void) x;                                             // 空操作，避免未使用参数警告; 显式告诉编译器:我故意不用这个参数
}
#define PUTCHAR_FUNCTION    int fputc(int ch, FILE *f)

#endif

PUTCHAR_FUNCTION                                          // printf 重定向输出
{
    while ((USART1->SR & 0X40) == 0);                     // 等待USART1发送完成标志(TC:bit6)置位,即等待上一次数据发送完成
    USART1->DR = (ch & 0xFF);                             // 字符写入数据寄存器，启动发送; 只取低8位，兼容 9 位数据模式            
    return ch;
}




