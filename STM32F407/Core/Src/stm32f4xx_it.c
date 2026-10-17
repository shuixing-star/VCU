/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Debugger snapshot. complete is written last; all addresses come from entry. */
typedef struct {
  uint32_t complete;
  uint32_t cfsr, hfsr, bfar, mmfar;
  uint32_t bfar_valid, mmfar_valid;
  uint32_t exc_return, frame_sp, core_frame_sp;
  uint32_t frame_readable, frame_plausible;
  uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
  uint32_t interrupted_exception;
} vcu_fault_snapshot_t;
volatile vcu_fault_snapshot_t g_vcu_fault;
/* A separate stack lets capture run even when the application stack is bad. */
__align(8) uint8_t g_vcu_fault_stack[512];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void vcu_fault_capture(uint32_t *frame, uint32_t exc_return);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
/* ARMCC5 assembly: preserve exception entry SP/LR before any C prologue. */
__asm void HardFault_Handler(void)
{
  IMPORT g_vcu_fault_stack
  IMPORT vcu_fault_capture
  PRESERVE8
  TST lr, #4
  ITE EQ
  MRSEQ r0, MSP
  MRSNE r0, PSP
  MOV r1, lr
  CPSID i
  LDR r2, =g_vcu_fault_stack
  ADD r2, r2, #512
  MSR MSP, r2
  B vcu_fault_capture
  ALIGN
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_3_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void vcu_fault_capture(uint32_t *frame, uint32_t exc_return)
{
  uint32_t core = (uint32_t)frame;
  uint32_t in_ram;
  const volatile uint32_t *saved;
  g_vcu_fault.complete = 0U;
  g_vcu_fault.cfsr = SCB->CFSR;
  g_vcu_fault.hfsr = SCB->HFSR;
  g_vcu_fault.bfar = SCB->BFAR;
  g_vcu_fault.mmfar = SCB->MMFAR;
  g_vcu_fault.bfar_valid = (SCB->CFSR & SCB_CFSR_BFARVALID_Msk) != 0U;
  g_vcu_fault.mmfar_valid = (SCB->CFSR & SCB_CFSR_MMARVALID_Msk) != 0U;
  g_vcu_fault.exc_return = exc_return;
  g_vcu_fault.frame_sp = core;
  g_vcu_fault.frame_readable = 0U;
  g_vcu_fault.frame_plausible = 0U;
  /* An extended floating-point exception frame precedes the core frame. */
  if ((exc_return & 0x10U) == 0U) core += 18U * sizeof(uint32_t);
  g_vcu_fault.core_frame_sp = core;
  in_ram = ((core >= 0x20000000U && core <= 0x2001FFE0U) ||
            (core >= 0x10000000U && core <= 0x1000FFE0U));
  if (in_ram && (core & 3U) == 0U &&
      (g_vcu_fault.cfsr & (SCB_CFSR_STKERR_Msk | SCB_CFSR_MSTKERR_Msk)) == 0U) {
    saved = (const volatile uint32_t *)core;
    g_vcu_fault.r0 = saved[0];
    g_vcu_fault.r1 = saved[1];
    g_vcu_fault.r2 = saved[2];
    g_vcu_fault.r3 = saved[3];
    g_vcu_fault.r12 = saved[4];
    g_vcu_fault.lr = saved[5];
    g_vcu_fault.pc = saved[6];
    g_vcu_fault.xpsr = saved[7];
    g_vcu_fault.interrupted_exception = saved[7] & 0x1FFU;
    g_vcu_fault.frame_readable = 1U;
    g_vcu_fault.frame_plausible =
        (saved[7] & (1U << 24U)) != 0U && (saved[7] & 0x1FFU) <= 97U;
  }
  __DSB();
  g_vcu_fault.complete = 0x4641554CU;
  while (1) {
    __NOP();
  }
}

/* No scheduler owns these exceptions. TIM7 alone supplies the HAL tick. */
void SVC_Handler(void)
{
  Error_Handler();
}

void PendSV_Handler(void)
{
  Error_Handler();
}

void SysTick_Handler(void)
{
  /* SysTick is not enabled in this bare-metal application. */
}


/* USER CODE END 1 */
