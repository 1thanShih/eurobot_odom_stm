/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/*
 * ===== 中文說明 =====
 * HAL 的時間基準（HAL_GetTick() / HAL_Delay() 用的 1 ms 計時）。
 * 一般專案用 SysTick，但 FreeRTOS 要獨佔 SysTick 來切換 task，
 * 所以 CubeMX 把 HAL 的計時改到 TIM11（F411 沒有 TIM6），這個檔案就是在設定 TIM11。
 * TIM11 中斷的路徑：TIM1_TRG_COM_TIM11_IRQHandler()（stm32f4xx_it.c）
 *   → HAL_TIM_PeriodElapsedCallback()（main.c）→ HAL_IncTick()。
 *
 * 函式：
 *   HAL_InitTick()    依目前的 APB2 時脈算出 prescaler，讓 TIM11 以 1 MHz 計數、每 1 ms 中斷一次。
 *                     HAL_Init() 和每次改時脈時，HAL 會自動呼叫。
 *   HAL_SuspendTick() 關掉 TIM11 的更新中斷（暫停計時，例如進低功耗前）。
 *   HAL_ResumeTick()  重新開啟 TIM11 的更新中斷。
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim11;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM11 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0U;

  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  HAL_StatusTypeDef     status;

  /* Enable TIM11 clock */
  __HAL_RCC_TIM11_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM11 clock */
      uwTimclock = HAL_RCC_GetPCLK2Freq();

  /* Compute the prescaler value to have TIM11 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM11 */
  htim11.Instance = TIM11;

  /* Initialize TIMx peripheral as follow:
   * Period = [(TIM11CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  htim11.Init.Period = (1000000U / 1000U) - 1U;
  htim11.Init.Prescaler = uwPrescalerValue;
  htim11.Init.ClockDivision = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim11);
  if (status == HAL_OK)
  {

    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim11);
    if (status == HAL_OK)
    {
    /* Enable the TIM11 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
      /* Configure the SysTick IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Configure the TIM IRQ priority */
        HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM11 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM11 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM11 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM11 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE);
}

