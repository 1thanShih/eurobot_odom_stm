/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
 * CubeMX 留給 FreeRTOS 應用程式碼的位置，例如 idle / tick hook、stack overflow hook，
 * 或是想放在這裡的額外 task。
 *
 * 唯一的 task（defaultTask）由 CubeMX 在 main.c 建立，不在這裡。
 * FreeRTOS 的設定（tick 1 kHz、heap 30000 bytes 等）在 Core/Inc/FreeRTOSConfig.h。
 *
 * 函式：
 *   vApplicationStackOverflowHook()  task 的 stack 爆掉時呼叫：記下 task 名稱、
 *                                    關中斷、LED2 快閃並停住。
 *   vApplicationMallocFailedHook()   heap 配置失敗時呼叫：只累計次數，不停機。
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 出錯時記在這裡，用 debugger 的 Live Expressions 觀察 */
const char *volatile rtos_overflow_task = NULL;  /* stack 爆掉的 task 名稱 */
volatile uint32_t    rtos_malloc_fail_count = 0; /* heap 配置失敗的次數 */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
/* FreeRTOS 切換 task 時發現 stack 爆掉會呼叫這裡。
 * 記憶體可能已經被破壞，不能繼續跑：關中斷、LED2 快閃，停在這裡。 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
    (void)xTask;
    rtos_overflow_task = (const char *)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {
        BSP_LED_Toggle(LED2);
        for (volatile uint32_t i = 0; i < 1000000U; i++) {
            /* 忙等，不依賴任何中斷或 RTOS */
        }
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
/* FreeRTOS 或 micro-ROS 的 heap 不夠時呼叫。
 * 呼叫端會拿到 NULL 並自己處理（micro-ROS 會重新連線），所以只記次數、不停機。
 * 這個數字不是 0，就表示 configTOTAL_HEAP_SIZE 該加大或有記憶體洩漏。 */
void vApplicationMallocFailedHook(void)
{
    rtos_malloc_fail_count++;
}
/* USER CODE END 5 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

