/* 假的 HAL：只提供 odometry.c 用到的東西，讓它能在電腦上編譯 */
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { int dummy; } TIM_TypeDef;
extern TIM_TypeDef fake_tim[6];
#define TIM1 (&fake_tim[1])
#define TIM2 (&fake_tim[2])
#define TIM3 (&fake_tim[3])
#define TIM4 (&fake_tim[4])
#define TIM5 (&fake_tim[5])
typedef struct { TIM_TypeDef *Instance; uint32_t cnt; int started; } TIM_HandleTypeDef;
#define TIM_CHANNEL_ALL 0x3C
#define __HAL_TIM_GET_COUNTER(h) ((h)->cnt)
#define __HAL_TIM_SET_COUNTER(h, v) ((h)->cnt = (v))
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t ch);
uint32_t HAL_GetTick(void);
void Error_Handler(void);
