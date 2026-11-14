/**
 * @file    odometry.c
 * @brief   Odometry pod encoder 讀取。說明見 odometry.h。
 */
#include "odometry.h"

#include "main.h"          /* HAL、Error_Handler() */
#include "robot_config.h"

/* CubeMX 在 main.c 產生的 timer handle */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

typedef struct {
    TIM_HandleTypeDef *htim;
    int8_t   direction;   /* +1 或 -1 */
    bool     is_32bit;    /* 在 odom_init() 自動判斷 */
    uint32_t last_count;
    int32_t  total;
    int32_t  delta;
} OdomPod;

/* 接線表：改接線時只需要改這裡（腳位要與 CubeMX .ioc 一致） */
static OdomPod pods[ODOM_POD_COUNT] = {
    [ODOM_POD_1] = { .htim = &htim2, .direction = ODOM_POD1_DIRECTION },  /* PA0 / PA1, 32-bit */
    [ODOM_POD_2] = { .htim = &htim3, .direction = ODOM_POD2_DIRECTION },  /* PA6 / PA7, 16-bit */
    [ODOM_POD_3] = { .htim = &htim1, .direction = ODOM_POD3_DIRECTION },  /* PA8 / PA9, 16-bit */
    [ODOM_POD_4] = { .htim = &htim4, .direction = ODOM_POD4_DIRECTION },  /* PB6 / PB7, 16-bit */
};

static uint32_t last_update_ms = 0;

static bool timer_is_32bit(const TIM_HandleTypeDef *htim)
{
    /* STM32F446 上只有 TIM2、TIM5 的計數器是 32-bit，其餘是 16-bit */
    return (htim->Instance == TIM2) || (htim->Instance == TIM5);
}

int32_t odom_counter_delta(uint32_t now, uint32_t last, bool is_32bit)
{
    /* 先用無號數相減（溢位時會自動繞回），再轉成有號數得到正負方向。
     * 例：16-bit 從 65530 走到 5，(uint16_t)(5 - 65530) = 11 → +11。
     * 前提是兩次讀值間移動量小於計數範圍的一半（16-bit 約 1.6 m），實務上不會超過。 */
    if (is_32bit) {
        return (int32_t)(now - last);
    }
    return (int16_t)(uint16_t)(now - last);
}

void odom_init(void)
{
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        OdomPod *pod = &pods[i];

        pod->is_32bit = timer_is_32bit(pod->htim);
        if (HAL_TIM_Encoder_Start(pod->htim, TIM_CHANNEL_ALL) != HAL_OK) {
            Error_Handler();
        }
        __HAL_TIM_SET_COUNTER(pod->htim, 0);
        pod->last_count = 0;
        pod->total = 0;
        pod->delta = 0;
    }
    last_update_ms = HAL_GetTick();
}

void odom_update(void)
{
    uint32_t now[ODOM_POD_COUNT];

    /* 先一次讀完所有計數器，讓各 pod 的取樣時間盡量一致，再做運算 */
    last_update_ms = HAL_GetTick();
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        now[i] = __HAL_TIM_GET_COUNTER(pods[i].htim);
    }

    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        OdomPod *pod = &pods[i];
        pod->delta = pod->direction *
                     odom_counter_delta(now[i], pod->last_count, pod->is_32bit);
        pod->total += pod->delta;
        pod->last_count = now[i];
    }
}

void odom_get_snapshot(OdomSnapshot *out)
{
    out->timestamp_ms = last_update_ms;
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        out->total[i] = pods[i].total;
        out->delta[i] = pods[i].delta;
    }
}

float odom_counts_to_mm(int32_t counts)
{
    return (float)counts * ODOM_MM_PER_COUNT;
}
