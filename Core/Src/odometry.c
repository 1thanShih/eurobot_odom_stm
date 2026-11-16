/**
 * @file    odometry.c
 * @brief   見 odometry.h。
 */
#include "odometry.h"

#include "main.h"
#include "robot_config.h"

/* CubeMX 在 main.c 產生的 timer handle */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

typedef struct {
    TIM_HandleTypeDef *htim;
    int8_t   direction;    /* +1 或 -1 */
    bool     is_32bit;     /* TIM2/TIM5 是 32-bit，其他 16-bit */
    uint32_t last_count;   /* 上一次讀到的 CNT */
    int32_t  total;        /* 累計 count */
} OdomPod;

/* 接線表：要和 .ioc 的腳位一致 */
static OdomPod pods[ODOM_POD_COUNT] = {
    [ODOM_POD_1] = { .htim = &htim2, .direction = ODOM_POD1_DIRECTION },  /* PA0 / PA1 */
    [ODOM_POD_2] = { .htim = &htim3, .direction = ODOM_POD2_DIRECTION },  /* PA6 / PA7 */
    [ODOM_POD_3] = { .htim = &htim1, .direction = ODOM_POD3_DIRECTION },  /* PA8 / PA9 */
    [ODOM_POD_4] = { .htim = &htim4, .direction = ODOM_POD4_DIRECTION },  /* PB6 / PB7 */
};

static uint32_t stamp_ms = 0;

int32_t odom_counter_delta(uint32_t now, uint32_t last, bool is_32bit)
{
    /* 用同樣寬度的無號數相減，繞回時結果自然正確 */
    if (is_32bit) {
        return (int32_t)(now - last);
    }
    return (int16_t)(uint16_t)(now - last);
}

void odom_init(void)
{
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        OdomPod *pod = &pods[i];

        pod->is_32bit = (pod->htim->Instance == TIM2) || (pod->htim->Instance == TIM5);
        if (HAL_TIM_Encoder_Start(pod->htim, TIM_CHANNEL_ALL) != HAL_OK) {
            Error_Handler();
        }
        __HAL_TIM_SET_COUNTER(pod->htim, 0);
        pod->last_count = 0;
        pod->total = 0;
    }
    stamp_ms = HAL_GetTick();
}

void odom_update(void)
{
    uint32_t now[ODOM_POD_COUNT];

    /* 先一口氣讀完 4 個，讓它們的時間點盡量一致 */
    stamp_ms = HAL_GetTick();
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        now[i] = __HAL_TIM_GET_COUNTER(pods[i].htim);
    }

    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        OdomPod *pod = &pods[i];
        pod->total += pod->direction *
                      odom_counter_delta(now[i], pod->last_count, pod->is_32bit);
        pod->last_count = now[i];
    }
}

void odom_get_snapshot(OdomSnapshot *out)
{
    out->stamp_ms = stamp_ms;
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        out->total[i] = pods[i].total;
    }
}
