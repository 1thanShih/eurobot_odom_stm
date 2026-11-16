/**
 * @file    odometry.c
 * @brief   讀 4 個 odometry pod 的 encoder，累計成開機以來的總 count。
 *
 * 每個 pod 接一個設成 Encoder mode 的 Timer，A/B 相的加減計數由硬體自己完成。
 * 這個檔案只負責：定期讀 Timer 的 CNT、處理計數器繞回（溢位）、
 * 乘上方向（+1/-1）後累加到 32-bit 的 total。
 * 對外介面見 odometry.h，接線與方向設定見 robot_config.h。
 *
 * 函式：
 *   odom_counter_delta()  算兩次讀值的差，正確處理繞回
 *                         （16-bit 時 65535 → 0 算 +1；32-bit 同理）。
 *   odom_init()           啟動 4 個 encoder timer、計數器歸零、清空累計值。
 *                         要在 MX_TIMx_Init() 之後呼叫一次。
 *   odom_update()         一次讀完 4 個 CNT（讓時間點盡量一致），
 *                         記下當下的 HAL_GetTick() 當 stamp，再把差值累加進 total。
 *                         要夠頻繁地呼叫，否則 16-bit 計數器繞回超過半圈會算錯。
 *   odom_get_snapshot()   把最近一次 odom_update() 的 stamp 和 4 個 total 複製出去。
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
