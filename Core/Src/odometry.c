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
    int8_t   direction;   // 1 -> forward, -1 -> backward
    bool     is_32bit;    //odom_init() will set this
    uint32_t last_count;  // last read value of the timer counter
    int32_t  total;
    int32_t  delta;
} OdomPod;

// configured in robot_config.h
static OdomPod pods[ODOM_POD_COUNT] = {
    [ODOM_POD_1] = { .htim = &htim2, .direction = ODOM_POD1_DIRECTION },  /* PA0 / PA1, 32-bit */
    [ODOM_POD_2] = { .htim = &htim3, .direction = ODOM_POD2_DIRECTION },  /* PA6 / PA7, 16-bit */
    [ODOM_POD_3] = { .htim = &htim1, .direction = ODOM_POD3_DIRECTION },  /* PA8 / PA9, 16-bit */
    [ODOM_POD_4] = { .htim = &htim4, .direction = ODOM_POD4_DIRECTION },  /* PB6 / PB7, 16-bit */
};

static uint32_t last_update_ms = 0;

static bool timer_is_32bit(const TIM_HandleTypeDef *htim)
{
    return (htim->Instance == TIM2) || (htim->Instance == TIM5);
}

int32_t odom_counter_delta(uint32_t now, uint32_t last, bool is_32bit)
{
    // first cast to unsigned to avoid undefined behavior 
    // when subtracting two unsigned numbers that wrap around
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

    // read all counters first, then update totals, to avoid time skew between reads
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
