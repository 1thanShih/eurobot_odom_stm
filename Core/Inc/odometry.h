/**
 * @file    odometry.h
 * @brief   讀 4 個 odometry pod 的 encoder，累計 count。
 *
 * 每個 pod 接一個 Encoder mode 的 Timer，硬體自己加減計數；
 * 這裡只負責定期讀取、處理計數器溢位、累加成 32-bit 的總量。
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ODOM_POD_1 = 0,
    ODOM_POD_2,
    ODOM_POD_3,
    ODOM_POD_4,
    ODOM_POD_COUNT
} OdomPodId;

typedef struct {
    uint32_t stamp_ms;              /* 讀取當下的 HAL_GetTick() */
    int32_t  total[ODOM_POD_COUNT]; /* 開機以來的累計 count */
} OdomSnapshot;

/** 啟動 encoder timer 並歸零。在 MX_TIMx_Init() 之後呼叫一次。 */
void odom_init(void);

/** 讀取 4 個計數器，更新累計值。 */
void odom_update(void);

/** 取得最近一次 odom_update() 的結果。 */
void odom_get_snapshot(OdomSnapshot *out);

/** 兩次讀值的差，會處理溢位（例如 65535 → 0 算 +1）。 */
int32_t odom_counter_delta(uint32_t now, uint32_t last, bool is_32bit);

#endif /* ODOMETRY_H */
