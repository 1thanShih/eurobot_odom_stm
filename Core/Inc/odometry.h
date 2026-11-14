/**
 * @file    odometry.h
 * @brief   讀取 odometry pod 的 encoder 計數。
 *
 * 每個 pod 接到一個設成 Encoder mode 的 Timer，硬體會自動加減計數，
 * 這個模組只負責定期讀取計數器、處理溢位、累計總量。
 * 座標 (x, y, θ) 由上位機計算，這裡不做。
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

/** 某一瞬間所有 pod 的資料（單位：count）。 */
typedef struct {
    uint32_t timestamp_ms;          /* 取樣時的 HAL_GetTick() */
    int32_t  total[ODOM_POD_COUNT]; /* 開機以來的累計 count */
    int32_t  delta[ODOM_POD_COUNT]; /* 與上一次取樣相比的變化量 */
} OdomSnapshot;

/** 啟動所有 encoder timer 並歸零。在 CubeMX 的 MX_TIMx_Init() 之後呼叫。 */
void odom_init(void);

/** 讀取所有計數器並更新累計值。週期性呼叫。 */
void odom_update(void);

/** 複製最近一次 odom_update() 的結果。 */
void odom_get_snapshot(OdomSnapshot *out);

/** count 轉換成 mm。 */
float odom_counts_to_mm(int32_t counts);

/**
 * 計算兩次讀值之間的變化量，正確處理計數器溢位（例如 65535 → 0）。
 * 不碰硬體，可在電腦上做單元測試。
 */
int32_t odom_counter_delta(uint32_t now, uint32_t last, bool is_32bit);

#endif /* ODOMETRY_H */
