/**
 * @file    imu.h
 * @brief   讀 BNO055（I2C1），把最新的加速度、角速度存成快照。
 *
 * 用 ACCGYRO 模式：只開加速度計和陀螺儀，不做融合，姿態由上位機自己算。
 * 讀取在自己的 FreeRTOS task 裡做：micro-ROS task 讀完 encoder 就呼叫 imu_request_sample()，
 * IMU task 馬上讀一筆，stamp 和 encoder 相同。其他 task 用 imu_get_snapshot() 拿資料。
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     valid;      /* 收到過至少一筆 */
    uint32_t seq;        /* 每讀到一筆 +1，用來判斷有沒有新資料 */
    uint32_t stamp_ms;   /* 觸發這筆讀取的 encoder stamp（STM32 開機後的毫秒數），和 /wheel_counts 一樣 */
    float    gyro[3];    /* 角速度 x, y, z（rad/s） */
    float    accel[3];   /* 加速度 x, y, z（m/s²，含重力） */
} ImuSnapshot;

/** 建立 IMU task。在 FreeRTOS task 裡呼叫一次。 */
void imu_start(void);

/** 讀完 encoder 後呼叫：叫 IMU task 用同一個 stamp 讀一筆。 */
void imu_request_sample(uint32_t stamp_ms);

/** 取得最新的 IMU 資料。 */
void imu_get_snapshot(ImuSnapshot *out);

#endif /* IMU_H */
