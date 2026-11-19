/**
 * @file    imu.h
 * @brief   讀 BNO085（I2C1），把最新的姿態、角速度、加速度存成快照。
 *
 * 用的報告：Game Rotation Vector（不用磁力計）、Gyroscope Calibrated、Accelerometer。
 * 讀取在自己的 FreeRTOS task 裡做，其他 task 用 imu_get_snapshot() 拿資料。
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     valid;      /* 收到過至少一筆姿態 */
    uint32_t seq;        /* 每收到一筆姿態 +1，用來判斷有沒有新資料 */
    uint64_t stamp_us;   /* 姿態的取樣時間，STM32 開機後的微秒數（和 HAL_GetTick 同一個時鐘） */
    float    quat[4];    /* 姿態四元數 w, x, y, z */
    float    gyro[3];    /* 角速度 x, y, z（rad/s） */
    float    accel[3];   /* 加速度 x, y, z（m/s²，含重力） */
} ImuSnapshot;

/** 建立 IMU task。在 FreeRTOS task 裡呼叫一次。 */
void imu_start(void);

/** 取得最新的 IMU 資料。 */
void imu_get_snapshot(ImuSnapshot *out);

#endif /* IMU_H */
