/**
 * @file    imu_hal.h
 * @brief   sh2 函式庫需要的硬體介面（I2C1 + RST/INT 腳），只給 imu.c 用。
 */
#ifndef IMU_HAL_H
#define IMU_HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "sh2_hal.h"

/** 取得給 sh2_open() 用的 HAL。 */
sh2_Hal_t *imu_hal_get(void);

/** INT 腳的中斷裡呼叫：記下資料到達的時間。 */
void imu_hal_on_int(void);

/** INT 腳是否為低電位（= BNO085 有資料要給）。 */
bool imu_hal_int_asserted(void);

/** 微秒時鐘，和 HAL_GetTick() 同一個時間基準。 */
uint32_t imu_hal_now_us(void);

#endif /* IMU_HAL_H */
