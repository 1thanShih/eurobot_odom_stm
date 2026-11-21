/**
 * @file    imu_hal.h
 * @brief   BNO055 的硬體介面（I2C1 讀寫暫存器、RST 腳），只給 imu.c 用。
 */
#ifndef IMU_HAL_H
#define IMU_HAL_H

#include <stdbool.h>
#include <stdint.h>

/** 拉低 RST 重置 BNO055，等它開機並找出 I2C 位址（0x28 或 0x29）。找不到回傳 false。 */
bool imu_hal_reset(void);

/** 寫一個暫存器。 */
bool imu_hal_write_reg(uint8_t reg, uint8_t val);

/** 從 reg 開始連續讀 len 個暫存器。 */
bool imu_hal_read_regs(uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* IMU_HAL_H */
