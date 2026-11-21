/**
 * @file    imu_hal.c
 * @brief   BNO055 的硬體介面：I2C1 讀寫暫存器、RST 重置。
 *
 * 只在 imu_task 裡被呼叫，所以直接用 blocking 的 HAL_I2C 函式。
 * BNO055 的 INT 腳只用在動作偵測，融合資料沒有「資料好了」的中斷，所以不用 INT。
 *
 * 除錯用（debugger 的 Live Expressions）：
 *   imu_i2c_addr7    找到的 7-bit 位址，0 = 還沒找到
 *
 * 函式：
 *   imu_hal_reset()      拉低 RST，等開機，用 CHIP_ID 在 0x28 / 0x29 找 BNO055。
 *   imu_hal_write_reg()  寫一個暫存器。
 *   imu_hal_read_regs()  連續讀多個暫存器。
 */
#include "imu_hal.h"

#include "main.h"
#include "cmsis_os.h"

#define IMU_I2C_ADDR_A      0x28U          /* ADR 接 GND 或不接（Adafruit 預設） */
#define IMU_I2C_ADDR_B      0x29U          /* ADR 接 3.3V */
#define IMU_I2C_TIMEOUT_MS  10U
#define IMU_RESET_MS        10U            /* RST 拉低多久 */
#define IMU_BOOT_MS         650U           /* 放開 RST 到能通訊（datasheet） */
#define IMU_BOOT_TIMEOUT_MS 1000U          /* 再多等這麼久還找不到就放棄 */
#define BNO055_REG_CHIP_ID  0x00U
#define BNO055_CHIP_ID      0xA0U

extern I2C_HandleTypeDef hi2c1;

/* 不加 static 讓 debugger 看得到 */
volatile uint8_t imu_i2c_addr7;

static bool chip_id_ok(uint8_t addr7)
{
    uint8_t id = 0;
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), BNO055_REG_CHIP_ID, I2C_MEMADD_SIZE_8BIT,
                            &id, 1, IMU_I2C_TIMEOUT_MS) == HAL_OK &&
           id == BNO055_CHIP_ID;
}

bool imu_hal_reset(void)
{
    imu_i2c_addr7 = 0;
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    osDelay(IMU_RESET_MS);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    osDelay(IMU_BOOT_MS);

    for (uint32_t waited = 0; waited < IMU_BOOT_TIMEOUT_MS; waited += 10U) {
        if (chip_id_ok(IMU_I2C_ADDR_A)) {
            imu_i2c_addr7 = IMU_I2C_ADDR_A;
            return true;
        }
        if (chip_id_ok(IMU_I2C_ADDR_B)) {
            imu_i2c_addr7 = IMU_I2C_ADDR_B;
            return true;
        }
        osDelay(10);
    }
    return false;   /* 沒接、沒電，或 SDA/SCL 不通 */
}

bool imu_hal_write_reg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(imu_i2c_addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, IMU_I2C_TIMEOUT_MS) == HAL_OK;
}

bool imu_hal_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(imu_i2c_addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, IMU_I2C_TIMEOUT_MS) == HAL_OK;
}
