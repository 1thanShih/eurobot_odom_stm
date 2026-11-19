/**
 * @file    imu_hal.c
 * @brief   把 CEVA sh2 函式庫接到 STM32：I2C1 收發、RST 重置、INT 判斷有沒有資料、微秒時鐘。
 *
 * 只在 imu_task 裡被呼叫，所以直接用 blocking 的 HAL_I2C 函式。
 * BNO085 一個封包的前 4 byte 是標頭（含長度），所以讀取分兩次：
 * 先讀 4 byte 拿長度，再整包讀一次（第二次會重新從標頭開始送）。
 *
 * 函式：
 *   hal_open()       拉低 RST 重置 BNO085，等它拉低 INT 表示開機完成。
 *   hal_close()      拉低 RST，讓 BNO085 停在重置狀態。
 *   hal_read()       INT 是低電位才讀；回傳讀到的長度，沒資料回傳 0。
 *   hal_write()      送一個封包，失敗回傳 SH2_ERR_IO（回傳 0 會讓 sh2 無限重試）。
 *   imu_hal_now_us() HAL_GetTick() × 1000 + TIM6 計數值（TIM6 是 1 MHz、每 1 ms 繞回）。
 */
#include "imu_hal.h"

#include "main.h"
#include "cmsis_os.h"
#include "sh2_err.h"

#define IMU_I2C_ADDR        (0x4AU << 1)   /* 7-bit 位址 0x4A，HAL 要左移一位 */
#define IMU_I2C_TIMEOUT_MS  10U
#define IMU_WRITE_RETRIES   3
#define IMU_RESET_MS        10U            /* RST 拉低多久 */
#define IMU_BOOT_TIMEOUT_MS 500U           /* 放開 RST 後等 INT 的上限 */
#define SHTP_HDR_LEN        4U

extern I2C_HandleTypeDef hi2c1;

static volatile uint32_t int_time_us;      /* 最近一次 INT 下降緣的時間 */
static volatile bool     int_time_fresh;   /* int_time_us 還沒被用過 */

uint32_t imu_hal_now_us(void)
{
    uint32_t ms, cnt;
    bool pending;
    /* 讀的途中剛好跨過 1 ms 就重讀 */
    do {
        ms  = HAL_GetTick();
        cnt = TIM6->CNT;
        /* TIM6 剛繞回、tick 中斷還沒跑：ms 還沒 +1，要自己補 */
        pending = ((TIM6->SR & TIM_SR_UIF) != 0U) && (cnt < 500U);
    } while (ms != HAL_GetTick());
    if (pending) {
        ms++;
    }
    return ms * 1000U + cnt;   /* uint32 溢位後剛好從 0 接續，sh2 會自己處理 */
}

void imu_hal_on_int(void)
{
    int_time_us    = imu_hal_now_us();
    int_time_fresh = true;
}

bool imu_hal_int_asserted(void)
{
    return HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_RESET;
}

static int hal_open(sh2_Hal_t *self)
{
    (void)self;
    int_time_fresh = false;

    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    osDelay(IMU_RESET_MS);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);

    /* 開機完成後 BNO085 會拉低 INT，準備送開機訊息 */
    for (uint32_t waited = 0; waited < IMU_BOOT_TIMEOUT_MS; waited++) {
        if (imu_hal_int_asserted()) {
            return SH2_OK;
        }
        osDelay(1);
    }
    return SH2_ERR_TIMEOUT;   /* 沒接或沒電 */
}

static void hal_close(sh2_Hal_t *self)
{
    (void)self;
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
}

static int hal_read(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t_us)
{
    (void)self;
    if (!imu_hal_int_asserted()) {
        return 0;
    }

    /* 有剛發生的 INT 就用它的時間，比讀完才記時間準 */
    static uint32_t last_t;
    uint32_t t = int_time_fresh ? int_time_us : imu_hal_now_us();
    int_time_fresh = false;
    if ((int32_t)(t - last_t) < 0) {
        t = last_t;   /* 不能倒退，否則 sh2 會誤判成 32-bit 溢位 */
    }
    last_t = t;

    uint8_t hdr[SHTP_HDR_LEN];
    if (HAL_I2C_Master_Receive(&hi2c1, IMU_I2C_ADDR, hdr, SHTP_HDR_LEN, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    uint16_t n = (uint16_t)((hdr[0] | (hdr[1] << 8)) & 0x7FFFU);   /* bit 15 是「續傳」旗標 */
    if (n < SHTP_HDR_LEN) {
        return 0;
    }
    if (n > len) {
        n = (uint16_t)len;   /* 太長就只讀放得下的部分，剩下的 BNO085 會分次續傳 */
    }
    if (HAL_I2C_Master_Receive(&hi2c1, IMU_I2C_ADDR, buf, n, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    *t_us = t;
    return n;
}

static int hal_write(sh2_Hal_t *self, uint8_t *buf, unsigned len)
{
    (void)self;
    for (int i = 0; i < IMU_WRITE_RETRIES; i++) {
        if (HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_ADDR, buf, (uint16_t)len, IMU_I2C_TIMEOUT_MS) == HAL_OK) {
            return (int)len;
        }
        osDelay(1);   /* BNO085 忙碌時會 NAK，稍等再試 */
    }
    return SH2_ERR_IO;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    (void)self;
    return imu_hal_now_us();
}

static sh2_Hal_t imu_hal = {
    .open      = hal_open,
    .close     = hal_close,
    .read      = hal_read,
    .write     = hal_write,
    .getTimeUs = hal_get_time_us,
};

sh2_Hal_t *imu_hal_get(void)
{
    return &imu_hal;
}
