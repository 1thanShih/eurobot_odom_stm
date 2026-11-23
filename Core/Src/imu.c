/**
 * @file    imu.c
 * @brief   在自己的 FreeRTOS task 裡讀 BNO055，把最新資料存成快照給 micro-ROS 發布。
 *
 * 流程：
 *   重置 → 設成 ACCGYRO 模式（只開加速度計、陀螺儀，不融合）→ 等 micro-ROS task 讀完 encoder 後觸發
 *   → 馬上一次讀 0x08~0x19（加速度、磁力、角速度；磁力計關著，那 6 byte 不用），stamp 直接用 encoder 的 stamp。
 *   感測器更新率是 BNO055 預設值：加速度 125 Hz、角速度 100 Hz。
 *   這樣每一筆 /imu 都和同一週期的 /wheel_counts 有一樣的時間戳。
 *   連續讀失敗、或 BNO055 不在正常運作狀態（例如自己重開機），就重置後重新初始化。
 *
 * 除錯用（debugger 的 Live Expressions）：
 *   imu_status         目前狀態
 *   imu_sys_err        SYS_ERR，0 = 沒有錯誤
 *   imu_restart_count  重新初始化的次數
 *
 * 函式：
 *   imu_start()            建立 imu_task。
 *   imu_request_sample()   micro-ROS task 讀完 encoder 後呼叫，叫 imu_task 用同一個 stamp 讀一筆。
 *   imu_get_snapshot()     複製最新的快照（用 critical section 保護）。
 *   imu_task()             主迴圈：初始化 → 讀資料 → 出問題就重來。
 *   imu_setup()            重置、設定單位和時脈、切到 ACCGYRO 模式。
 *   imu_run()              每次被觸發就讀一筆，出問題才返回。
 *   store_sample()         把讀到的原始值換算成 SI 單位，寫進快照。
 */
#include "imu.h"

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu_hal.h"

/* BNO055 暫存器（page 0） */
#define BNO_REG_PAGE_ID         0x07U
#define BNO_REG_DATA_START      0x08U   /* ACC_DATA_X_LSB */
#define BNO_REG_SYS_STATUS      0x39U
#define BNO_REG_UNIT_SEL        0x3BU
#define BNO_REG_OPR_MODE        0x3DU
#define BNO_REG_PWR_MODE        0x3EU
#define BNO_REG_SYS_TRIGGER     0x3FU

#define DATA_LEN            18U     /* 0x08~0x19 */
#define OFS_ACC             0U      /* 0x08 */
#define OFS_GYR             12U     /* 0x14 */

#define BNO_OPR_MODE_ACCGYRO    0x05U   /* 只開加速度計、陀螺儀，不融合 */
#define BNO_PWR_MODE_NORMAL     0x00U
#define BNO_UNIT_SEL_GYRO_RPS   0x02U   /* 角速度用 rad/s，其他維持預設（m/s²） */
#define BNO_SYS_TRIGGER_EXT_CLK 0x80U   /* 用板上的 32.768 kHz 石英，比內部振盪器準 */
#define BNO_SYS_STATUS_RUNNING  0x06U   /* 正常運作（沒有融合） */

#define ACC_LSB_PER_MS2     100.0f
#define GYR_LSB_PER_RPS     900.0f

#define IMU_SAMPLE_FLAG     0x01U
#define IMU_WAIT_MS         200U    /* 等觸發的上限（micro-ROS ping agent 時會停一下） */
#define IMU_RETRY_MS        1000U   /* 找不到 BNO055 時多久重試 */
#define IMU_MAX_FAILS       10U     /* 連續讀失敗幾次就重新初始化 */
#define IMU_CHECK_EVERY     100U    /* 每讀幾次檢查一次狀態（約 1 秒） */

typedef enum {
    IMU_STATUS_STARTING,
    IMU_STATUS_NOT_FOUND,   /* 沒接、沒電，或 I2C 不通 */
    IMU_STATUS_RUNNING,
} ImuStatus;

/* 除錯用，不加 static 讓 debugger 看得到 */
volatile ImuStatus imu_status = IMU_STATUS_STARTING;
volatile uint8_t   imu_sys_err;
volatile uint32_t  imu_restart_count = 0;

static osThreadId_t volatile imu_thread = NULL;
static volatile uint32_t request_stamp_ms;   /* 觸發時的 encoder stamp */
static ImuSnapshot snapshot;                 /* 跨 task 共用，讀寫都要進 critical section */

static const osThreadAttr_t imu_task_attr = {
    .name       = "imuTask",
    .stack_size = 512 * 4,
    /* 比 micro-ROS 高：一觸發就搶先讀（約 1 ms），讀完 micro-ROS 再發布，同一週期就送得出去 */
    .priority   = osPriorityAboveNormal,
};

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void store_sample(const uint8_t *raw, uint32_t stamp_ms)
{
    taskENTER_CRITICAL();
    for (int i = 0; i < 3; i++) {
        snapshot.gyro[i]  = (float)le16(&raw[OFS_GYR + 2 * i]) / GYR_LSB_PER_RPS;
        snapshot.accel[i] = (float)le16(&raw[OFS_ACC + 2 * i]) / ACC_LSB_PER_MS2;
    }
    snapshot.stamp_ms = stamp_ms;
    snapshot.seq++;
    snapshot.valid = true;
    taskEXIT_CRITICAL();
}

static bool imu_setup(void)
{
    if (!imu_hal_reset()) {
        return false;
    }
    /* 重置後本來就在 CONFIG 模式，只有這個模式能改設定 */
    if (!imu_hal_write_reg(BNO_REG_PAGE_ID, 0) ||
        !imu_hal_write_reg(BNO_REG_PWR_MODE, BNO_PWR_MODE_NORMAL) ||
        !imu_hal_write_reg(BNO_REG_UNIT_SEL, BNO_UNIT_SEL_GYRO_RPS) ||
        !imu_hal_write_reg(BNO_REG_SYS_TRIGGER, BNO_SYS_TRIGGER_EXT_CLK)) {
        return false;
    }
    osDelay(10);
    if (!imu_hal_write_reg(BNO_REG_OPR_MODE, BNO_OPR_MODE_ACCGYRO)) {
        return false;
    }
    osDelay(20);   /* 切模式要 7 ms */
    return true;
}

static void imu_run(void)
{
    uint8_t  raw[DATA_LEN];
    uint8_t  st[2];   /* SYS_STATUS、SYS_ERR */
    uint32_t fails = 0;
    uint32_t count = 0;

    (void)osThreadFlagsClear(IMU_SAMPLE_FLAG);   /* 丟掉初始化期間累積的觸發 */
    for (;;) {
        if ((osThreadFlagsWait(IMU_SAMPLE_FLAG, osFlagsWaitAny, IMU_WAIT_MS) & osFlagsError) != 0U) {
            continue;   /* 逾時：micro-ROS 暫時沒在讀 encoder，不算 IMU 的錯 */
        }
        uint32_t stamp_ms = request_stamp_ms;

        if (!imu_hal_read_regs(BNO_REG_DATA_START, raw, DATA_LEN)) {
            if (++fails >= IMU_MAX_FAILS) {
                return;
            }
            continue;
        }
        fails = 0;
        store_sample(raw, stamp_ms);   /* 感測器更新率和讀取不同步，偶爾會和上一筆一樣 */

        if (++count % IMU_CHECK_EVERY == 0U) {
            if (!imu_hal_read_regs(BNO_REG_SYS_STATUS, st, sizeof(st))) {
                continue;   /* 讀失敗交給上面的計數處理 */
            }
            imu_sys_err = st[1];
            if (st[0] != BNO_SYS_STATUS_RUNNING) {
                return;   /* 重開機或出錯，設定已經不在了 */
            }
        }
    }
}

static void imu_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!imu_setup()) {
            imu_status = IMU_STATUS_NOT_FOUND;
            osDelay(IMU_RETRY_MS);
            continue;
        }
        imu_status = IMU_STATUS_RUNNING;
        imu_run();
        imu_restart_count++;
    }
}

void imu_start(void)
{
    imu_thread = osThreadNew(imu_task, NULL, &imu_task_attr);
    if (imu_thread == NULL) {
        Error_Handler();
    }
}

void imu_request_sample(uint32_t stamp_ms)
{
    if (imu_thread != NULL) {
        request_stamp_ms = stamp_ms;
        (void)osThreadFlagsSet(imu_thread, IMU_SAMPLE_FLAG);
    }
}

void imu_get_snapshot(ImuSnapshot *out)
{
    taskENTER_CRITICAL();
    *out = snapshot;
    taskEXIT_CRITICAL();
}
