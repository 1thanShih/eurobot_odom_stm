/**
 * @file    imu.c
 * @brief   在自己的 FreeRTOS task 裡讀 BNO085，把最新資料存成快照給 micro-ROS 發布。
 *
 * 流程：
 *   INT 拉低 → EXTI 中斷叫醒 imu_task → sh2_service() 讀封包 → on_sensor() 更新快照
 *   找不到 BNO085、或超過 IMU_DATA_TIMEOUT_MS 沒資料，就重置後重新初始化。
 *   BNO085 自己重開機（SH2_RESET）時設定會消失，要重新開啟報告。
 *
 * 除錯用（debugger 的 Live Expressions）：
 *   imu_status         目前狀態
 *   imu_prod_ids       BNO085 回報的韌體版本，讀得到代表 I2C 和協定都通了
 *   imu_restart_count  重新初始化的次數
 *
 * 函式：
 *   imu_start()          建立 imu_task。
 *   imu_get_snapshot()   複製最新的快照（用 critical section 保護）。
 *   imu_task()           主迴圈：初始化 → 讀資料 → 出問題就重來。
 *   imu_setup()          sh2_open、讀 product ID、開啟 3 種報告。
 *   imu_enable_reports() 設定 Game Rotation Vector、Gyro、Accel 的輸出週期。
 *   imu_run()            等 INT、讀資料，直到逾時才返回。
 *   on_sensor()          sh2 收到一筆報告時呼叫，寫進快照。
 *   on_event()           sh2 的事件（例如 BNO085 重開機）。
 *   HAL_GPIO_EXTI_Callback()  INT 腳中斷：記時間、叫醒 imu_task。
 */
#include "imu.h"

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "robot_config.h"
#include "imu_hal.h"

#include "sh2.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

#define IMU_INT_FLAG         0x01U
#define IMU_POLL_MS          5U      /* 沒等到 INT 也定期檢查，避免漏掉 */
#define IMU_MAX_READS        8       /* 每次醒來最多連讀幾包 */
#define IMU_RETRY_MS         1000U   /* 找不到 BNO085 時多久重試 */
#define IMU_DATA_TIMEOUT_MS  500U    /* 這麼久沒資料就重新初始化 */

typedef enum {
    IMU_STATUS_STARTING,
    IMU_STATUS_NOT_FOUND,   /* 沒接、沒電，或 I2C 不通 */
    IMU_STATUS_RUNNING,
} ImuStatus;

/* 除錯用，不加 static 讓 debugger 看得到 */
volatile ImuStatus imu_status = IMU_STATUS_STARTING;
sh2_ProductIds_t   imu_prod_ids;
volatile uint32_t  imu_restart_count = 0;

static osThreadId_t volatile imu_thread = NULL;
static ImuSnapshot snapshot;        /* 跨 task 共用，讀寫都要進 critical section */
static volatile bool reset_seen;    /* BNO085 重開機過，要重新開報告 */
static uint32_t last_data_ms;

static const osThreadAttr_t imu_task_attr = {
    .name       = "imuTask",
    .stack_size = 512 * 4,
    .priority   = osPriorityBelowNormal,   /* 比 micro-ROS 低，不影響 encoder 的週期 */
};

static void on_sensor(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t v;
    if (sh2_decodeSensorEvent(&v, event) != SH2_OK) {
        return;
    }

    taskENTER_CRITICAL();
    switch (v.sensorId) {
    case SH2_GAME_ROTATION_VECTOR:
        snapshot.quat[0]  = v.un.gameRotationVector.real;
        snapshot.quat[1]  = v.un.gameRotationVector.i;
        snapshot.quat[2]  = v.un.gameRotationVector.j;
        snapshot.quat[3]  = v.un.gameRotationVector.k;
        snapshot.stamp_us = v.timestamp;
        snapshot.seq++;
        snapshot.valid = true;
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        snapshot.gyro[0] = v.un.gyroscope.x;
        snapshot.gyro[1] = v.un.gyroscope.y;
        snapshot.gyro[2] = v.un.gyroscope.z;
        break;
    case SH2_ACCELEROMETER:
        snapshot.accel[0] = v.un.accelerometer.x;
        snapshot.accel[1] = v.un.accelerometer.y;
        snapshot.accel[2] = v.un.accelerometer.z;
        break;
    default:
        break;
    }
    taskEXIT_CRITICAL();

    last_data_ms = HAL_GetTick();
}

static void on_event(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET) {
        reset_seen = true;
    }
}

static bool imu_enable_reports(void)
{
    static const sh2_SensorId_t ids[] = {
        SH2_GAME_ROTATION_VECTOR,
        SH2_GYROSCOPE_CALIBRATED,
        SH2_ACCELEROMETER,
    };
    sh2_SensorConfig_t cfg = {0};
    cfg.reportInterval_us = IMU_REPORT_PERIOD_US;

    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        if (sh2_setSensorConfig(ids[i], &cfg) != SH2_OK) {
            return false;
        }
    }
    return true;
}

static bool imu_setup(void)
{
    if (sh2_open(imu_hal_get(), on_event, NULL) != SH2_OK) {
        return false;
    }
    reset_seen = false;   /* 開機時的那次重開機不用處理 */
    if (sh2_setSensorCallback(on_sensor, NULL) != SH2_OK ||
        sh2_getProdIds(&imu_prod_ids) != SH2_OK ||
        !imu_enable_reports()) {
        sh2_close();
        return false;
    }
    return true;
}

static void imu_run(void)
{
    last_data_ms = HAL_GetTick();
    for (;;) {
        (void)osThreadFlagsWait(IMU_INT_FLAG, osFlagsWaitAny, IMU_POLL_MS);

        for (int i = 0; i < IMU_MAX_READS && imu_hal_int_asserted(); i++) {
            sh2_service();
        }

        if (reset_seen) {
            reset_seen = false;
            if (!imu_enable_reports()) {
                return;
            }
        }
        if (HAL_GetTick() - last_data_ms > IMU_DATA_TIMEOUT_MS) {
            return;
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
        sh2_close();
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

void imu_get_snapshot(ImuSnapshot *out)
{
    taskENTER_CRITICAL();
    *out = snapshot;
    taskEXIT_CRITICAL();
}

/* HAL 的 EXTI callback 只有一個，所有 GPIO 中斷都會進來，只處理 IMU_INT */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IMU_INT_Pin) {
        return;
    }
    imu_hal_on_int();
    if (imu_thread != NULL) {
        (void)osThreadFlagsSet(imu_thread, IMU_INT_FLAG);
    }
}
