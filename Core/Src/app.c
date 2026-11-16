/**
 * @file    app.c
 * @brief   應用程式主流程。
 */
#include "app.h"

#include "odometry.h"
#include "robot_config.h"

#if APP_OUTPUT_MICROROS
#include "microros_app.h"
#else
#include "cmsis_os.h"
#include "telemetry.h"
#endif

void app_init(void)
{
    odom_init();   /* 啟動 4 個 encoder timer */
}

#if APP_OUTPUT_MICROROS

void app_task(void)
{
    microros_app_task();
}

#else /* CSV 模式 */

void app_task(void)
{
    uint32_t next_wake = osKernelGetTickCount();

    for (;;) {
        odom_update();

        OdomSnapshot snap;
        odom_get_snapshot(&snap);
        telemetry_send_odom(&snap);

        /* 用「下一次該醒來的時間」排程，週期不會因為傳送花的時間而變長。
         * FreeRTOS tick = 1 ms（FreeRTOSConfig.h 的 configTICK_RATE_HZ），所以可以直接加 ms */
        next_wake += ODOM_SAMPLE_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}

#endif
