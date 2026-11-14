/**
 * @file    app.c
 * @brief   應用程式主流程。說明見 app.h。
 */
#include "app.h"

#include "main.h"          /* HAL_GetTick() */
#include "odometry.h"
#include "robot_config.h"
#include "telemetry.h"

static uint32_t last_sample_ms = 0;

void app_init(void)
{
    /* 注意：Nucleo 的 LED / 按鈕（BSP_LED_Init、BSP_PB_Init）在 main.c 中
     * 是在 app_init() 之後才初始化，所以不要在這裡操作它們。 */
    odom_init();
    last_sample_ms = HAL_GetTick();
}

void app_loop(void)
{
    /* 用時間戳判斷是否到了取樣時間，而不是 HAL_Delay()：
     * 這樣傳輸花掉的時間不會累加進週期，週期才會接近 ODOM_SAMPLE_PERIOD_MS。 */
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - last_sample_ms < ODOM_SAMPLE_PERIOD_MS) {
        return;
    }
    last_sample_ms = now_ms;

    odom_update();

    OdomSnapshot snap;
    odom_get_snapshot(&snap);
    telemetry_send_odom(&snap);
}
