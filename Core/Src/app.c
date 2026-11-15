/**
 * @file    app.c
 * @brief   Application main loop.
 */
#include "app.h"

#include "main.h"          /* HAL_GetTick() */
#include "odometry.h"
#include "robot_config.h"
#include "telemetry.h"

static uint32_t last_sample_ms = 0;

void app_init(void)
{
    odom_init(); // from odometry.c, initialize odometry module, start encoder timers
    last_sample_ms = HAL_GetTick();
}

void app_loop(void)
{
    // every ODOM_SAMPLE_PERIOD_MS sample odometry and send telemetry
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
