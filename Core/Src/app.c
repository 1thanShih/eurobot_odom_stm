/**
 * @file    app.c
 * @brief   見 app.h。
 */
#include "app.h"

#include "microros_app.h"
#include "odometry.h"

void app_init(void)
{
    odom_init();
}

void app_task(void)
{
    microros_app_task();
}
