/**
 * @file    app.c
 * @brief   應用程式的入口，把 CubeMX 產生的 main.c 和我們自己的模組接起來。
 *
 * main.c 只呼叫這裡的兩個函式，其他邏輯都放在各自的模組裡。
 * 這樣重新用 CubeMX 產生 code 時，我們的程式不會被覆蓋。
 *
 * 函式：
 *   app_init()  開機時在 main() 裡呼叫一次（FreeRTOS 啟動前）。
 *               目前只做 odom_init()：啟動 4 個 encoder timer 並歸零。
 *   app_task()  在 FreeRTOS 的 defaultTask 裡執行，不會返回。
 *               目前直接進入 microros_app_task()（讀 encoder + 發布 ROS topic 的主迴圈）。
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
