/*
 * microros_app.h
 *
 * micro-ROS 連線與發布。在 FreeRTOS 的 defaultTask 裡呼叫 microros_app_task()，
 * 它不會返回。
 *
 * 目前是「環境測試」版本：連上 agent 後每 100 ms 在 /stm32_test 發布一個遞增的
 * std_msgs/Int32。之後再換成 odometry 的訊息。
 *
 * LED2（板上綠燈）：
 *   熄滅 = 等待 agent
 *   亮起 = 已連線，正在發布
 */
#ifndef MICROROS_APP_H
#define MICROROS_APP_H

void microros_app_task(void);

#endif /* MICROROS_APP_H */
