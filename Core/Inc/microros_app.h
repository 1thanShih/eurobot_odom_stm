/**
 * @file    microros_app.h
 * @brief   透過 micro-ROS 發布 odometry count 和 IMU 資料。
 *
 * Topic : /wheel_counts
 * Type  : std_msgs/Int32MultiArray，data 固定 5 個值
 *           data[0]   = stamp，STM32 開機後的毫秒數（讀 encoder 的時間點）
 *           data[1~4] = pod1~4 的累計 count
 * 週期  : ODOM_SAMPLE_PERIOD_MS（robot_config.h）
 *
 * Topic : /imu/data_raw（sensor_msgs/Imu），frame_id = imu_link
 *           BNO055 加速度計、陀螺儀的原始值，沒有姿態（orientation_covariance[0] = -1），姿態由上位機融合
 *           header.stamp = 和同一週期 /wheel_counts 的 data[0] 完全相同
 * 週期  : 每讀一次 encoder 就發一筆（ODOM_SAMPLE_PERIOD_MS）；沒接 IMU 就不發
 *
 * 板上 LED（PC13）：熄滅 = 等待 agent，亮起 = 已連線。
 * agent 晚開或中途斷線都會自動重連，不用按 RESET。
 */
#ifndef MICROROS_APP_H
#define MICROROS_APP_H

/** 在 FreeRTOS task 裡呼叫，不會返回。 */
void microros_app_task(void);

#endif /* MICROROS_APP_H */
