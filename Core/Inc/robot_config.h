/**
 * @file    robot_config.h
 * @brief   可調參數都放這裡。
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* 各 pod 的方向：往前推時 count 變負，就把 +1 改成 -1 */
#define ODOM_POD1_DIRECTION      (+1)
#define ODOM_POD2_DIRECTION      (+1)
#define ODOM_POD3_DIRECTION      (+1)
#define ODOM_POD4_DIRECTION      (+1)

/* 讀 encoder 並發布的週期（ms） */
#define ODOM_SAMPLE_PERIOD_MS    10U

/* BNO085 每種報告（姿態、角速度、加速度）的輸出週期（µs） */
#define IMU_REPORT_PERIOD_US     10000U

/* /imu 的共變異數（對角線），估計值，上位機 EKF 可再調 */
#define IMU_ORIENTATION_VAR      0.002    /* rad²，約 ±2.5° */
#define IMU_GYRO_VAR             0.0004   /* (rad/s)² */
#define IMU_ACCEL_VAR            0.04     /* (m/s²)² */

#endif /* ROBOT_CONFIG_H */
