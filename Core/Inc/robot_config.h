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

/* 讀 encoder 並發布的週期（ms）。IMU 也跟著每週期讀一次（BNO055 融合輸出 100 Hz，比 10 小沒有用） */
#define ODOM_SAMPLE_PERIOD_MS    10U

/* /imu/data_raw 的共變異數（對角線），估計值，上位機可再調 */
#define IMU_GYRO_VAR             0.0004   /* (rad/s)² */
#define IMU_ACCEL_VAR            0.04     /* (m/s²)² */

#endif /* ROBOT_CONFIG_H */
