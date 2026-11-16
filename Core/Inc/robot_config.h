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

#endif /* ROBOT_CONFIG_H */
