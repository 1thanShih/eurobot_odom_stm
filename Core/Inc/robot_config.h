/**
 * @file    robot_config.h
 * @brief   機器人參數集中管理。校正、換零件、改方向時只需要改這個檔案。
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* ---- Odometry pod 機構參數 -------------------------------------------- */
#define ODOM_COUNTS_PER_REV      2000.0f   /* 輪子轉一圈的 count 數（已含 4 倍頻） */
#define ODOM_WHEEL_DIAMETER_MM   32.0f     /* 輪徑，實測校正後更新這裡 */
#define ODOM_MM_PER_COUNT        (3.14159265f * ODOM_WHEEL_DIAMETER_MM / ODOM_COUNTS_PER_REV)

/* ---- 各 pod 方向：往前推時數值變負，就把 +1 改成 -1 ------------------ */
#define ODOM_POD1_DIRECTION      (+1)
#define ODOM_POD2_DIRECTION      (+1)
#define ODOM_POD3_DIRECTION      (+1)
#define ODOM_POD4_DIRECTION      (+1)

/* ---- 輸出方式 ---------------------------------------------------------- */
/* USART2 一次只能給一種用途：
 *   1 = micro-ROS（需要上位機跑 micro-ROS agent）
 *   0 = CSV 模式：每 ODOM_SAMPLE_PERIOD_MS 送一行文字，可直接用序列埠工具或
 *       tools/odom_host.py 看，適合除錯 */
#define APP_OUTPUT_MICROROS      1

/* ---- 取樣與傳輸 -------------------------------------------------------- */
/* USART2 = 460800 baud（在 .ioc 設定）。一行 CSV 約 65~80 bytes，約需 1.5~2 ms，
 * 必須小於取樣週期。改 baud rate 時，CubeMX 和上位機要一起改。 */
#define ODOM_SAMPLE_PERIOD_MS       10U
#define TELEMETRY_UART_TIMEOUT_MS   100U

/* 預留：多塊板子時用來辨識來源，目前尚未放進輸出 */
#define BOARD_ID                1

#endif /* ROBOT_CONFIG_H */
