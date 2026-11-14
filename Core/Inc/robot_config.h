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

/* ---- 取樣與傳輸 -------------------------------------------------------- */
/* 一行 CSV 約 65~80 bytes，115200 baud 約需 6~7 ms，必須小於取樣週期。
 * 若要提高取樣率，需同時提高 baud rate（CubeMX 的 USART2 與上位機都要改）。 */
#define ODOM_SAMPLE_PERIOD_MS       10U
#define TELEMETRY_UART_TIMEOUT_MS   100U

/* 預留：多塊板子時用來辨識來源，目前尚未放進輸出 */
#define BOARD_ID                1

#endif /* ROBOT_CONFIG_H */
