/**
 * @file    telemetry.c
 * @brief   CSV 輸出。格式說明見 telemetry.h。
 */
#include "telemetry.h"

#include <stdio.h>

#include "main.h"
#include "robot_config.h"

/* CubeMX 在 main.c 產生的 UART handle */
extern UART_HandleTypeDef huart2;

#define TELEMETRY_LINE_MAX  256

void telemetry_send_odom(const OdomSnapshot *snap)
{
    static uint8_t seq = 0;
    char buf[TELEMETRY_LINE_MAX];

    /* seq 一定遞增，就算這行沒送出，上位機也能從跳號發現 */
    uint8_t this_seq = seq++;

    int n = snprintf(buf, sizeof(buf),
                     "%u,%lu,"                      /* seq, ts_ms        */
                     "%ld,%ld,%ld,%ld,"             /* total1..4         */
                     "%ld,%ld,%ld,%ld,"             /* delta1..4         */
                     "%.2f,%.2f,%.2f,%.2f\r\n",     /* mm1..4            */

                     /* seq, ts_ms */
                     (unsigned)this_seq, (unsigned long)snap->timestamp_ms,
                     /* total1..4 */
                     (long)snap->total[ODOM_POD_1], (long)snap->total[ODOM_POD_2],
                     (long)snap->total[ODOM_POD_3], (long)snap->total[ODOM_POD_4],
                     /* delta1..4 */
                     (long)snap->delta[ODOM_POD_1], (long)snap->delta[ODOM_POD_2],
                     (long)snap->delta[ODOM_POD_3], (long)snap->delta[ODOM_POD_4],
                     /* mm1..4 */
                     odom_counts_to_mm(snap->total[ODOM_POD_1]),
                     odom_counts_to_mm(snap->total[ODOM_POD_2]),
                     odom_counts_to_mm(snap->total[ODOM_POD_3]),
                     odom_counts_to_mm(snap->total[ODOM_POD_4]));

    /* 格式化失敗或被截斷：寧可不送，也不要送出半行資料 */
    if (n < 0 || n >= (int)sizeof(buf)) {
        return;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, TELEMETRY_UART_TIMEOUT_MS);
}
