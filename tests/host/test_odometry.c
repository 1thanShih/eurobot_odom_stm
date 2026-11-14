/*
 * 在電腦上測試 odometry.c / telemetry.c（不需要 STM32 板子）。
 * fake_hal/main.h 用假的 HAL 取代真的硬體。
 *
 * 執行方式（在專案根目錄）：
 *   gcc -std=gnu11 -Wall -Wextra -Wshadow -Werror \
 *       -Itests/host/fake_hal -ICore/Inc \
 *       tests/host/test_odometry.c Core/Src/odometry.c Core/Src/telemetry.c \
 *       -o test_odometry && ./test_odometry
 */
#include <stdio.h>
#include <stdlib.h>
#include "main.h"   /* 這裡會拿到 fake_hal/main.h */
#include "odometry.h"
#include "telemetry.h"

TIM_TypeDef fake_tim[6];
TIM_HandleTypeDef htim1 = { .Instance = TIM1 }, htim2 = { .Instance = TIM2 }, htim3 = { .Instance = TIM3 }, htim4 = { .Instance = TIM4 };
UART_HandleTypeDef huart2;
static uint32_t tick = 0;
static char uart_out[512];
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t ch) { (void)ch; h->started = 1; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *d, uint16_t n, uint32_t t)
{ (void)u; (void)t; memcpy(uart_out, d, n); uart_out[n] = 0; return HAL_OK; }
uint32_t HAL_GetTick(void) { return tick; }
void Error_Handler(void) { puts("Error_Handler called"); exit(1); }

static int fails = 0;
#define CHECK(cond) do { if (cond) printf("  PASS  %s\n", #cond); else { printf("  FAIL  %s (line %d)\n", #cond, __LINE__); fails++; } } while (0)

int main(void)
{
    puts("[1] counter_delta: 溢位處理");
    CHECK(odom_counter_delta(5, 65530, false) == 11);       /* 16-bit 正轉繞回 */
    CHECK(odom_counter_delta(65530, 5, false) == -11);      /* 16-bit 反轉繞回 */
    CHECK(odom_counter_delta(100, 40, false) == 60);
    CHECK(odom_counter_delta(0x10, 0xFFFFFFF0u, true) == 32);    /* 32-bit 繞回 */
    CHECK(odom_counter_delta(0xFFFFFFF0u, 0x10, true) == -32);

    puts("[2] init: 4 個 timer 都要啟動");
    htim1.cnt = htim2.cnt = htim3.cnt = htim4.cnt = 999;
    odom_init();
    CHECK(htim1.started && htim2.started && htim3.started && htim4.started);
    CHECK(htim1.cnt == 0 && htim2.cnt == 0 && htim3.cnt == 0 && htim4.cnt == 0);

    puts("[3] update: 累計與溢位（模擬硬體計數器）");
    tick = 10;
    htim2.cnt = 0xFFFFFFFFu;  /* pod1 32-bit 往回 1 */
    htim3.cnt = 65535;        /* pod2 16-bit 往回 1 */
    htim1.cnt = 300;          /* pod3 前進 300 */
    htim4.cnt = 0;
    odom_update();
    tick = 20;
    htim2.cnt = 4;            /* 再前進 5 */
    htim3.cnt = 9;            /* 從 65535 繞回到 9 = +10 */
    htim1.cnt = 100;          /* 後退 200 */
    htim4.cnt = 65000;        /* 後退 536 */
    odom_update();
    OdomSnapshot s;
    odom_get_snapshot(&s);
    CHECK(s.timestamp_ms == 20);
    CHECK(s.total[ODOM_POD_1] == 4    && s.delta[ODOM_POD_1] == 5);
    CHECK(s.total[ODOM_POD_2] == 9    && s.delta[ODOM_POD_2] == 10);
    CHECK(s.total[ODOM_POD_3] == 100  && s.delta[ODOM_POD_3] == -200);
    CHECK(s.total[ODOM_POD_4] == -536 && s.delta[ODOM_POD_4] == -536);

    puts("[4] telemetry: CSV 格式");
    telemetry_send_odom(&s);
    printf("  output: %s", uart_out);
    CHECK(strncmp(uart_out, "0,20,4,9,100,-536,5,10,-200,-536,", 33) == 0);
    int commas = 0; for (char *p = uart_out; *p; p++) commas += (*p == ',');
    CHECK(commas == 13);   /* 14 個欄位 */
    CHECK(strstr(uart_out, "\r\n") != NULL);
    telemetry_send_odom(&s);
    CHECK(uart_out[0] == '1');   /* seq 遞增 */

    printf("\n%s (%d failed)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
    return fails != 0;
}
