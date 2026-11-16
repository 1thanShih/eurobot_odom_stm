/**
 * @file    app.h
 * @brief   程式進入點，main.c 只呼叫這兩個函式。
 */
#ifndef APP_H
#define APP_H

/** 開機時呼叫一次（FreeRTOS 啟動前）。 */
void app_init(void);

/** 在 FreeRTOS 的 defaultTask 裡執行，不會返回。 */
void app_task(void);

#endif /* APP_H */
