/**
 * @file    app.h
 * @brief   應用程式進入點，相當於 Arduino 的 setup() / loop()。
 *
 * main.c 由 CubeMX 產生，只負責硬體初始化，然後呼叫這兩個函式。
 * 我們自己的程式都從這裡開始，新增功能時不需要修改 main.c。
 */
#ifndef APP_H
#define APP_H

/** 開機時呼叫一次。此時 CubeMX 的 MX_*_Init() 都已經執行完畢。 */
void app_init(void);

/** 在 main() 的 while(1) 裡不斷被呼叫。不要在裡面阻塞太久。 */
void app_loop(void);

#endif /* APP_H */
