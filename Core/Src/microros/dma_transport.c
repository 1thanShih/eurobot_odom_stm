/*
 * ===== 中文說明 =====
 * micro-ROS 的自訂傳輸層：用 UART + DMA 和電腦上的 micro-ROS agent 收送資料。
 * 來源是 micro_ros_stm32cubemx_utils 的範例，在 microros_app.c 用
 * rmw_uros_set_custom_transport() 註冊，UART handle（huart2）由 transport->args 傳進來。
 *
 * 接收的做法：DMA 設成 circular，在背景持續把收到的 byte 寫進 2048 bytes 的環形 buffer。
 * 讀取時看 DMA 剩餘計數就知道寫到哪裡（tail），再從上次讀到的位置（head）複製出來。
 * CPU 不用每個 byte 都處理中斷。
 *
 * 函式：
 *   cubemx_transport_open()   啟動 circular RX DMA，開始在背景收資料。
 *   cubemx_transport_close()  停止 UART 的 DMA。
 *   cubemx_transport_write()  用 TX DMA 送出一包，每 1 ms 檢查一次直到送完。
 *                             UART 正在忙就直接回傳 0（這包沒送出去）。
 *   cubemx_transport_read()   每 1 ms 看一次有沒有新資料，最多等 timeout 毫秒；
 *                             有資料就從環形 buffer 複製最多 len 個 byte，回傳實際複製的數量。
 */
#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    HAL_StatusTypeDef ret;
    if (uart->gState == HAL_UART_STATE_READY){
        ret = HAL_UART_Transmit_DMA(uart, buf, len);
        while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
            osDelay(1);
        }

        return (ret == HAL_OK) ? len : 0;
    }else{
        return 0;
    }
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    int ms_used = 0;
    do
    {
        __disable_irq();
        dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
        __enable_irq();
        ms_used++;
        osDelay(portTICK_RATE_MS);
    } while (dma_head == dma_tail && ms_used < timeout);
    
    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }
    
    return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM