/*
 * ===== 中文說明 =====
 * micro-ROS 的自訂傳輸層：用 Black Pill 的 Type-C（USB CDC 虛擬 COM 埠）和電腦上的
 * micro-ROS agent 收送資料。電腦端會看到 /dev/ttyACM0，鮑率設定沒有作用。
 * 改寫自 micro_ros_stm32cubemx_utils 的 usb_cdc_transport.c，在 microros_app.c 用
 * rmw_uros_set_custom_transport() 註冊。USB 本身由 CubeMX 產生的 MX_USB_DEVICE_Init()
 * 在 StartDefaultTask 開頭啟動，這裡只把 CDC 的 callback 換成自己的。
 *
 * 和原範例的差別：
 *   - 修正環形 buffer 繞回時第二段從錯的位置複製
 *   - buffer 滿了就丟掉新資料，不覆蓋還沒讀的
 *   - 送出改成非阻塞：資料先放進 TX 環形 buffer 就返回，USB 中斷送完一包再接著送下一包，
 *     不用每筆訊息都等電腦收完（每週期要送好幾筆訊息，逐筆等會超過 10 ms）
 *   - USB 還沒被電腦設定好（沒插線、agent 沒開）就直接回傳 0，不會卡住
 *
 * 函式：
 *   cdc_control()             電腦設定/讀取鮑率等參數時呼叫，只把值存起來再回傳。
 *   cdc_receive()             USB 中斷裡呼叫：把收到的資料放進環形 buffer，再準備收下一包。
 *   cdc_transmit_cplt()       USB 中斷裡呼叫：上一包送完了，從 TX buffer 移掉，再送下一包。
 *   tx_kick()                 USB 閒著而且 TX buffer 有資料，就把連續的一段交給 USB 送出。
 *   cubemx_transport_open()   把 CDC 的 callback 換成上面三個。
 *   cubemx_transport_close()  不用做事。
 *   cubemx_transport_write()  整筆放進 TX buffer（空間不夠最多等 WRITE_TIMEOUT_MS，還是不夠就整筆丟掉），
 *                             再叫 tx_kick() 開始送。
 *   cubemx_transport_read()   每 1 ms 看一次有沒有新資料，最多等 timeout 毫秒；
 *                             有資料就從環形 buffer 複製最多 len 個 byte，回傳實際數量。
 */
#include <uxr/client/transport.h>
#include <uxr/client/util/time.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"

#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

#define RX_BUFFER_SIZE    2048U
#define TX_BUFFER_SIZE    2048U
#define WRITE_TIMEOUT_MS  10       /* TX buffer 滿了（電腦沒在讀）時最多等多久 */

extern USBD_HandleTypeDef  hUsbDeviceFS;

/* 環形 buffer：ISR 寫 rx_tail，task 讀 rx_head */
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile size_t  rx_head = 0;
static volatile size_t  rx_tail = 0;

/* TX 環形 buffer：task 寫 tx_head，USB 中斷移動 tx_tail。
 * 兩邊都只在中斷被擋住時（task 用 critical section）修改，所以不會互相打架。 */
static uint8_t          tx_buffer[TX_BUFFER_SIZE];
static volatile size_t  tx_head = 0;
static volatile size_t  tx_tail = 0;
static volatile size_t  tx_inflight = 0;   /* 正在送的長度，0 = USB 閒著 */

/* 115200 bps、1 stop bit、無同位、8 bits（USB CDC 用不到，只是讓電腦讀得到合理的值） */
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

static int8_t cdc_control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;
    if (cmd == CDC_SET_LINE_CODING) {
        memcpy(line_coding, pbuf, sizeof(line_coding));
    } else if (cmd == CDC_GET_LINE_CODING) {
        memcpy(pbuf, line_coding, sizeof(line_coding));
    }
    return USBD_OK;
}

static int8_t cdc_receive(uint8_t *buf, uint32_t *len)
{
    size_t tail = rx_tail;
    for (uint32_t i = 0; i < *len; i++) {
        size_t next = (tail + 1U) % RX_BUFFER_SIZE;
        if (next == rx_head) {
            break;   /* 滿了，丟掉剩下的 */
        }
        rx_buffer[tail] = buf[i];
        tail = next;
    }
    rx_tail = tail;

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

/* 呼叫時中斷要被擋住（USB 中斷裡，或 task 的 critical section 裡） */
static void tx_kick(void)
{
    if (tx_inflight != 0U || tx_head == tx_tail) {
        return;
    }
    /* 只送連續的一段，繞回的部分等下一次 */
    size_t n = (tx_head > tx_tail) ? (tx_head - tx_tail) : (TX_BUFFER_SIZE - tx_tail);
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &tx_buffer[tx_tail], (uint32_t)n);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK) {
        tx_inflight = n;
    }
}

static int8_t cdc_transmit_cplt(uint8_t *buf, uint32_t *len, uint8_t epnum)
{
    (void)buf;
    (void)len;
    (void)epnum;
    tx_tail = (tx_tail + tx_inflight) % TX_BUFFER_SIZE;
    tx_inflight = 0;
    tx_kick();
    return USBD_OK;
}

static size_t tx_free(void)
{
    return (tx_tail + TX_BUFFER_SIZE - tx_head - 1U) % TX_BUFFER_SIZE;
}

/* USB 已被電腦設定好；重新列舉過（送到一半被重設）就清空 TX buffer */
static bool usb_configured(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    bool ok = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) && (hcdc != NULL);

    taskENTER_CRITICAL();
    if (!ok || (tx_inflight != 0U && hcdc->TxState == 0U)) {
        tx_head = tx_tail = tx_inflight = 0;
    }
    taskEXIT_CRITICAL();
    return ok;
}

bool cubemx_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    USBD_Interface_fops_FS.Control      = cdc_control;
    USBD_Interface_fops_FS.Receive      = cdc_receive;
    USBD_Interface_fops_FS.TransmitCplt = cdc_transmit_cplt;
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;
    (void)err;
    if (len >= TX_BUFFER_SIZE || !usb_configured()) {
        return 0;
    }

    /* 整筆放得下才放，只放一半會讓 agent 收到壞掉的封包 */
    int64_t start = uxr_millis();
    while (tx_free() < len) {
        if ((uxr_millis() - start) >= WRITE_TIMEOUT_MS) {
            return 0;
        }
        osDelay(1);
    }

    size_t head  = tx_head;
    size_t first = TX_BUFFER_SIZE - head;
    if (first > len) {
        first = len;
    }
    memcpy(&tx_buffer[head], buf, first);
    memcpy(tx_buffer, &buf[first], len - first);

    taskENTER_CRITICAL();
    tx_head = (head + len) % TX_BUFFER_SIZE;
    tx_kick();
    taskEXIT_CRITICAL();
    return len;
}

size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)transport;
    (void)err;
    int64_t start = uxr_millis();
    size_t  got   = 0;

    do {
        while ((rx_head != rx_tail) && (got < len)) {
            buf[got++] = rx_buffer[rx_head];
            rx_head = (rx_head + 1U) % RX_BUFFER_SIZE;
        }
        if (got > 0U) {
            break;
        }
        osDelay(1);
    } while ((uxr_millis() - start) < timeout);

    return got;
}

#endif /* RMW_UXRCE_TRANSPORT_CUSTOM */
