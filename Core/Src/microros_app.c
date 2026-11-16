/**
 * @file    microros_app.c
 * @brief   見 microros_app.h。
 *
 * 連線狀態：
 *   WAITING ──ping 到 agent，建立 node/publisher──▶ CONNECTED
 *      ▲                                               │ ping 不到
 *      └──────────── 刪除 node/publisher ◀── DISCONNECTED
 */
#include "microros_app.h"

#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"
#include "odometry.h"
#include "robot_config.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <std_msgs/msg/int32_multi_array.h>

#define NODE_NAME          "odom_stm32"
#define TOPIC_NAME         "odom_counts"
#define MSG_LEN            (1 + ODOM_POD_COUNT)   /* stamp + 4 個 count */
#define RETRY_PERIOD_MS    500U    /* 等待時多久找一次 agent */
#define PING_PERIOD_MS     1000U   /* 連線中多久確認一次 agent 還在 */

/* rcl 函式有 warn_unused_result，(void) 壓不掉，刻意忽略時用這個 */
#define IGNORE_RESULT(expr) do { rcl_ret_t ignored_ = (expr); (void)ignored_; } while (0)

extern UART_HandleTypeDef huart2;

/* 由 Core/Src/microros/ 提供 */
bool   cubemx_transport_open(struct uxrCustomTransport *transport);
bool   cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
void  *microros_allocate(size_t size, void *state);
void   microros_deallocate(void *pointer, void *state);
void  *microros_reallocate(void *pointer, size_t size, void *state);
void  *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

typedef enum {
    AGENT_WAITING,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED,
} AgentState;

static rclc_support_t  support;
static rcl_node_t      node;
static rcl_publisher_t publisher;
static std_msgs__msg__Int32MultiArray msg;
static int32_t         msg_data[MSG_LEN];

static void setup_transport_and_allocator(void)
{
    rmw_uros_set_custom_transport(true, (void *)&huart2,
                                  cubemx_transport_open, cubemx_transport_close,
                                  cubemx_transport_write, cubemx_transport_read);

    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate      = microros_allocate;
    allocator.deallocate    = microros_deallocate;
    allocator.reallocate    = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&allocator)) {
        Error_Handler();
    }

    /* data 指向固定的陣列，不需要動態配置 */
    msg.data.data     = msg_data;
    msg.data.size     = MSG_LEN;
    msg.data.capacity = MSG_LEN;
}

static bool entities_create(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    node      = rcl_get_zero_initialized_node();
    publisher = rcl_get_zero_initialized_publisher();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }
    if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) {
        return false;
    }
    /* best effort：送不出去就丟掉，不會卡住 */
    return rclc_publisher_init_best_effort(&publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
               TOPIC_NAME) == RCL_RET_OK;
}

static void entities_destroy(void)
{
    /* agent 已經不在，不要等它回應 */
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    /* 斷線時一定回報失敗，忽略即可，本地記憶體照樣會釋放 */
    IGNORE_RESULT(rcl_publisher_fini(&publisher, &node));
    IGNORE_RESULT(rcl_node_fini(&node));
    IGNORE_RESULT(rclc_support_fini(&support));
}

static void publish_counts(void)
{
    OdomSnapshot snap;
    odom_get_snapshot(&snap);

    msg_data[0] = (int32_t)snap.stamp_ms;   /* 約 24.8 天後會變負數，比賽用不到 */
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        msg_data[1 + i] = snap.total[i];
    }
    IGNORE_RESULT(rcl_publish(&publisher, &msg, NULL));
}

void microros_app_task(void)
{
    setup_transport_and_allocator();

    AgentState state = AGENT_WAITING;
    uint32_t last_check_ms = 0;
    uint32_t next_wake = osKernelGetTickCount();   /* 1 tick = 1 ms */

    for (;;) {
        /* 沒連線也要照常讀，不然 16-bit 計數器繞回會漏算 */
        odom_update();
        uint32_t now_ms = HAL_GetTick();

        switch (state) {
        case AGENT_WAITING:
            BSP_LED_Off(LED2);
            if (now_ms - last_check_ms >= RETRY_PERIOD_MS) {
                last_check_ms = now_ms;
                if (rmw_uros_ping_agent(10, 1) == RMW_RET_OK) {
                    state = entities_create() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                }
            }
            break;

        case AGENT_CONNECTED:
            BSP_LED_On(LED2);
            publish_counts();
            if (now_ms - last_check_ms >= PING_PERIOD_MS) {
                last_check_ms = now_ms;
                if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                }
            }
            break;

        case AGENT_DISCONNECTED:
        default:
            entities_destroy();
            state = AGENT_WAITING;
            break;
        }

        /* 固定週期執行；如果剛才卡太久（例如建立連線），就從現在重新算，不要連發補回 */
        next_wake += ODOM_SAMPLE_PERIOD_MS;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0) {
            next_wake = osKernelGetTickCount();
        }
        osDelayUntil(next_wake);
    }
}
