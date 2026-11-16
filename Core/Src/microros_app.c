/*
 * microros_app.c
 *
 * micro-ROS 連線狀態機：
 *
 *   WAITING ──ping 成功、建立 node/publisher──► CONNECTED
 *      ▲                                          │
 *      └──── 刪除 node/publisher ◄── DISCONNECTED ◄┘ ping 失敗（USB 拔掉、agent 關掉）
 *
 * 不管板子和 agent 誰先開，或中途斷線，都會自動重新連上，不需要按 RESET。
 */
#include "microros_app.h"

#include "main.h"
#include "cmsis_os.h"

#include <stdbool.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <std_msgs/msg/int32.h>

#define MICROROS_NODE_NAME        "odom_stm32"
#define MICROROS_TEST_TOPIC       "stm32_test"
#define MICROROS_PUBLISH_PERIOD_MS  100U
#define MICROROS_PING_PERIOD_MS    1000U   /* 連線中多久確認一次 agent 還在 */
#define MICROROS_RETRY_PERIOD_MS    500U   /* 等待 agent 時多久試一次 */

/* 刻意忽略回傳值。rcl 的函式標了 warn_unused_result，(void) 轉型壓不掉警告 */
#define IGNORE_RESULT(expr) do { rcl_ret_t ignored_ = (expr); (void)ignored_; } while (0)

extern UART_HandleTypeDef huart2;

/* micro_ros_stm32cubemx_utils 提供：dma_transport.c */
bool   cubemx_transport_open(struct uxrCustomTransport *transport);
bool   cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

/* micro_ros_stm32cubemx_utils 提供：microros_allocators.c */
void *microros_allocate(size_t size, void *state);
void  microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

typedef enum {
    AGENT_WAITING,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED,
} AgentState;

static rclc_support_t      support;
static rcl_node_t          node;
static rcl_publisher_t     test_pub;
static std_msgs__msg__Int32 test_msg;

static bool entities_create(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    node     = rcl_get_zero_initialized_node();
    test_pub = rcl_get_zero_initialized_publisher();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }
    if (rclc_node_init_default(&node, MICROROS_NODE_NAME, "", &support) != RCL_RET_OK) {
        return false;
    }
    /* best effort：送不出去就丟掉，不會為了重送而卡住 */
    if (rclc_publisher_init_best_effort(&test_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            MICROROS_TEST_TOPIC) != RCL_RET_OK) {
        return false;
    }
    return true;
}

static void entities_destroy(void)
{
    /* agent 已經不在了，不要等它回應刪除請求 */
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    /* 斷線時刪除一定會回報錯誤（agent 收不到），忽略即可，本地的記憶體照樣會釋放 */
    IGNORE_RESULT(rcl_publisher_fini(&test_pub, &node));
    IGNORE_RESULT(rcl_node_fini(&node));
    IGNORE_RESULT(rclc_support_fini(&support));
}

static void setup_transport_and_allocator(void)
{
    rmw_uros_set_custom_transport(true, (void *)&huart2,
                                  cubemx_transport_open,
                                  cubemx_transport_close,
                                  cubemx_transport_write,
                                  cubemx_transport_read);

    rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate      = microros_allocate;
    allocator.deallocate    = microros_deallocate;
    allocator.reallocate    = microros_reallocate;
    allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&allocator)) {
        Error_Handler();
    }
}

void microros_app_task(void)
{
    setup_transport_and_allocator();

    AgentState state = AGENT_WAITING;
    uint32_t last_ping_ms = 0;

    for (;;) {
        switch (state) {
        case AGENT_WAITING:
            BSP_LED_Off(LED2);
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                if (entities_create()) {
                    test_msg.data = 0;
                    last_ping_ms = HAL_GetTick();
                    state = AGENT_CONNECTED;
                } else {
                    entities_destroy();
                }
            } else {
                osDelay(MICROROS_RETRY_PERIOD_MS);
            }
            break;

        case AGENT_CONNECTED:
            BSP_LED_On(LED2);
            if (HAL_GetTick() - last_ping_ms >= MICROROS_PING_PERIOD_MS) {
                last_ping_ms = HAL_GetTick();
                if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                    break;
                }
            }
            /* best effort：這一筆送不出去就算了，斷線由上面的 ping 偵測 */
            IGNORE_RESULT(rcl_publish(&test_pub, &test_msg, NULL));
            test_msg.data++;
            osDelay(MICROROS_PUBLISH_PERIOD_MS);
            break;

        case AGENT_DISCONNECTED:
        default:
            entities_destroy();
            state = AGENT_WAITING;
            break;
        }
    }
}
