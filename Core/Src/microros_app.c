/**
 * @file    microros_app.c
 * @brief   用 micro-ROS 把 encoder 累計 count（/wheel_counts）和 IMU 原始資料（/imu/data_raw、/imu/mag）發布到 ROS 2。
 *
 * STM32 經 Type-C（USB CDC 虛擬 COM 埠）和電腦上的 micro-ROS agent 溝通，
 * agent 再把資料轉進 ROS 2 網路。訊息格式見 microros_app.h。
 *
 * 連線狀態：
 *   WAITING ──ping 到 agent，建立 node/publisher──▶ CONNECTED
 *      ▲                                               │ ping 不到
 *      └──────────── 刪除 node/publisher ◀── DISCONNECTED
 *
 * 函式：
 *   setup_transport_and_allocator()  開機時做一次：告訴 micro-ROS 用 USB CDC 傳資料
 *                                    （Core/Src/microros/usb_cdc_transport.c），
 *                                    記憶體配置用自訂的 allocator，並把 msg.data 指向固定陣列。
 *   entities_create()                連上 agent 後建立 support（domain ID = ROS_DOMAIN_ID）、
 *                                    node（odom_stm32）和三個 best-effort publisher。
 *                                    任何一步失敗就回傳 false。
 *   entities_destroy()               agent 斷線時釋放 publisher、node、support。
 *                                    不等 agent 回應，失敗也照樣釋放本地記憶體。
 *   publish_counts()                 取 odometry 快照，填成 [stamp, pod1~4] 後發布一筆。
 *   imu_msg_init()                   開機時填好 IMU 訊息不會變的欄位（frame_id、共變異數）。
 *   publish_imu()                    IMU 有新的一筆才發布 /imu/data_raw 和 /imu/mag，沒接 IMU 就不發。
 *   microros_app_task()              主迴圈，不會返回。每 ODOM_SAMPLE_PERIOD_MS 執行一次：
 *                                    先 odom_update()，再依連線狀態找 agent、發布資料或清理，
 *                                    板上 LED（PC13）亮 = 已連線。
 */
#include "microros_app.h"

#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"
#include "odometry.h"
#include "imu.h"
#include "robot_config.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#define NODE_NAME          "odom_stm32"
#define TOPIC_NAME         "wheel_counts"
#define IMU_TOPIC_NAME     "imu/data_raw"   /* imu_tools（madgwick 等）預設吃的名字 */
#define MAG_TOPIC_NAME     "imu/mag"
#define IMU_FRAME_ID       "imu_link"
#define ROS_DOMAIN_ID      4U      /* 主機端 ROS_DOMAIN_ID 要設成一樣才看得到 topic */
#define MSG_LEN            (1 + ODOM_POD_COUNT)   /* stamp + 4 個 count */
#define RETRY_PERIOD_MS    500U    /* 等待時多久找一次 agent */
#define PING_PERIOD_MS     1000U   /* 連線中多久確認一次 agent 還在 */

/* rcl 函式有 warn_unused_result，(void) 壓不掉，刻意忽略時用這個 */
#define IGNORE_RESULT(expr) do { rcl_ret_t ignored_ = (expr); (void)ignored_; } while (0)

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

static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__Imu imu_msg;
static rcl_publisher_t mag_publisher;
static sensor_msgs__msg__MagneticField mag_msg;
static uint32_t        imu_last_seq;   /* 上次發布的 IMU 序號，用來判斷有沒有新資料 */
static char            imu_frame_id[] = IMU_FRAME_ID;

static void imu_msg_init(void)
{
    /* frame_id 指向固定字串，不需要動態配置 */
    imu_msg.header.frame_id.data     = imu_frame_id;
    imu_msg.header.frame_id.size     = sizeof(imu_frame_id) - 1;
    imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
    mag_msg.header.frame_id          = imu_msg.header.frame_id;

    /* 沒有姿態：依 sensor_msgs/Imu 的慣例，orientation_covariance[0] = -1 */
    imu_msg.orientation_covariance[0] = -1.0;

    /* 共變異數只填對角線（0、4、8），其他是 0；磁場的共變異數全 0 = 未知 */
    for (int i = 0; i < 3; i++) {
        imu_msg.angular_velocity_covariance[i * 4]    = IMU_GYRO_VAR;
        imu_msg.linear_acceleration_covariance[i * 4] = IMU_ACCEL_VAR;
    }
}

static void setup_transport_and_allocator(void)
{
    rmw_uros_set_custom_transport(true, NULL,
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

    imu_msg_init();
}

static bool entities_create(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    node          = rcl_get_zero_initialized_node();
    publisher     = rcl_get_zero_initialized_publisher();
    imu_publisher = rcl_get_zero_initialized_publisher();
    mag_publisher = rcl_get_zero_initialized_publisher();

    /* 設定 domain ID；rcl_init 會複製一份 options，用完就可以釋放 */
    rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_opts, allocator) != RCL_RET_OK) {
        return false;
    }
    rcl_ret_t ret = rcl_init_options_set_domain_id(&init_opts, ROS_DOMAIN_ID);
    if (ret == RCL_RET_OK) {
        ret = rclc_support_init_with_options(&support, 0, NULL, &init_opts, &allocator);
    }
    IGNORE_RESULT(rcl_init_options_fini(&init_opts));
    if (ret != RCL_RET_OK) {
        return false;
    }
    if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) {
        return false;
    }
    /* best effort：送不出去就丟掉，不會卡住 */
    if (rclc_publisher_init_best_effort(&publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            TOPIC_NAME) != RCL_RET_OK) {
        return false;
    }
    if (rclc_publisher_init_best_effort(&imu_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            IMU_TOPIC_NAME) != RCL_RET_OK) {
        return false;
    }
    return rclc_publisher_init_best_effort(&mag_publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
               MAG_TOPIC_NAME) == RCL_RET_OK;
}

static void entities_destroy(void)
{
    /* agent 已經不在，不要等它回應 */
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    /* 斷線時一定回報失敗，忽略即可，本地記憶體照樣會釋放 */
    IGNORE_RESULT(rcl_publisher_fini(&mag_publisher, &node));
    IGNORE_RESULT(rcl_publisher_fini(&imu_publisher, &node));
    IGNORE_RESULT(rcl_publisher_fini(&publisher, &node));
    IGNORE_RESULT(rcl_node_fini(&node));
    IGNORE_RESULT(rclc_support_fini(&support));
}

static void publish_counts(const OdomSnapshot *snap)
{
    msg_data[0] = (int32_t)snap->stamp_ms;   /* 約 24.8 天後會變負數，比賽用不到 */
    for (int i = 0; i < ODOM_POD_COUNT; i++) {
        msg_data[1 + i] = snap->total[i];
    }
    IGNORE_RESULT(rcl_publish(&publisher, &msg, NULL));
}

static void publish_imu(void)
{
    ImuSnapshot snap;
    imu_get_snapshot(&snap);
    if (!snap.valid || snap.seq == imu_last_seq) {
        return;   /* 沒接 IMU，或還沒有新的一筆 */
    }
    imu_last_seq = snap.seq;

    /* stamp 和同一週期 /wheel_counts 的 data[0] 完全相同（STM32 開機後的毫秒數） */
    imu_msg.header.stamp.sec     = (int32_t)(snap.stamp_ms / 1000U);
    imu_msg.header.stamp.nanosec = (snap.stamp_ms % 1000U) * 1000000U;

    imu_msg.angular_velocity.x = snap.gyro[0];
    imu_msg.angular_velocity.y = snap.gyro[1];
    imu_msg.angular_velocity.z = snap.gyro[2];

    imu_msg.linear_acceleration.x = snap.accel[0];
    imu_msg.linear_acceleration.y = snap.accel[1];
    imu_msg.linear_acceleration.z = snap.accel[2];

    IGNORE_RESULT(rcl_publish(&imu_publisher, &imu_msg, NULL));

    mag_msg.header.stamp     = imu_msg.header.stamp;
    mag_msg.magnetic_field.x = snap.mag[0];
    mag_msg.magnetic_field.y = snap.mag[1];
    mag_msg.magnetic_field.z = snap.mag[2];
    IGNORE_RESULT(rcl_publish(&mag_publisher, &mag_msg, NULL));
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
        OdomSnapshot odom;
        odom_get_snapshot(&odom);
        imu_request_sample(odom.stamp_ms);   /* IMU task 優先權較高，會馬上讀完才回來 */
        uint32_t now_ms = HAL_GetTick();

        switch (state) {
        case AGENT_WAITING:
            LED_OFF();
            if (now_ms - last_check_ms >= RETRY_PERIOD_MS) {
                last_check_ms = now_ms;
                if (rmw_uros_ping_agent(10, 1) == RMW_RET_OK) {
                    state = entities_create() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                }
            }
            break;

        case AGENT_CONNECTED:
            LED_ON();
            publish_counts(&odom);
            publish_imu();
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
