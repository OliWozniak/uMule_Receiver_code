/* app_freertos.c
 *
 * micro-ROS task:
 * - Subskrybuje /cmd_vel (geometry_msgs/Twist)
 * - Przelicza linear.x i angular.z na prędkości kół i wrzuca do kolejki
 * - Publikuje /wheel_state (std_msgs/msg/Float32MultiArray) co ~100ms
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "usart.h"
#include "dxl_control.h"  // Zawiera definicje DXL_RosCommand_t, dxl_cmd_queue itp.
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE END PTD */

// ---------------------------------------------------------------------------
// Parametry kinematyki robota różnicowego
// ---------------------------------------------------------------------------
#define WHEEL_SEPARATION_M   0.30f   // rozstaw kół [m]
#define WHEEL_RADIUS_M       0.08f   // promień koła [m]
#define MAX_RPM              114.0f
#define RPM_TO_RAD_S         (2.0f * 3.14159f / 60.0f)
#define MAX_WHEEL_SPEED_MS   (MAX_RPM * RPM_TO_RAD_S * WHEEL_RADIUS_M)
#define MPS_TO_SPEED_PCT(v)  ((v) / MAX_WHEEL_SPEED_MS * 100.0f)

static float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ---------------------------------------------------------------------------
// Wątek defaultTask (micro-ROS)
// ---------------------------------------------------------------------------

uint32_t defaultTaskBuffer[6000];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_mem = &defaultTaskBuffer[0],
    .stack_size = sizeof(defaultTaskBuffer),
    .cb_mem    = &defaultTaskControlBlock,
    .cb_size   = sizeof(defaultTaskControlBlock),
    .priority  = (osPriority_t) osPriorityNormal,
};
osThreadId_t defaultTaskHandle;

/* Prototypy transportu (wymagane przez micro-ROS) */
bool   cubemx_transport_open (struct uxrCustomTransport *transport);
bool   cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read (struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate      (size_t size, void *state);
void  microros_deallocate    (void *pointer, void *state);
void *microros_reallocate    (void *pointer, size_t size, void *state);
void *microros_zero_allocate (size_t n, size_t size, void *state);

// ---------------------------------------------------------------------------
// Callback subskrypcji /cmd_vel
// ---------------------------------------------------------------------------
static geometry_msgs__msg__Twist cmd_vel_msg;

static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear  = (float)msg->linear.x;
    float angular = (float)msg->angular.z;

    float v_right_ms = linear + angular * (WHEEL_SEPARATION_M / 2.0f);
    float v_left_ms  = linear - angular * (WHEEL_SEPARATION_M / 2.0f);

    DXL_RosCommand_t cmd;

    // Prawe koło
    cmd.id = ID_RIGHT_WHEEL;
    cmd.speed_pct = clamp(MPS_TO_SPEED_PCT(v_right_ms), -100.0f, 100.0f);
    osMessageQueuePut(dxl_cmd_queue, &cmd, 0, 0);

    // Lewe koło
    cmd.id = ID_LEFT_WHEEL;
    cmd.speed_pct = clamp(MPS_TO_SPEED_PCT(v_left_ms), -100.0f, 100.0f);
    osMessageQueuePut(dxl_cmd_queue, &cmd, 0, 0);
}

#define RCCHECK(fn) { rcl_ret_t _rc = (fn); if (_rc != RCL_RET_OK) { while(1) { osDelay(1000); } } }

void StartDefaultTask(void *argument)
{
    /* --- Transport --- */
    rmw_uros_set_custom_transport(
        true, (void *)&huart1,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    /* --- Alokator --- */
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    rcutils_set_default_allocator(&freeRTOS_allocator);

    /* --- Połączenie z agentem --- */
    while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        osDelay(500);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);

    /* --- Inicjalizacja ROS 2 --- */
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t node;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "stm32_robot_node", "", &support));

    /* --- Publisher --- */
    rcl_publisher_t wheel_pub;
    std_msgs__msg__Float32MultiArray wheel_state_msg;
    static float wheel_data[6] = {0};
    wheel_state_msg.data.data = wheel_data;
    wheel_state_msg.data.size = 6;
    wheel_state_msg.data.capacity = 6;

    RCCHECK(rclc_publisher_init_default(
        &wheel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_state"));

    /* --- Subscriber --- */
    rcl_subscription_t cmd_vel_sub;
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    /* --- Executor --- */
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        cmd_vel_callback, ON_NEW_DATA));

    DXL_RosFeedback_t feedback;
    uint32_t last_pub_ms = 0;

    for (;;)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

        uint32_t now = osKernelGetTickCount();
        if ((now - last_pub_ms) >= 100) {
            last_pub_ms = now;

            // Pobieranie danych z kolejki wystawionej przez dxl_control.c
            if (osMessageQueueGet(dxl_feedback_queue, &feedback, NULL, 0) == osOK) {
                for (int i = 0; i < 6; i++) {
                    wheel_data[i] = feedback.data[i];
                }
                rcl_publish(&wheel_pub, &wheel_state_msg, NULL);
            }
        }

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        osDelay(10);
    }
}

void MX_FREERTOS_Init(void)
{
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}
