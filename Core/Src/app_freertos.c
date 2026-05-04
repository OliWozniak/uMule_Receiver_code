/* app_freertos.c
 *
 * micro-ROS task:
 *   - Subskrybuje /cmd_vel (geometry_msgs/Twist)
 *   - Przelicza linear.x i angular.z na prędkości kół i wrzuca do kolejki
 *   - Publikuje /wheel_state (std_msgs/msg/Float32MultiArray) co ~100ms
 *
 * Połączenie z agentem micro-ROS przez USART1 (Half-Duplex lub pełny duplex).
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
#include "rx_64_comm.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE END PTD */

// ---------------------------------------------------------------------------
// Parametry kinematyki robota różnicowego
// Dostosuj do swoich kół i rozstawu!
// ---------------------------------------------------------------------------
#define WHEEL_SEPARATION_M   0.30f   // rozstaw kół [m]
#define WHEEL_RADIUS_M       0.08f   // promień koła [m]
// Prędkość max serwomechanizmu RX-64 w trybie koła = ~114 RPM
// speed% = 100 odpowiada ~114 RPM = ~0.955 m/s przy r=0.08m
// Przelicznik: speed% = (omega_rad_s * r / (114 * 2*pi/60)) * 100
#define MAX_RPM              114.0f
#define RPM_TO_RAD_S         (2.0f * 3.14159f / 60.0f)
#define MAX_WHEEL_SPEED_MS   (MAX_RPM * RPM_TO_RAD_S * WHEEL_RADIUS_M)
// speed% = (v_ms / MAX_WHEEL_SPEED_MS) * 100
#define MPS_TO_SPEED_PCT(v)  ((v) / MAX_WHEEL_SPEED_MS * 100.0f)

// ---------------------------------------------------------------------------
// Stała do klamrowania wartości
// ---------------------------------------------------------------------------
static float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ---------------------------------------------------------------------------
// Wątek defaultTask (micro-ROS)
// ---------------------------------------------------------------------------

/* Statyczna alokacja stosu */
uint32_t defaultTaskBuffer[6000];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
    .name      = "defaultTask",
    .stack_mem = &defaultTaskBuffer[0],
    .stack_size = sizeof(defaultTaskBuffer),
    .cb_mem    = &defaultTaskControlBlock,
    .cb_size   = sizeof(defaultTaskControlBlock),
    .priority  = (osPriority_t) osPriorityNormal,
};
osThreadId_t defaultTaskHandle;

/* Deklaracje transportu micro-ROS (implementacja w micro_ros_stm32cubemx_utils) */
bool   cubemx_transport_open (struct uxrCustomTransport *transport);
bool   cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                               const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read (struct uxrCustomTransport *transport,
                               uint8_t *buf, size_t len, int timeout, uint8_t *err);

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
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    float linear  = (float)msg->linear.x;   // [m/s]
    float angular = (float)msg->angular.z;  // [rad/s]

    // Kinematyka robota różnicowego:
    // v_right = linear + angular * (L/2)
    // v_left  = linear - angular * (L/2)
    float v_right_ms = linear + angular * (WHEEL_SEPARATION_M / 2.0f);
    float v_left_ms  = linear - angular * (WHEEL_SEPARATION_M / 2.0f);

    // Przelicz na procent prędkości i zaklamruj do [-100, 100]
    float speed_right = clamp(MPS_TO_SPEED_PCT(v_right_ms), -100.0f, 100.0f);
    float speed_left  = clamp(MPS_TO_SPEED_PCT(v_left_ms),  -100.0f, 100.0f);

    DynamixelCommand_t cmd;

    cmd.id    = 1;  // prawe koło
    cmd.speed = speed_right;
    osMessageQueuePut(dynamixelCmdQueueHandle, &cmd, 0, 0);

    cmd.id    = 2;  // lewe koło
    cmd.speed = speed_left;
    osMessageQueuePut(dynamixelCmdQueueHandle, &cmd, 0, 0);
}

// ---------------------------------------------------------------------------
// Makro pomocnicze: sprawdź wynik rcl i zablokuj przy błędzie
// ---------------------------------------------------------------------------
#define RCCHECK(fn) \
    { rcl_ret_t _rc = (fn); \
      if (_rc != RCL_RET_OK) { \
        printf("micro-ROS error %d at line %d\n", (int)_rc, __LINE__); \
        while(1) { osDelay(1000); } \
      } \
    }

// ---------------------------------------------------------------------------
// Wątek micro-ROS
// ---------------------------------------------------------------------------
void StartDefaultTask(void *argument)
{
    /* --- Transport --- */
    rmw_uros_set_custom_transport(
        true,
        (void *)&huart1,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    /* --- Alokator FreeRTOS --- */
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error setting allocator (line %d)\n", __LINE__);
    }

    /* --- Czekaj na agenta micro-ROS --- */
    printf("Waiting for micro-ROS agent...\n");
    while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        osDelay(500);
    }
    printf("Agent found!\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1); // LED stale świeci = połączono

    /* --- Inicjalizacja węzła --- */
    rclc_support_t  support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_node_t      node;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "stm32_wheel_node", "", &support));

    /* --- Publisher: /wheel_state (Float32MultiArray, 6 floatów) ---
     *  [0] right_position [deg]
     *  [1] right_velocity [RPM]
     *  [2] right_load     [%]
     *  [3] left_position  [deg]
     *  [4] left_velocity  [RPM]
     *  [5] left_load      [%]
     */
    rcl_publisher_t wheel_pub;
    std_msgs__msg__Float32MultiArray wheel_state_msg;

    // Alokacja tablicy danych
    static float wheel_data[6] = {0};
    wheel_state_msg.data.data     = wheel_data;
    wheel_state_msg.data.size     = 6;
    wheel_state_msg.data.capacity = 6;

    RCCHECK(rclc_publisher_init_default(
        &wheel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_state"));

    /* --- Subscriber: /cmd_vel (geometry_msgs/Twist) --- */
    rcl_subscription_t cmd_vel_sub;
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    /* --- Executor (1 subskrypcja) --- */
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        cmd_vel_callback, ON_NEW_DATA));

    WheelStateFeedback_t feedback;
    uint32_t last_pub_ms = 0;

    printf("micro-ROS node ready.\n");

    for (;;)
    {
        /* Obsługa subskrypcji (nieblokująca, timeout=0) */
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

        /* Publikacja feedbacku ze stanu kół (dane z kolejki od DynamixelTask) */
        uint32_t now = osKernelGetTickCount();
        if ((now - last_pub_ms) >= 100) {   // co 100ms
            last_pub_ms = now;

            if (osMessageQueueGet(wheelFeedbackQueueHandle,
                                  &feedback, NULL, 0) == osOK) {
                for (int i = 0; i < 6; i++) {
                    wheel_data[i] = feedback.data[i];
                }
                rcl_ret_t ret = rcl_publish(&wheel_pub, &wheel_state_msg, NULL);
                if (ret != RCL_RET_OK) {
                    printf("Publish error %d\n", (int)ret);
                }
            }
        }

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); // LED mruga = micro-ROS żyje
        osDelay(10);
    }
}

// ---------------------------------------------------------------------------
// Inicjalizacja FreeRTOS (wywoływana z main.c)
// ---------------------------------------------------------------------------
void MX_FREERTOS_Init(void)
{
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
                                    &defaultTask_attributes);
}
