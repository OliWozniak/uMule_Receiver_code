/* app_freertos.c */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "dxl_control.h" // Nasz nowy interfejs



/* micro-ROS headers */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

/* --- DEKLARACJE (To rozwiązuje błędy 'undeclared') --- */
void StartMicroROSTask(void *argument);
void cmd_vel_callback(const void * msin);

// Deklaracje funkcji transportowych micro-ROS
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// Deklaracje allocatorów
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


// Atrybuty taska micro-ROS
const osThreadAttr_t defaultTask_attributes = {
  .name = "micro_ros_task",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* --- Zmienne globalne micro-ROS --- */
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t wheel_pub;
std_msgs__msg__Float32MultiArray wheel_state_msg;
float wheel_data[6];

void MX_FREERTOS_Init(void) {
    /* 1. Inicjalizacja sprzętu Dynamixel */
    DXL_Manager_Init();

    /* 2. Utworzenie kolejek */
    dxl_cmd_queue = osMessageQueueNew(10, sizeof(DXL_RosCommand_t), NULL);
    dxl_feedback_queue = osMessageQueueNew(10, sizeof(DXL_RosFeedback_t), NULL);

    /* 3. Utworzenie Tasków */
    osThreadNew(DXL_Manager_Task, NULL, &dxl_task_attr);
}

// --- Callback dla /cmd_vel ---
void cmd_vel_callback(const void * msin) {
    // Używamy pełnej nazwy typu z podwójnym podkreśleniem __
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msin;

    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;
    float wheel_separation = 0.4f;

    float v_r = linear_x + (angular_z * wheel_separation / 2.0f);
    float v_l = linear_x - (angular_z * wheel_separation / 2.0f);

    float max_vel = 0.5f;

    DXL_RosCommand_t cmd_r = { .id = ID_RIGHT_WHEEL, .speed_pct = (v_r / max_vel) * 100.0f };
    DXL_RosCommand_t cmd_l = { .id = ID_LEFT_WHEEL, .speed_pct = (v_l / max_vel) * 100.0f };

    osMessageQueuePut(dxl_cmd_queue, &cmd_r, 0, 0);
    osMessageQueuePut(dxl_cmd_queue, &cmd_l, 0, 0);
}

