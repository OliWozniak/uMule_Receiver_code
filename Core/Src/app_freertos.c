/* app_freertos.c
 *
 * micro-ROS task:
 * - Subskrybuje /wheel_commands (sensor_msgs/JointState) od topic_based_ros2_control
 * - Wysyla komendy predkosci do Dynamixeli przez kolejke FreeRTOS
 * - Publikuje /wheel_states (sensor_msgs/JointState) co ~100ms
 *
 * Kolejnosc jointow: [0]=left_wheel_joint, [1]=right_wheel_joint
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"   /* osStaticThreadDef_t i pozostale typy CMSIS-RTOS v2 ze static allocation */

#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joint_state.h>

#include "usart.h"
#include "dxl_control.h"

// ---------------------------------------------------------------------------
// KONFIGURACJA SIECI
// ROS_DOMAIN_ID - musi byc identyczne jak na komputerze hosta (export ROS_DOMAIN_ID=...)
// ---------------------------------------------------------------------------
#ifndef ROS_DOMAIN_ID
#define ROS_DOMAIN_ID   42        // <-- zmien na wartosc z `echo $ROS_DOMAIN_ID` na PC
#endif

// ---------------------------------------------------------------------------
// Parametry kinematyki robota
// Musza byc IDENTYCZNE z my_controllers.yaml i ros2_control.xacro
// ---------------------------------------------------------------------------
#define WHEEL_SEPARATION_M   0.30f
#define WHEEL_RADIUS_M       0.08f
#define MAX_RPM              114.0f
#define RPM_TO_RAD_S         (2.0f * 3.14159265f / 60.0f)
#define MAX_WHEEL_RAD_S      (MAX_RPM * RPM_TO_RAD_S)   // ~11.94 rad/s
#define DXL_UNIT_TO_RAD_S   (0.111f * RPM_TO_RAD_S)     // 1 unit AX = 0.111 RPM -> rad/s
#define DEG_TO_RAD           (3.14159265f / 180.0f)

static inline float clamp(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ---------------------------------------------------------------------------
// Statyczna alokacja wiadomosci JointState
// WAZNE: micro-ROS na embedded wymaga statycznej alokacji wszystkich buforow,
// bez tego rcl_publish/take moze hardfaultowac przy serializacji
// ---------------------------------------------------------------------------

/* --- SUBSCRIBER: /wheel_commands --- */
static sensor_msgs__msg__JointState      wheel_cmd_msg;
static rosidl_runtime_c__String          cmd_name_data[2];
static char                              cmd_name0_buf[32]      = "left_wheel_joint";
static char                              cmd_name1_buf[32]      = "right_wheel_joint";
static char                              cmd_frame_id_buf[32]   = {0};
static double                            cmd_position[2]   = {0.0, 0.0};
static double                            cmd_velocity[2]   = {0.0, 0.0};
static double                            cmd_effort[2]     = {0.0, 0.0};

/* --- PUBLISHER: /wheel_states --- */
static sensor_msgs__msg__JointState      wheel_state_msg;
static rosidl_runtime_c__String          state_name_data[2];
static char                              state_name0_buf[32]    = "left_wheel_joint";
static char                              state_name1_buf[32]    = "right_wheel_joint";
static char                              state_frame_id_buf[32] = "base_link";
static double                            state_position[2]   = {0.0, 0.0};
static double                            state_velocity[2]   = {0.0, 0.0};
static double                            state_effort[2]     = {0.0, 0.0};


// ---------------------------------------------------------------------------
// Hooki wymagane przez configSUPPORT_STATIC_ALLOCATION = 1
// ---------------------------------------------------------------------------
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t  uxIdleTaskStack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t  uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}

// ---------------------------------------------------------------------------
// Pomocnicza inicjalizacja JointState
// Ustawia wszystkie pola sekwencji ORAZ header (frame_id i stamp).
// Bez inicjalizacji header.frame_id micro-ROS przy serializacji moze siegnac
// po niezainicjalizowany wskaznik => HardFault.
// ---------------------------------------------------------------------------
static void joint_state_msg_init(
    sensor_msgs__msg__JointState *msg,
    rosidl_runtime_c__String     *names,
    char *name0, char *name1,
    char *frame_id_buf, size_t frame_id_cap,
    double *pos, double *vel, double *eff)
{
    /* header */
    msg->header.frame_id.data     = frame_id_buf;
    msg->header.frame_id.size     = (frame_id_buf && frame_id_buf[0]) ? strlen(frame_id_buf) : 0;
    msg->header.frame_id.capacity = frame_id_cap;
    msg->header.stamp.sec     = 0;
    msg->header.stamp.nanosec = 0;

    /* names */
    names[0].data     = name0;
    names[0].size     = strlen(name0);
    names[0].capacity = 32;
    names[1].data     = name1;
    names[1].size     = strlen(name1);
    names[1].capacity = 32;
    msg->name.data     = names;
    msg->name.size     = 2;
    msg->name.capacity = 2;

    /* position / velocity / effort */
    msg->position.data     = pos;
    msg->position.size     = 2;
    msg->position.capacity = 2;

    msg->velocity.data     = vel;
    msg->velocity.size     = 2;
    msg->velocity.capacity = 2;

    msg->effort.data     = eff;
    msg->effort.size     = 2;
    msg->effort.capacity = 2;
}

// ---------------------------------------------------------------------------
// Callback /wheel_commands
// topic_based_ros2_control wysyla velocity w rad/s
// Kolejnosc: [0]=left_wheel_joint, [1]=right_wheel_joint
// ---------------------------------------------------------------------------
static void wheel_cmd_callback(const void *msgin)
{
    const sensor_msgs__msg__JointState *msg =
        (const sensor_msgs__msg__JointState *)msgin;

    if (msg->velocity.size < 2) return;

    float v_left_rad_s  = (float)msg->velocity.data[0];
    float v_right_rad_s = (float)msg->velocity.data[1];

    /* rad/s -> speed_pct [-100, 100] */
    float pct_left  = clamp(v_left_rad_s  / MAX_WHEEL_RAD_S * 100.0f, -100.0f, 100.0f);
    float pct_right = clamp(v_right_rad_s / MAX_WHEEL_RAD_S * 100.0f, -100.0f, 100.0f);

    DXL_RosCommand_t cmd;

    cmd.id = ID_LEFT_WHEEL;
    cmd.speed_pct = pct_left;
    osMessageQueuePut(dxl_cmd_queue, &cmd, 0, 0);

    cmd.id = ID_RIGHT_WHEEL;
    cmd.speed_pct = pct_right;
    osMessageQueuePut(dxl_cmd_queue, &cmd, 0, 0);
}

// ---------------------------------------------------------------------------
// Makra obslugi bledow
// RCCHECK     - blad krytyczny (init), zapala czerwona LED, wiesza task
// RCSOFTCHECK - blad miekki (np. publish), tylko inkrementuje licznik bledow
// ---------------------------------------------------------------------------
static volatile uint32_t soft_error_count = 0;

#define RCCHECK(fn) {                                           \
    rcl_ret_t _rc = (fn);                                       \
    if (_rc != RCL_RET_OK) {                                    \
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);     \
        while(1) { osDelay(1000); }                             \
    }                                                           \
}

#define RCSOFTCHECK(fn) {                                       \
    rcl_ret_t _rc = (fn);                                       \
    if (_rc != RCL_RET_OK) { soft_error_count++; }              \
}

// ---------------------------------------------------------------------------
// Watek micro-ROS
// Stack 4096 slow (16 kB) - z naddatkiem dla JointState i sesji XRCE-DDS.
// Wczesniej 8000 slow bylo przewymiarowane; 4096 zostawia zapas heap dla micro-ROS.
//
// UWAGA: uzywamy StaticTask_t (typ FreeRTOS), a nie osStaticThreadDef_t.
// `osStaticThreadDef_t` to alias dodany dopiero w nowszych wersjach
// cmsis_os2.h (typedef StaticTask_t osStaticThreadDef_t). W starszych
// build'ach CMSIS-RTOS V2 ten alias nie istnieje, ale StaticTask_t jest
// zawsze gdy configSUPPORT_STATIC_ALLOCATION == 1.
// cmsis_os2.c i tak rzutuje cb_mem na (StaticTask_t *) wewnatrz osThreadNew.
// ---------------------------------------------------------------------------
uint32_t      defaultTaskBuffer[3000];
StaticTask_t  defaultTaskControlBlock;

const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_mem  = &defaultTaskBuffer[0],
    .stack_size = sizeof(defaultTaskBuffer),
    .cb_mem     = &defaultTaskControlBlock,
    .cb_size    = sizeof(defaultTaskControlBlock),
    .priority   = (osPriority_t)osPriorityNormal,
};
osThreadId_t defaultTaskHandle;

/* Prototypy transportu UART - implementacja w osobnym pliku CubeMX */
bool   cubemx_transport_open (struct uxrCustomTransport *t);
bool   cubemx_transport_close(struct uxrCustomTransport *t);
size_t cubemx_transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read (struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err);
void  *microros_allocate     (size_t size, void *state);
void   microros_deallocate   (void *pointer, void *state);
void  *microros_reallocate   (void *pointer, size_t size, void *state);
void  *microros_zero_allocate(size_t n, size_t size, void *state);

void StartDefaultTask(void *argument)
{
    (void)argument;

    /* 1. Transport UART1 */
    rmw_uros_set_custom_transport(
        true, (void *)&huart1,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    /* 2. Alokator FreeRTOS */
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        while(1) { osDelay(1000); }
    }

    /* 3. Oczekiwanie na agenta - miga dioda GPIO_PIN_3 */
    while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        osDelay(500);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);  /* Agent polaczony */

    /* 4. Inicjalizacja ROS 2 z ustawieniem ROS_DOMAIN_ID
     *
     * Zamiast rclc_support_init uzywamy wariantu *_with_options, zeby
     * mozna bylo ustawic domain_id (do laczenia zdalnego z PC).
     */
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, (size_t)ROS_DOMAIN_ID));

    rclc_support_t support;
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    /* Synchronizacja czasu z agentem - potrzebne do prawidlowego header.stamp.
     * Jesli sie nie uda, dalej dzialamy ze stamp=0 (ros2_control to akceptuje). */
    (void)rmw_uros_sync_session(1000);

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "stm32_robot_node", "", &support));

    /* 5. Inicjalizacja buforow wiadomosci (header + nazwy + sekwencje) */
    joint_state_msg_init(&wheel_cmd_msg,
        cmd_name_data, cmd_name0_buf, cmd_name1_buf,
        cmd_frame_id_buf, sizeof(cmd_frame_id_buf),
        cmd_position, cmd_velocity, cmd_effort);

    joint_state_msg_init(&wheel_state_msg,
        state_name_data, state_name0_buf, state_name1_buf,
        state_frame_id_buf, sizeof(state_frame_id_buf),
        state_position, state_velocity, state_effort);

    /* 6. Publisher /wheel_states */
    rcl_publisher_t wheel_pub;
    RCCHECK(rclc_publisher_init_default(
        &wheel_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "wheel_states"));

    /* 7. Subscriber /wheel_commands */
    rcl_subscription_t wheel_cmd_sub;
    RCCHECK(rclc_subscription_init_default(
        &wheel_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "wheel_commands"));

    /* 8. Executor - 1 subskrypcja */
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &wheel_cmd_sub, &wheel_cmd_msg,
        wheel_cmd_callback, ON_NEW_DATA));

    /* 9. Petla glowna */
    DXL_RosFeedback_t feedback;
    uint32_t last_pub_tick = 0;
    uint32_t last_ping_tick = 0;

    for (;;)
    {
        /* Obsluga subskrypcji - timeout 0 = nieblokujace */
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

        uint32_t now = osKernelGetTickCount();

        /* Publikacja /wheel_states co 100 ms */
        if ((now - last_pub_tick) >= 100)
        {
            last_pub_tick = now;

            if (osMessageQueueGet(dxl_feedback_queue, &feedback, NULL, 0) == osOK)
            {
                /*
                 * Dane z dxl_control.c (DXL_RosFeedback_t.data[]):
                 * [0] = RIGHT position (stopnie)
                 * [1] = RIGHT velocity (DXL raw, signed: + = CCW, - = CW)
                 * [2] = RIGHT load (%)
                 * [3] = LEFT  position (stopnie)
                 * [4] = LEFT  velocity (DXL raw, signed)
                 * [5] = LEFT  load (%)
                 *
                 * JointState: indeks [0] = left, [1] = right
                 *
                 * UWAGA: kola sa zamontowane lustrzanie (xacro: rpy +/-pi/2,
                 * axis z = +/-1). Jesli okaze sie ze odom liczy zly kierunek,
                 * dodaj minus przy state_velocity[1] i state_position[1] (lub [0]).
                 */

                /* Predkosc: DXL units -> rad/s */
                state_velocity[0] = (double)(feedback.data[4] * DXL_UNIT_TO_RAD_S); /* left  */
                state_velocity[1] = (double)(feedback.data[1] * DXL_UNIT_TO_RAD_S); /* right */

                /* Pozycja: stopnie -> rad */
                state_position[0] = (double)(feedback.data[3] * DEG_TO_RAD); /* left  */
                state_position[1] = (double)(feedback.data[0] * DEG_TO_RAD); /* right */

                /* Effort: load % */
                state_effort[0] = (double)feedback.data[5]; /* left  */
                state_effort[1] = (double)feedback.data[2]; /* right */

                /* Timestamp - jesli sync sie udal, bedzie sensowny;
                 * jesli nie, bedzie 0 (i to tez jest OK dla ros2_control). */
                int64_t time_ns = rmw_uros_epoch_nanos();
                wheel_state_msg.header.stamp.sec     = (int32_t)(time_ns / 1000000000LL);
                wheel_state_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

                RCSOFTCHECK(rcl_publish(&wheel_pub, &wheel_state_msg, NULL));
            }
        }

        /* Ping agenta co 2 s - wykrywa zerwane polaczenie. Przy braku agenta
         * mozna by zrobic reinit, ale na razie tylko sygnalizujemy LED'em. */
        if ((now - last_ping_tick) >= 2000) {
            last_ping_tick = now;
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
            }
        }

        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
        osDelay(10);
    }
}

// ---------------------------------------------------------------------------
// MX_FREERTOS_Init - wywolywane z main() przed osKernelStart()
// ---------------------------------------------------------------------------
void MX_FREERTOS_Init(void)
{
    /* Kolejki MUSZA byc utworzone przed startem taskow ktore ich uzywaja.
     * Powiekszylem dxl_cmd_queue z 10 do 32 - chroni przed gubieniem komend
     * przy szybkich strumieniach z ros2_control (np. 100 Hz). */
    dxl_cmd_queue      = osMessageQueueNew(32, sizeof(DXL_RosCommand_t),  NULL);
    dxl_feedback_queue = osMessageQueueNew(5,  sizeof(DXL_RosFeedback_t), NULL);

    /* Inicjalizacja i start tasku Dynamixel */
    DXL_Manager_Init();
    osThreadNew(DXL_Manager_Task, NULL, &dxl_task_attr);

    /* Start tasku micro-ROS */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}
