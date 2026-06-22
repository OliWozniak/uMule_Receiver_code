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
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "usart.h"
#include "dxl_control.h"
#include "imu_task.h"
#include "ina219_task.h"
#include "sonar_task.h"

// ---------------------------------------------------------------------------
// KONFIGURACJA SIECI
// ROS_DOMAIN_ID - musi byc identyczne jak na komputerze hosta (export ROS_DOMAIN_ID=...)
// ---------------------------------------------------------------------------
#ifndef ROS_DOMAIN_ID
#define ROS_DOMAIN_ID   0       // <-- zmien na wartosc z `echo $ROS_DOMAIN_ID` na PC
#endif

// ---------------------------------------------------------------------------
// Parametry kinematyki robota
// Musza byc IDENTYCZNE z my_controllers.yaml i ros2_control.xacro
// ---------------------------------------------------------------------------
#define WHEEL_SEPARATION_M   0.30f
#define WHEEL_RADIUS_M       0.08f
#define MAX_RPM              97.0f
#define RPM_TO_RAD_S         (2.0f * 3.14159265f / 60.0f)
#define MAX_WHEEL_RAD_S      (MAX_RPM * RPM_TO_RAD_S)   // ~11.94 rad/s
#define DXL_UNIT_TO_RAD_S   (0.111f * RPM_TO_RAD_S)     // 1 unit RX-64 = 0.111 RPM -> rad/s
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

/* --- PUBLISHER: /imu (sensor_msgs/Imu — accel + gyro z LSM6DS3TR-C) --- */
static sensor_msgs__msg__Imu             imu_msg;
static char                              imu_frame_id_buf[16] = "imu_link";

/* --- PUBLISHER: /mag (sensor_msgs/MagneticField — magnetometr LIS3MDL) ---
 * sensor_msgs/Imu nie zawiera pola magnetometru, dlatego mag = osobny topic. */
static sensor_msgs__msg__MagneticField   mag_msg;
static char                              mag_frame_id_buf[16] = "imu_link";

/* --- PUBLISHER: /power (std_msgs/Float32MultiArray — INA219) ---
 * data[0]=voltage_V, data[1]=current_A, data[2]=power_W */
static std_msgs__msg__Float32MultiArray  power_msg;
static float                             power_data_buf[3]   = {0.0f, 0.0f, 0.0f};

/* --- PUBLISHER: /STM_sonar_N (sensor_msgs/Range — HC-SR04 × 4) ---
 * Jeden publisher na sensor; NaN w range.range oznacza brak echa.
 * min_range=0.02 m, max_range=4.0 m, field_of_view~15° (spec HC-SR04) */
static sensor_msgs__msg__Range           sonar_msg[SONAR_COUNT];
static char sonar_frame_id_buf[SONAR_COUNT][16] = {
    "sonar_0", "sonar_1", "sonar_2", "sonar_3"
};


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
// RCCHECK     - blad krytyczny podczas (re)inicjalizacji: ustawia flage init_ok
// RCSOFTCHECK - blad miekki (np. publish), tylko inkrementuje licznik bledow
// ---------------------------------------------------------------------------
static volatile uint32_t soft_error_count = 0;

#define RCCHECK(fn) {                                           \
    rcl_ret_t _rc = (fn);                                       \
    if (_rc != RCL_RET_OK) { init_ok = false; }                 \
}

#define RCSOFTCHECK(fn) {                                       \
    rcl_ret_t _rc = (fn);                                       \
    if (_rc != RCL_RET_OK) { soft_error_count++; }              \
}

// ---------------------------------------------------------------------------
// Uchwyty encji micro-ROS — zbiorcza struktura dla create/destroy
// ---------------------------------------------------------------------------
typedef struct {
    rclc_support_t      support;
    rcl_node_t          node;
    rcl_publisher_t     wheel_pub;
    rcl_publisher_t     imu_pub;
    rcl_publisher_t     mag_pub;
    rcl_publisher_t     power_pub;
    rcl_publisher_t     sonar_pub[SONAR_COUNT];
    rcl_subscription_t  wheel_cmd_sub;
    rclc_executor_t     executor;
} RosEntities_t;

static RosEntities_t ros_ent;

// ---------------------------------------------------------------------------
// Pomocnik: wyslij zerowa predkosc do obu kol (bezpieczenstwo przy utracie polaczenia)
// ---------------------------------------------------------------------------
static void dxl_stop_all(void)
{
    DXL_RosCommand_t stop = {0};
    stop.id = ID_LEFT_WHEEL;  stop.speed_pct = 0.0f;
    osMessageQueuePut(dxl_cmd_queue, &stop, 0, 0);
    stop.id = ID_RIGHT_WHEEL;
    osMessageQueuePut(dxl_cmd_queue, &stop, 0, 0);
}

// ---------------------------------------------------------------------------
// Tworzenie wszystkich encji micro-ROS (wywolywane po kazdym (re)polaczeniu)
// Zwraca true jesli wszystko OK.
// ---------------------------------------------------------------------------
static bool ros_create_entities(rcl_allocator_t *allocator)
{
    bool init_ok = true;

    /* Support — kazdý krok inicjalizowany oddzielnie, by nie wywolywac fini na
     * niezainicjalizowanej strukturze i nie isc dalej po bledzie. */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, *allocator) != RCL_RET_OK) return false;
    if (rcl_init_options_set_domain_id(&init_options, (size_t)ROS_DOMAIN_ID) != RCL_RET_OK) {
        rcl_init_options_fini(&init_options);
        return false;
    }
    bool support_ok = (rclc_support_init_with_options(
        &ros_ent.support, 0, NULL, &init_options, allocator) == RCL_RET_OK);
    rcl_init_options_fini(&init_options);
    if (!support_ok) return false;

    (void)rmw_uros_sync_session(1000);

    RCCHECK(rclc_node_init_default(&ros_ent.node, "stm32_robot_node", "", &ros_ent.support));
    /* Wczesny powrot: publishery nie moga byc inicjalizowane na nieprawidlowym nodzie */
    if (!init_ok) { rclc_support_fini(&ros_ent.support); return false; }

    /* Inicjalizacja buforow wiadomosci (wskazniki na statyczne bufory) */
    joint_state_msg_init(&wheel_cmd_msg,
        cmd_name_data, cmd_name0_buf, cmd_name1_buf,
        cmd_frame_id_buf, sizeof(cmd_frame_id_buf),
        cmd_position, cmd_velocity, cmd_effort);

    joint_state_msg_init(&wheel_state_msg,
        state_name_data, state_name0_buf, state_name1_buf,
        state_frame_id_buf, sizeof(state_frame_id_buf),
        state_position, state_velocity, state_effort);

    memset(&imu_msg, 0, sizeof(imu_msg));
    imu_msg.header.frame_id.data     = imu_frame_id_buf;
    imu_msg.header.frame_id.size     = strlen(imu_frame_id_buf);
    imu_msg.header.frame_id.capacity = sizeof(imu_frame_id_buf);
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = 5.1e-7f;
    imu_msg.angular_velocity_covariance[4] = 5.1e-7f;
    imu_msg.angular_velocity_covariance[8] = 5.1e-7f;
    imu_msg.linear_acceleration_covariance[0] = 8.1e-5f;
    imu_msg.linear_acceleration_covariance[4] = 8.1e-5f;
    imu_msg.linear_acceleration_covariance[8] = 8.1e-5f;

    memset(&mag_msg, 0, sizeof(mag_msg));
    mag_msg.header.frame_id.data     = mag_frame_id_buf;
    mag_msg.header.frame_id.size     = strlen(mag_frame_id_buf);
    mag_msg.header.frame_id.capacity = sizeof(mag_frame_id_buf);

    memset(&power_msg, 0, sizeof(power_msg));
    power_msg.data.data     = power_data_buf;
    power_msg.data.size     = 3;
    power_msg.data.capacity = 3;

    for (uint8_t i = 0; i < SONAR_COUNT; i++) {
        memset(&sonar_msg[i], 0, sizeof(sonar_msg[i]));
        sonar_msg[i].header.frame_id.data     = sonar_frame_id_buf[i];
        sonar_msg[i].header.frame_id.size     = strlen(sonar_frame_id_buf[i]);
        sonar_msg[i].header.frame_id.capacity = sizeof(sonar_frame_id_buf[i]);
        sonar_msg[i].radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        sonar_msg[i].field_of_view  = 0.2618f;
        sonar_msg[i].min_range      = 0.02f;
        sonar_msg[i].max_range      = 4.00f;
        sonar_msg[i].range          = 0.0f;
    }

    /* Publishery */
    RCCHECK(rclc_publisher_init_default(
        &ros_ent.wheel_pub, &ros_ent.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "wheel_states"));
    RCCHECK(rclc_publisher_init_best_effort(
        &ros_ent.imu_pub, &ros_ent.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "STM_imu"));
    RCCHECK(rclc_publisher_init_best_effort(
        &ros_ent.mag_pub, &ros_ent.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), "STM_mag"));
    RCCHECK(rclc_publisher_init_best_effort(
        &ros_ent.power_pub, &ros_ent.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "STM_power"));

    {
        static const char *sonar_topic_names[SONAR_COUNT] = {
            "STM_sonar_0", "STM_sonar_1", "STM_sonar_2", "STM_sonar_3"
        };
        for (uint8_t i = 0; i < SONAR_COUNT; i++) {
            RCCHECK(rclc_publisher_init_best_effort(
                &ros_ent.sonar_pub[i], &ros_ent.node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
                sonar_topic_names[i]));
        }
    }

    /* Subscriber + executor */
    RCCHECK(rclc_subscription_init_default(
        &ros_ent.wheel_cmd_sub, &ros_ent.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "wheel_commands"));
    RCCHECK(rclc_executor_init(&ros_ent.executor, &ros_ent.support.context, 1, allocator));
    RCCHECK(rclc_executor_add_subscription(
        &ros_ent.executor, &ros_ent.wheel_cmd_sub, &wheel_cmd_msg,
        wheel_cmd_callback, ON_NEW_DATA));

    return init_ok;
}

// ---------------------------------------------------------------------------
// Niszczenie wszystkich encji micro-ROS (przed reinicjalizacja)
// ---------------------------------------------------------------------------
static void ros_destroy_entities(rcl_allocator_t *allocator)
{
    rclc_executor_fini(&ros_ent.executor);
    rcl_subscription_fini(&ros_ent.wheel_cmd_sub, &ros_ent.node);
    rcl_publisher_fini(&ros_ent.wheel_pub,   &ros_ent.node);
    rcl_publisher_fini(&ros_ent.imu_pub,     &ros_ent.node);
    rcl_publisher_fini(&ros_ent.mag_pub,     &ros_ent.node);
    rcl_publisher_fini(&ros_ent.power_pub,   &ros_ent.node);
    for (uint8_t i = 0; i < SONAR_COUNT; i++)
        rcl_publisher_fini(&ros_ent.sonar_pub[i], &ros_ent.node);
    rcl_node_fini(&ros_ent.node);
    rclc_support_fini(&ros_ent.support);
    (void)allocator;
}

// ---------------------------------------------------------------------------
// Watek micro-ROS — statyczna alokacja stosu i TCB.
// Stack 3000 slow = 12000 bajtow: wystarczajacy dla JointState i sesji XRCE-DDS.
// ---------------------------------------------------------------------------
static uint32_t      defaultTaskBuffer[3000];
static StaticTask_t  defaultTaskControlBlock;

const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .cb_mem     = &defaultTaskControlBlock,
    .cb_size    = sizeof(defaultTaskControlBlock),
    .stack_mem  = defaultTaskBuffer,
    .stack_size = sizeof(defaultTaskBuffer),  /* 12000 B = 3000 slow */
    .priority   = (osPriority_t) osPriorityNormal,
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

    /* 1. Transport UART1 — ustawiany raz, nie zmienia sie przy reconnect */
    rmw_uros_set_custom_transport(
        true, (void *)&huart1,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

    /* 2. Alokator FreeRTOS — rowniez raz */
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        while(1) { osDelay(1000); }
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();

    /* 3. Zewnetrzna petla reconnect — nigdy nie wychodzi */
    for (;;)
    {
        /* 3a. Czekaj na agenta — miga LED GPIO_PIN_3 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
            osDelay(500);
        }

        /* 3b. Tworzenie encji — jesli blad, czekaj i sprobuj ponownie */
        if (!ros_create_entities(&allocator)) {
            osDelay(1000);
            continue;
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); /* Agent polaczony */

        /* 3c. Petla glowna — dziala dopoki agent odpowiada */
        DXL_RosFeedback_t  feedback  = {0};
        IMU_QueueData_t    imu_q     = {0};
        INA219_QueueData_t ina219_q  = {0};
        Sonar_QueueData_t  sonar_q   = {0};
        uint32_t last_pub_tick   = 0;
        uint32_t last_imu_tick   = 0;
        uint32_t last_power_tick = 0;
        uint32_t last_sonar_tick = 0;
        uint32_t last_ping_tick  = 0;

        for (;;)
        {
            rclc_executor_spin_some(&ros_ent.executor, RCL_MS_TO_NS(0));

            osMessageQueueGet(dxl_feedback_queue, &feedback, NULL, 0);
            osMessageQueueGet(imu_data_queue,     &imu_q,    NULL, 0);
            osMessageQueueGet(ina219_data_queue,  &ina219_q, NULL, 0);
            osMessageQueueGet(sonar_data_queue,   &sonar_q,  NULL, 0);

            uint32_t now = osKernelGetTickCount();

            /* Publikacja /wheel_states co 20 ms (~50 Hz) */
            if ((now - last_pub_tick) >= 20)
            {
                last_pub_tick = now;

                state_velocity[0] = -(double)(feedback.data[4] * DXL_UNIT_TO_RAD_S);
                state_velocity[1] =  (double)(feedback.data[1] * DXL_UNIT_TO_RAD_S);
                state_position[0] = -(double)(feedback.data[3] * DEG_TO_RAD);
                state_position[1] =  (double)(feedback.data[0] * DEG_TO_RAD);
                state_effort[0]   = -(double)feedback.data[5];
                state_effort[1]   =  (double)feedback.data[2];

                int64_t time_ns = rmw_uros_epoch_nanos();
                wheel_state_msg.header.stamp.sec     = (int32_t)(time_ns / 1000000000LL);
                wheel_state_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

                RCSOFTCHECK(rcl_publish(&ros_ent.wheel_pub, &wheel_state_msg, NULL));
            }

            /* Publikacja /STM_imu + /STM_mag co 20 ms (~50 Hz) */
            if ((now - last_imu_tick) >= 20)
            {
                last_imu_tick = now;
                int64_t ts = rmw_uros_epoch_nanos();
                int32_t  ts_sec  = (int32_t)(ts / 1000000000LL);
                uint32_t ts_nsec = (uint32_t)(ts % 1000000000LL);

                imu_msg.header.stamp.sec      = ts_sec;
                imu_msg.header.stamp.nanosec  = ts_nsec;
                imu_msg.angular_velocity.x    = (double)imu_q.gyro_x;
                imu_msg.angular_velocity.y    = (double)imu_q.gyro_y;
                imu_msg.angular_velocity.z    = (double)imu_q.gyro_z;
                imu_msg.linear_acceleration.x = (double)imu_q.accel_x;
                imu_msg.linear_acceleration.y = (double)imu_q.accel_y;
                imu_msg.linear_acceleration.z = (double)imu_q.accel_z;

                if (imu_q.cov_valid) {
                    imu_msg.angular_velocity_covariance[0] = (double)imu_q.var_gx;
                    imu_msg.angular_velocity_covariance[4] = (double)imu_q.var_gy;
                    imu_msg.angular_velocity_covariance[8] = (double)imu_q.var_gz;
                    imu_msg.linear_acceleration_covariance[0] = (double)imu_q.var_ax;
                    imu_msg.linear_acceleration_covariance[4] = (double)imu_q.var_ay;
                    imu_msg.linear_acceleration_covariance[8] = (double)imu_q.var_az;
                }
                RCSOFTCHECK(rcl_publish(&ros_ent.imu_pub, &imu_msg, NULL));

                mag_msg.header.stamp.sec     = ts_sec;
                mag_msg.header.stamp.nanosec = ts_nsec;
                mag_msg.magnetic_field.x     = (double)imu_q.mag_x;
                mag_msg.magnetic_field.y     = (double)imu_q.mag_y;
                mag_msg.magnetic_field.z     = (double)imu_q.mag_z;
                if (imu_q.cov_valid) {
                    mag_msg.magnetic_field_covariance[0] = (double)imu_q.var_mx;
                    mag_msg.magnetic_field_covariance[4] = (double)imu_q.var_my;
                    mag_msg.magnetic_field_covariance[8] = (double)imu_q.var_mz;
                }
                RCSOFTCHECK(rcl_publish(&ros_ent.mag_pub, &mag_msg, NULL));
            }

            /* Publikacja /power co 500 ms */
            if ((now - last_power_tick) >= 500)
            {
                last_power_tick = now;
                power_data_buf[0] = ina219_q.voltage_V;
                power_data_buf[1] = ina219_q.current_A;
                power_data_buf[2] = ina219_q.power_W;
                RCSOFTCHECK(rcl_publish(&ros_ent.power_pub, &power_msg, NULL));
            }

            /* Publikacja /STM_sonar_N co 200 ms */
            if ((now - last_sonar_tick) >= 200)
            {
                last_sonar_tick = now;
                for (uint8_t i = 0; i < SONAR_COUNT; i++) {
                    int64_t ts_ns = (int64_t)sonar_q.timestamp_ms[i] * 1000000LL;
                    sonar_msg[i].header.stamp.sec     = (int32_t)(ts_ns / 1000000000LL);
                    sonar_msg[i].header.stamp.nanosec = (uint32_t)(ts_ns % 1000000000LL);
                    sonar_msg[i].range = sonar_q.distance_m[i];
                    RCSOFTCHECK(rcl_publish(&ros_ent.sonar_pub[i], &sonar_msg[i], NULL));
                }
            }

            /* Ping agenta co 2 s — przy braku odpowiedzi: zatrzymaj silniki i reinit */
            if ((now - last_ping_tick) >= 2000)
            {
                last_ping_tick = now;
                if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
                    /* Utrata polaczenia: zatrzymaj silniki dla bezpieczenstwa */
                    dxl_stop_all();
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
                    break; /* wyjdz z petli wewnetrznej → destroy → reconnect */
                }
            }

            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
            osDelay(1);
        }

        /* 3d. Niszczenie encji przed ponowna inicjalizacja */
        ros_destroy_entities(&allocator);
        osDelay(500); /* krotka przerwa zanim sprobujemy ping ponownie */
    }
}

// ---------------------------------------------------------------------------
// MX_FREERTOS_Init - wywolywane z main() przed osKernelStart()
// ---------------------------------------------------------------------------
void MX_FREERTOS_Init(void) {
    /* DXL: inicjalizacja sprzetu, kolejki, task */
    DXL_Manager_Init();
    dxl_cmd_queue      = osMessageQueueNew(32, sizeof(DXL_RosCommand_t),  NULL);
    dxl_feedback_queue = osMessageQueueNew(5,  sizeof(DXL_RosFeedback_t), NULL);
    osThreadNew(DXL_Manager_Task, NULL, &dxl_task_attr);

    /* IMU (I2C4): LSM6DS3TR-C + LIS3MDL */
    IMU_Manager_Init();
    imu_data_queue = osMessageQueueNew(2, sizeof(IMU_QueueData_t), NULL);
    osThreadNew(IMU_Manager_Task, NULL, &imu_task_attr);

    /* INA219 (I2C3): czujnik pradu */
    INA219_Manager_Init();
    ina219_data_queue = osMessageQueueNew(2, sizeof(INA219_QueueData_t), NULL);
    osThreadNew(INA219_Manager_Task, NULL, &ina219_task_attr);

    /* HC-SR04 (TIM2 IC + TIM6 seq): czujniki odleglosci */
    Sonar_Manager_Init();
    sonar_data_queue = osMessageQueueNew(2, sizeof(Sonar_QueueData_t), NULL);
    osThreadNew(Sonar_Manager_Task, NULL, &sonar_task_attr);

    /* micro-ROS */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
}
