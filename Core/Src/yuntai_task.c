#include "yuntai_task.h"
#include "can_bsp.h"
#include "jc4310.h"
#include "fdcan.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "user_lib.h"
#include "pid.h"
#include "debug_uart.h"
#include <math.h>
#include <stdio.h>

static float gyro[3] = {0.0f};
static float acc[3] = {0.0f};
static float temp = 0.0f;
static float imuQuat[4] = {0.0f};
static float imuAngle[3] = {0.0f};

typedef enum {
    YUNTAI_STATE_INIT = 0,
    YUNTAI_STATE_BMI088_INIT,
    YUNTAI_STATE_MOTOR_CALIBRATE,
    YUNTAI_STATE_MOTOR_CLOSED_LOOP,
    YUNTAI_STATE_MOTOR_SET_MODE,
    YUNTAI_STATE_LOCK_ATTITUDE,
    YUNTAI_STATE_RUNNING,
} YuntaiState;

static YuntaiState g_state = YUNTAI_STATE_INIT;
static uint32_t g_state_timestamp = 0;
static uint32_t g_last_imu_timestamp = 0;
static uint32_t g_last_control_timestamp = 0;
static uint32_t g_last_debug_timestamp = 0;
static uint8_t  g_state_cmd_sent = 0;

static float yaw_target = 0.0f;
static float pitch_target = 0.0f;

static PID_Controller yaw_pid;
static PID_Controller pitch_pid;
static uint8_t  g_imu_ok = 0;
static uint8_t  g_bmi088_retry = 0;

#define BMI088_TIMEOUT_MS      5000
#define CALIBRATE_DELAY        2000
#define CLOSED_LOOP_DELAY      500
#define SET_MODE_DELAY         500
#define LOCK_ATTITUDE_DELAY    500
#define IMU_PERIOD             2
#define CONTROL_PERIOD         5
#define DEBUG_PERIOD           500

#define INS_YAW_ADDRESS_OFFSET   0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET  2

static float angle_diff_rad(float target, float current)
{
    return normalize_angle_rad(target - current);
}

static void AHRS_init(float quat[4])
{
    quat[0] = 1.0f; quat[1] = 0.0f; quat[2] = 0.0f; quat[3] = 0.0f;
}

static void AHRS_update(float quat[4], float g[3], float a[3])
{
    MahonyAHRSupdateIMU(quat, g[0], g[1], g[2], a[0], a[1], a[2]);
}

static void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw   = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll  = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]), 2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

void yuntai_init(void)
{
    can_bsp_init();
    debug_println("=== YunTai Start ===");
    g_state = YUNTAI_STATE_BMI088_INIT;
    g_state_timestamp = HAL_GetTick();
    g_state_cmd_sent = 0;
}

void yuntai_control_loop(void)
{
    uint32_t now = HAL_GetTick();

    switch (g_state)
    {
    case YUNTAI_STATE_INIT:
        break;

    // ---- 1. BMI088初始化 ----
    case YUNTAI_STATE_BMI088_INIT:
        if ((now - g_state_timestamp) < 500) break;  // 每500ms重试一次

        g_bmi088_retry++;
        {
            char buf[40];
            int len = snprintf(buf, sizeof(buf), "BMI088: Try #%d...\r\n", (int)g_bmi088_retry);
            if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
        }
        {
            uint8_t result = BMI088_init();
            char buf[40];
            int len = snprintf(buf, sizeof(buf), "BMI088: result=%d\r\n", (int)result);
            if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
            if (result == BMI088_NO_ERROR)
            {
                g_imu_ok = 1;
                AHRS_init(imuQuat);
                debug_println("BMI088: OK -> Motor Calibrate");
                g_state = YUNTAI_STATE_MOTOR_CALIBRATE;
                g_state_timestamp = now;
                g_state_cmd_sent = 0;
            }
            else
            {
                g_state_timestamp = now;
            }
        }
        break;

    // ---- 2. 上电稳定延时（不校准，电机已出厂校准） ----
    case YUNTAI_STATE_MOTOR_CALIBRATE:
        if ((now - g_state_timestamp) > CALIBRATE_DELAY)
        {
            g_state = YUNTAI_STATE_MOTOR_CLOSED_LOOP;
            g_state_timestamp = now;
            g_state_cmd_sent = 0;
        }
        break;

    // ---- 3. 进入闭环 ----
    case YUNTAI_STATE_MOTOR_CLOSED_LOOP:
        if (!g_state_cmd_sent)
        {
            g_state_cmd_sent = 1;
            debug_println("Motor: Closed Loop...");
            jc_enter_closed_loop(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
            jc_enter_closed_loop(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
        }
        if ((now - g_state_timestamp) > CLOSED_LOOP_DELAY)
        {
            g_state = YUNTAI_STATE_MOTOR_SET_MODE;
            g_state_timestamp = now;
            g_state_cmd_sent = 0;
        }
        break;

    // ---- 4. 速度模式 ----
    case YUNTAI_STATE_MOTOR_SET_MODE:
        if (!g_state_cmd_sent)
        {
            g_state_cmd_sent = 1;
            debug_println("Motor: Speed Mode...");
            jc_set_control_mode(&hfdcan1, YUNTAI_MOTOR_YAW_ID, JC_MODE_SPEED);
            jc_set_control_mode(&hfdcan2, YUNTAI_MOTOR_PITCH_ID, JC_MODE_SPEED);
        }
        if ((now - g_state_timestamp) > SET_MODE_DELAY)
        {
            g_state = YUNTAI_STATE_LOCK_ATTITUDE;
            g_state_timestamp = now;
            g_state_cmd_sent = 0;
            g_last_imu_timestamp = 0;
            debug_println("IMU: Locking attitude...");
        }
        break;

    // ---- 5. 锁定初始姿态 ----
    case YUNTAI_STATE_LOCK_ATTITUDE:
        if ((now - g_last_imu_timestamp) >= IMU_PERIOD)
        {
            g_last_imu_timestamp = now;
            BMI088_read(gyro, acc, &temp);
            AHRS_update(imuQuat, gyro, acc);
            GetAngle(imuQuat,
                imuAngle + INS_YAW_ADDRESS_OFFSET,
                imuAngle + INS_PITCH_ADDRESS_OFFSET,
                imuAngle + INS_ROLL_ADDRESS_OFFSET);
        }
        if ((now - g_state_timestamp) > LOCK_ATTITUDE_DELAY)
        {
            yaw_target   = imuAngle[INS_YAW_ADDRESS_OFFSET];
            pitch_target = imuAngle[INS_PITCH_ADDRESS_OFFSET];
            // PID: kp小、输出限制30rpm
            PID_Init(&yaw_pid, 200.0f, 30.0f, 10.0f, 50.0f);
            yaw_pid.integral_limit = 10.0f;
            PID_Init(&pitch_pid, 80.0f, 10.0f, 5.0f, 30.0f);
            pitch_pid.integral_limit = 5.0f;
            {
                char buf[80];
                int yt = (int)(yaw_target * 5730.0f);
                int pt = (int)(pitch_target * 5730.0f);
                int len = snprintf(buf, sizeof(buf), "TARGET: Y=%d P=%d (deg*100)\r\n", yt, pt);
                if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
            }
            g_state = YUNTAI_STATE_RUNNING;
            g_state_timestamp = now;
            g_last_control_timestamp = now;
            g_last_debug_timestamp = now;
            debug_println("=== STABILIZATION ACTIVE ===");
        }
        break;

    // ---- 6. 双轴稳定闭环 ----
    case YUNTAI_STATE_RUNNING:
        if ((now - g_last_imu_timestamp) >= IMU_PERIOD)
        {
            g_last_imu_timestamp = now;
            BMI088_read(gyro, acc, &temp);
            AHRS_update(imuQuat, gyro, acc);
            GetAngle(imuQuat,
                imuAngle + INS_YAW_ADDRESS_OFFSET,
                imuAngle + INS_PITCH_ADDRESS_OFFSET,
                imuAngle + INS_ROLL_ADDRESS_OFFSET);
        }
        if ((now - g_last_control_timestamp) >= CONTROL_PERIOD)
        {
            g_last_control_timestamp = now;
            float dt = (float)CONTROL_PERIOD / 1000.0f;
            // Yaw: PID闭环（水平电机旋转会反馈到IMU）
            float yaw_err = angle_diff_rad(yaw_target, imuAngle[INS_YAW_ADDRESS_OFFSET]);
            PID_Update(&yaw_pid, yaw_err, 0.0f, dt);
            jc_set_speed_rpm_x100(&hfdcan1, YUNTAI_MOTOR_YAW_ID, (int32_t)(yaw_pid.out * 100.0f));

            // Pitch: 直接用陀螺仪角速度驱动（俯仰电机不影响底板IMU，不能用角度PID）
            // gyro单位为rad/s，转为rpm*100后取反（补偿方向）
            int32_t pitch_speed = -(int32_t)(gyro[1] * 1500.0f);  // rad/s → rpm*100
            jc_set_speed_rpm_x100(&hfdcan2, YUNTAI_MOTOR_PITCH_ID, pitch_speed);
        }
        // 每200ms输出一次IMU数据
        if ((now - g_last_debug_timestamp) >= DEBUG_PERIOD)
        {
            g_last_debug_timestamp = now;
            float yaw_err = angle_diff_rad(yaw_target, imuAngle[INS_YAW_ADDRESS_OFFSET]);
            int ye = (int)(yaw_err * 5730.0f);
            int yo = (int)(yaw_pid.out * 100.0f);
            int pg = (int)(gyro[1] * 5730.0f);  // pitch陀螺角速度 deg/s*100
            int ps = -(int)(gyro[1] * 600.0f);  // pitch电机指令 rpm*100
            char buf[80];
            int len = snprintf(buf, sizeof(buf), "PID: Ye=%d Yo=%d | Gy=%d Ps=%d\r\n", ye, yo, pg, ps);
            if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100);
            debug_print_imu(imuAngle[INS_YAW_ADDRESS_OFFSET],
                           imuAngle[INS_PITCH_ADDRESS_OFFSET],
                           imuAngle[INS_ROLL_ADDRESS_OFFSET],
                           gyro[0], gyro[1], gyro[2],
                           acc[0], acc[1], acc[2]);
        }
        break;
    }
}
