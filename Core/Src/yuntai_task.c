/**
  ******************************************************************************
  * @file    yuntai_task.c
  * @brief   双轴云台生产控制链路
  *
  * yaw:
  *   IMU锁定角 + 视觉偏置 -> 角度PID -> 力矩电流
  *
  * pitch:
  *   编码器锁定角 + 平台俯仰补偿 + 视觉偏置
  *   -> 编码器位置PID -> 速度PI -> 力矩电流
  ******************************************************************************
  */

#include "yuntai_task.h"
#include "can_bsp.h"
#include "jc4310.h"
#include "fdcan.h"
#include "gimbal_link.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "user_lib.h"
#include "pid.h"
#include "debug_uart.h"
#include <math.h>
#include <stdio.h>

/* 启动时序。启动路径只使用延时，不等待非必要条件。 */
#define BMI_RETRY_MS             500u
#define MOTOR_BOOT_DELAY_MS     1500u
#define CLOSED_LOOP_DELAY_MS     300u
#define MODE_DELAY_MS            300u
#define LOCK_DELAY_MS           1200u
#define LOCK_BIAS_CAL_MS         800u
#define GYRO_BIAS_MAX            0.10f
#define GYRO_BIAS_MIN_SAMPLES    100u

/* 1 kHz 主控制与 IMU。 */
#define IMU_PERIOD_MS              1u
#define CONTROL_PERIOD_MS          1u
#define UART10_LOG_PERIOD_MS      50u

/* yaw：角度 PID 直接输出电流。 */
#define YAW_KP_MA_PER_RAD       6000.0f
#define YAW_KD_MA_PER_RAD_S     1500.0f
#define YAW_KI_MA_PER_RAD_S     4000.0f
#define YAW_I_LIMIT_MA          1500.0f
#define YAW_CURRENT_LIMIT_A        8.0f

/* pitch：位置 PID -> 速度 PI -> 电流。 */
#define PITCH_KP                  12.0f
#define PITCH_KI                   0.8f
#define PITCH_KD                   0.12f
#define PITCH_OUT_RPM            120.0f
#define PITCH_I_LIMIT_RPM          0.70f
#define PITCH_DEADZONE_DEG         0.03f
#define PITCH_TRAVEL_LIMIT_DEG    95.0f
#define PITCH_FB_TIMEOUT_MS       50u
#define PITCH_RATE_KP_MA_PER_RPM  30.0f
#define PITCH_RATE_KI_MA_PER_RPM_S 1.0f
#define PITCH_CURRENT_LIMIT_A      3.0f
#define PITCH_MAX_SPEED_RPM      250.0f
#define PITCH_PLATFORM_SIGN       (-1.0f)

/* 视觉外环：远近距离共用同一组参数。 */
#define VIS_KP_YAW                 9.0f
#define VIS_KI_YAW                 0.0f
#define VIS_KD_YAW                 0.6f
#define VIS_KP_PITCH               8.0f
#define VIS_KI_PITCH               0.0f
#define VIS_KD_PITCH               0.25f
#define VIS_SIGN_YAW              (-1.0f)
#define VIS_SIGN_PITCH            (-1.0f)
#define VIS_RATE_YAW_DPS         250.0f
#define VIS_RATE_PITCH_DPS       250.0f
#define VIS_I_LIMIT_DEG           30.0f
#define VIS_D_LIMIT_DEG           30.0f
#define VIS_YAW_LIMIT_DEG        120.0f
#define VIS_PITCH_LIMIT_DEG       20.0f
#define VIS_ERROR_YAW_LIMIT_DEG   30.0f
#define VIS_ERROR_PITCH_LIMIT_DEG 20.0f
#define VIS_DEADBAND_YAW_DEG       0.20f
#define VIS_DEADBAND_PITCH_DEG     0.20f
#define VIS_FREEZE_MS             150u

/* 短时运动投影补偿。 */
#define VIS_LATENCY_COMP_GAIN       0.70f
#define VIS_LATENCY_MAX_MS          90.0f
#define VIS_LATENCY_YAW_LIMIT_DEG   10.0f
#define VIS_LATENCY_PITCH_LIMIT_DEG  8.0f

/* 视觉单帧大跳保护。 */
#define VIS_JUMP_YAW_DEG           5.0f
#define VIS_JUMP_PITCH_DEG         2.0f
#define VIS_JUMP_MATCH_YAW_DEG     2.0f
#define VIS_JUMP_MATCH_PITCH_DEG   0.8f
#define VIS_JUMP_RETURN_YAW_DEG    1.0f
#define VIS_JUMP_RETURN_PITCH_DEG  0.5f

/* 故障位。 */
#define FAULT_NONE               0x00u
#define FAULT_PITCH_FB_LOST      0x01u
#define FAULT_PITCH_RUNAWAY      0x02u

typedef enum
{
    YUNTAI_STATE_BMI088_INIT = 0,
    YUNTAI_STATE_MOTOR_BOOT,
    YUNTAI_STATE_CLOSED_LOOP,
    YUNTAI_STATE_SET_TORQUE_MODE,
    YUNTAI_STATE_LOCK,
    YUNTAI_STATE_RUNNING
} YuntaiState;

typedef struct
{
    float raw;
    float prev;
    float cont;
    float spd;
    uint32_t t_ms;
    uint8_t seen;
} EncFb_t;

typedef struct
{
    float integral_deg;
    float prev_error_deg;
    uint8_t have_prev;
} VisPidState_t;

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint32_t last_stage;
    uint32_t last_ms;
    uint32_t fault_type;
    uint32_t cfsr;
    uint32_t hfsr;
    int16_t yaw_torque_x100;
    int16_t pitch_torque_x100;
} YuntaiRetained_t;

#define YUNTAI_RETAINED_MAGIC 0x59414E54u

__attribute__((section(".noinit"), used))
static YuntaiRetained_t g_retained;

static float gyro[3];
static float gyro_c[3];
static float acc[3];
static float temp;
static float imuQuat[4];
static float imuAngle[3];

static YuntaiState g_state = YUNTAI_STATE_BMI088_INIT;
static uint32_t g_state_ms;
static uint8_t g_fast_recovery;
static uint32_t g_boot_reset_reason;
static uint32_t g_boot_stage;
static uint32_t g_boot_last_ms;
static uint32_t g_boot_fault_type;
static uint32_t g_boot_cfsr;
static uint32_t g_boot_hfsr;

static uint32_t g_last_imu_ms;
static uint32_t g_last_ctrl_ms;
static uint32_t g_last_hold_ms;
static uint32_t g_last_read_ms;
static uint32_t g_last_uart10_ms;
static uint8_t g_cmd_sent;
static uint8_t g_imu_ok;
static uint8_t g_bias_cal;
static uint8_t g_uart10_boot_logged;
static float g_gyro_bias[3];
static float g_gyro_sum[3];
static uint32_t g_gyro_n;

static PID_Controller g_pitch_pid;
static EncFb_t g_pitch_fb;

static float g_yaw_lock_rad;
static float g_pitch_plat_lock;
static float g_plat_dtheta;
static float g_pitch_motor_lock;
static float g_pitch_tgt_deg;

static float g_rate_i;
static float g_rate_iq_ma;
static float g_pitch_tq_i;
static float g_pitch_iq_ma;
static float g_pitch_cmd_rpm;
static float g_pitch_pos_deg;

static VisPidState_t g_vis_pid_yaw;
static VisPidState_t g_vis_pid_pitch;
static float g_vis_yaw_rate_dps;
static float g_vis_pitch_rate_dps;
static float g_vis_yaw_error_deg;
static float g_vis_pitch_error_deg;
static float g_vis_yaw_angle_deg;
static float g_vis_pitch_angle_deg;
static float g_vis_jump_yaw_deg;
static float g_vis_jump_pitch_deg;
static uint8_t g_vis_jump_pending;
static uint32_t g_vis_last_ms;
static uint32_t g_vis_applied_ms;

static uint32_t g_vis_fps_win_ms;
static uint32_t g_vis_fps_frames;
static uint8_t g_vis_fps_seq;
static uint8_t g_vis_fps_mode = 0xFFu;
static float g_vis_fps;
static uint8_t g_prev_mode = 0xFFu;
static uint8_t g_fault;

static float clampf(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }
    if (value < low)
    {
        return low;
    }
    return value;
}

static float angle_diff_rad(float target, float current)
{
    return normalize_angle_rad(target - current);
}

static void ahrs_init(float q[4])
{
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
}

static void get_angle(const float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                   2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

static void fmt_f(char *out, int cap, float value, int decimals)
{
    long mul = (decimals == 2) ? 100L : ((decimals == 1) ? 10L : 1L);
    int negative = (value < 0.0f);
    long scaled;

    if (negative)
    {
        value = -value;
    }
    scaled = (long)(value * (float)mul + 0.5f);
    if (decimals == 2)
    {
        snprintf(out, (size_t)cap, "%s%ld.%02ld", negative ? "-" : "",
                 scaled / 100L, scaled % 100L);
    }
    else if (decimals == 1)
    {
        snprintf(out, (size_t)cap, "%s%ld.%ld", negative ? "-" : "",
                 scaled / 10L, scaled % 10L);
    }
    else
    {
        snprintf(out, (size_t)cap, "%s%ld", negative ? "-" : "", scaled);
    }
}

void yuntai_mark_stage(uint32_t stage)
{
    uint32_t now = HAL_GetTick();
    if ((g_retained.last_stage != stage) ||
        ((now - g_retained.last_ms) >= 100u))
    {
        g_retained.last_stage = stage;
        g_retained.last_ms = now;
    }
}

void yuntai_record_fault(uint32_t fault_type)
{
    g_retained.fault_type = fault_type;
    g_retained.cfsr = SCB->CFSR;
    g_retained.hfsr = SCB->HFSR;
    g_retained.sequence++;
    g_retained.magic = YUNTAI_RETAINED_MAGIC;
    __DSB();
}

static void enc_update(EncFb_t *feedback, float raw, float speed_rpm,
                       uint32_t now)
{
    if (speed_rpm > -30000.0f)
    {
        feedback->spd = speed_rpm;
    }
    if (!feedback->seen)
    {
        feedback->raw = raw;
        feedback->prev = raw;
        feedback->cont = raw;
        feedback->seen = 1u;
        feedback->t_ms = now;
        return;
    }

    {
        float delta = raw - feedback->prev;
        if (delta > 180.0f)
        {
            delta -= 360.0f;
        }
        else if (delta < -180.0f)
        {
            delta += 360.0f;
        }
        if (fabsf(delta) > 90.0f)
        {
            feedback->t_ms = now;
            return;
        }
        feedback->cont += delta;
    }
    feedback->raw = raw;
    feedback->prev = raw;
    feedback->t_ms = now;
}

static void yuntai_read_pitch_feedback(uint32_t now)
{
    CanRxFrame_t frame;
    float position;
    int16_t speed_raw;
    int16_t current_raw;

    while (can_bsp_get_frame(&hfdcan2, &frame))
    {
        if (jc_parse_speed_reply(frame.data, frame.len, &position,
                                 &speed_raw, &current_raw))
        {
            enc_update(&g_pitch_fb, position,
                       (float)speed_raw / 100.0f, now);
        }
        else if (jc_parse_position_reply(frame.data, frame.len, &position))
        {
            enc_update(&g_pitch_fb, position, -32768.0f, now);
        }
    }
}

static uint8_t enc_fresh(const EncFb_t *feedback, uint32_t now,
                         uint32_t timeout_ms)
{
    return (uint8_t)((feedback->seen != 0u) &&
                     ((now - feedback->t_ms) < timeout_ms));
}

static void vis_pid_reset(VisPidState_t *pid)
{
    pid->integral_deg = 0.0f;
    pid->prev_error_deg = 0.0f;
    pid->have_prev = 0u;
}

static float vis_pid_step(VisPidState_t *pid, float error_deg, float dt,
                          float kp, float ki, float kd, float output_limit)
{
    float proportional;
    float derivative = 0.0f;
    float next_integral;
    float output;

    dt = clampf(dt, 0.001f, 0.100f);
    proportional = kp * error_deg;
    if (pid->have_prev != 0u)
    {
        derivative = clampf(
            kd * ((error_deg - pid->prev_error_deg) / dt),
            -VIS_D_LIMIT_DEG, VIS_D_LIMIT_DEG
        );
    }
    next_integral = clampf(
        pid->integral_deg + (ki * error_deg * dt),
        -VIS_I_LIMIT_DEG, VIS_I_LIMIT_DEG
    );
    output = proportional + next_integral + derivative;

    if (((output > output_limit) && (error_deg > 0.0f)) ||
        ((output < -output_limit) && (error_deg < 0.0f)))
    {
        output = proportional + pid->integral_deg + derivative;
    }
    else
    {
        pid->integral_deg = next_integral;
    }
    pid->prev_error_deg = error_deg;
    pid->have_prev = 1u;
    return clampf(output, -output_limit, output_limit);
}

static void vis_outer_reset(void)
{
    vis_pid_reset(&g_vis_pid_yaw);
    vis_pid_reset(&g_vis_pid_pitch);
    g_vis_yaw_rate_dps = 0.0f;
    g_vis_pitch_rate_dps = 0.0f;
    g_vis_yaw_error_deg = 0.0f;
    g_vis_pitch_error_deg = 0.0f;
    g_vis_yaw_angle_deg = 0.0f;
    g_vis_pitch_angle_deg = 0.0f;
    g_vis_jump_yaw_deg = 0.0f;
    g_vis_jump_pitch_deg = 0.0f;
    g_vis_jump_pending = 0u;
    g_vis_last_ms = 0u;
    g_vis_applied_ms = 0u;
}

static float vis_total_age_ms(const GimbalCmd_t *cmd, uint32_t now)
{
    float age_ms = (float)cmd->vis_proc_age_ms;
    if (cmd->px_ms != 0u)
    {
        age_ms += (float)(now - cmd->px_ms);
    }
    return clampf(age_ms, 0.0f, 9999.0f);
}

static void vis_outer_step(const GimbalCmd_t *cmd, uint8_t mode, uint32_t now)
{
    if ((mode != GP_MODE_AIM) || (cmd->rx_count == 0u) ||
        (cmd->aim_valid == 0u) ||
        (vis_total_age_ms(cmd, now) > (float)VIS_FREEZE_MS))
    {
        g_vis_yaw_rate_dps = 0.0f;
        g_vis_pitch_rate_dps = 0.0f;
        g_vis_jump_pending = 0u;
        return;
    }

    if (cmd->px_ms == g_vis_last_ms)
    {
        return;
    }

    {
        float frame_dt = 0.0125f;
        float yaw_deg = VIS_SIGN_YAW * cmd->vis_yaw_deg;
        float pitch_deg = VIS_SIGN_PITCH * cmd->vis_pitch_deg;
        float latency_s;
        float yaw_rate;
        float pitch_rate;

        if ((g_vis_applied_ms != 0u) && (cmd->px_ms > g_vis_applied_ms))
        {
            frame_dt = (float)(cmd->px_ms - g_vis_applied_ms) / 1000.0f;
        }
        g_vis_last_ms = cmd->px_ms;

        latency_s = clampf(vis_total_age_ms(cmd, now), 0.0f,
                           VIS_LATENCY_MAX_MS) * 0.001f;
        if (cmd->target_velocity_valid != 0u)
        {
            yaw_deg += clampf(
                VIS_SIGN_YAW * cmd->target_yaw_rate_dps * latency_s *
                    VIS_LATENCY_COMP_GAIN,
                -VIS_LATENCY_YAW_LIMIT_DEG,
                VIS_LATENCY_YAW_LIMIT_DEG
            );
            pitch_deg += clampf(
                -VIS_SIGN_PITCH * cmd->target_pitch_rate_dps * latency_s *
                    VIS_LATENCY_COMP_GAIN,
                -VIS_LATENCY_PITCH_LIMIT_DEG,
                VIS_LATENCY_PITCH_LIMIT_DEG
            );
        }

        yaw_deg = clampf(yaw_deg, -VIS_ERROR_YAW_LIMIT_DEG,
                         VIS_ERROR_YAW_LIMIT_DEG);
        pitch_deg = clampf(pitch_deg, -VIS_ERROR_PITCH_LIMIT_DEG,
                           VIS_ERROR_PITCH_LIMIT_DEG);
        if (fabsf(yaw_deg) < VIS_DEADBAND_YAW_DEG)
        {
            yaw_deg = 0.0f;
        }
        if (fabsf(pitch_deg) < VIS_DEADBAND_PITCH_DEG)
        {
            pitch_deg = 0.0f;
        }
        g_vis_yaw_error_deg = yaw_deg;
        g_vis_pitch_error_deg = pitch_deg;

        if ((g_vis_pid_yaw.have_prev != 0u) &&
            (g_vis_pid_pitch.have_prev != 0u))
        {
            float yaw_from_pending =
                fabsf(yaw_deg - g_vis_jump_yaw_deg);
            float pitch_from_pending =
                fabsf(pitch_deg - g_vis_jump_pitch_deg);
            float yaw_from_last =
                fabsf(yaw_deg - g_vis_pid_yaw.prev_error_deg);
            float pitch_from_last =
                fabsf(pitch_deg - g_vis_pid_pitch.prev_error_deg);

            if (g_vis_jump_pending != 0u)
            {
                if ((yaw_from_pending <= VIS_JUMP_MATCH_YAW_DEG) &&
                    (pitch_from_pending <= VIS_JUMP_MATCH_PITCH_DEG))
                {
                    g_vis_jump_pending = 0u;
                }
                else if ((yaw_from_last <= VIS_JUMP_RETURN_YAW_DEG) &&
                         (pitch_from_last <= VIS_JUMP_RETURN_PITCH_DEG))
                {
                    g_vis_jump_pending = 0u;
                }
                else
                {
                    g_vis_jump_yaw_deg = yaw_deg;
                    g_vis_jump_pitch_deg = pitch_deg;
                    g_vis_yaw_rate_dps = 0.0f;
                    g_vis_pitch_rate_dps = 0.0f;
                    return;
                }
            }
            else if ((yaw_from_last > VIS_JUMP_YAW_DEG) ||
                     (pitch_from_last > VIS_JUMP_PITCH_DEG))
            {
                g_vis_jump_pending = 1u;
                g_vis_jump_yaw_deg = yaw_deg;
                g_vis_jump_pitch_deg = pitch_deg;
                g_vis_yaw_rate_dps = 0.0f;
                g_vis_pitch_rate_dps = 0.0f;
                return;
            }
        }

        yaw_rate = vis_pid_step(&g_vis_pid_yaw, yaw_deg, frame_dt,
                                VIS_KP_YAW, VIS_KI_YAW, VIS_KD_YAW,
                                VIS_RATE_YAW_DPS);
        pitch_rate = vis_pid_step(&g_vis_pid_pitch, pitch_deg, frame_dt,
                                  VIS_KP_PITCH, VIS_KI_PITCH, VIS_KD_PITCH,
                                  VIS_RATE_PITCH_DPS);
        g_vis_yaw_rate_dps = yaw_rate;
        g_vis_pitch_rate_dps = pitch_rate;
        g_vis_yaw_angle_deg = clampf(
            g_vis_yaw_angle_deg + yaw_rate * frame_dt,
            -VIS_YAW_LIMIT_DEG, VIS_YAW_LIMIT_DEG
        );
        g_vis_pitch_angle_deg = clampf(
            g_vis_pitch_angle_deg + pitch_rate * frame_dt,
            -VIS_PITCH_LIMIT_DEG, VIS_PITCH_LIMIT_DEG
        );
        g_vis_applied_ms = cmd->px_ms;
    }
}

static void yuntai_lock_reference(void)
{
    g_yaw_lock_rad = imuAngle[0];
    g_pitch_plat_lock = imuAngle[1];
    g_plat_dtheta = 0.0f;
    g_pitch_motor_lock = g_pitch_fb.seen ? g_pitch_fb.cont : 0.0f;
    g_pitch_tgt_deg = g_pitch_motor_lock;
    g_pitch_tq_i = 0.0f;
    g_pitch_iq_ma = 0.0f;
    g_rate_i = 0.0f;
    g_rate_iq_ma = 0.0f;
    PID_Reset(&g_pitch_pid);
    vis_outer_reset();
}

static void yuntai_imu_update(uint32_t now)
{
    uint8_t axis;

    if ((now - g_last_imu_ms) < IMU_PERIOD_MS)
    {
        return;
    }
    g_last_imu_ms = now;
    yuntai_mark_stage(16u);
    BMI088_read(gyro, acc, &temp);
    yuntai_mark_stage(17u);

    for (axis = 0u; axis < 3u; axis++)
    {
        gyro_c[axis] = gyro[axis] - g_gyro_bias[axis];
        if (g_bias_cal != 0u)
        {
            g_gyro_sum[axis] += gyro[axis];
        }
    }
    if (g_bias_cal != 0u)
    {
        g_gyro_n++;
    }

    MahonyAHRSupdateIMU(imuQuat, gyro_c[0], gyro_c[1], gyro_c[2],
                        acc[0], acc[1], acc[2]);
    get_angle(imuQuat, &imuAngle[0], &imuAngle[1], &imuAngle[2]);
}

static void gyro_bias_finalize(void)
{
    uint8_t axis;

    if (g_gyro_n < GYRO_BIAS_MIN_SAMPLES)
    {
        for (axis = 0u; axis < 3u; axis++)
        {
            g_gyro_bias[axis] = 0.0f;
        }
    }
    else
    {
        for (axis = 0u; axis < 3u; axis++)
        {
            g_gyro_bias[axis] = clampf(
                g_gyro_sum[axis] / (float)g_gyro_n,
                -GYRO_BIAS_MAX, GYRO_BIAS_MAX
            );
        }
    }
    g_bias_cal = 0u;
}

static void yuntai_control(uint32_t now)
{
    const GimbalCmd_t *cmd = gimbal_link_cmd();
    const uint8_t aim_mode = (uint8_t)(cmd->mode == GP_MODE_AIM);
    const float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
    uint8_t pitch_fb_ok;

    yuntai_mark_stage(30u);
    if (cmd->mode != g_prev_mode)
    {
        g_prev_mode = cmd->mode;
        if (aim_mode != 0u)
        {
            vis_outer_reset();
        }
    }

    /* 俯仰编码器主动查询与控制环同频：1 kHz。 */
    if ((now - g_last_read_ms) >= 1u)
    {
        g_last_read_ms = now;
        jc_read_position(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
    }

    yuntai_mark_stage(31u);
    yuntai_read_pitch_feedback(now);
    pitch_fb_ok = enc_fresh(&g_pitch_fb, now, PITCH_FB_TIMEOUT_MS);

    if (g_imu_ok != 0u)
    {
        g_plat_dtheta = clampf(
            angle_diff_rad(imuAngle[1], g_pitch_plat_lock),
            -1.75f, 1.75f
        );
    }
    if (gimbal_link_take_zero_req())
    {
        yuntai_lock_reference();
    }

    yuntai_mark_stage(32u);
    vis_outer_step(cmd, cmd->mode, now);

    yuntai_mark_stage(33u);
    {
        float visual_rad = aim_mode ?
            (g_vis_yaw_angle_deg / 57.29578f) : 0.0f;
        float yaw_error = angle_diff_rad(
            g_yaw_lock_rad + visual_rad, imuAngle[0]
        );
        float current_limit_ma = YAW_CURRENT_LIMIT_A * 1000.0f;

        g_rate_i += YAW_KI_MA_PER_RAD_S * yaw_error * dt;
        g_rate_i = clampf(g_rate_i, -YAW_I_LIMIT_MA, YAW_I_LIMIT_MA);
        g_rate_iq_ma = clampf(
            YAW_KP_MA_PER_RAD * yaw_error
            - YAW_KD_MA_PER_RAD_S * gyro_c[2]
            + g_rate_i,
            -current_limit_ma, current_limit_ma
        );
        jc_set_torque_x100(
            &hfdcan1, YUNTAI_MOTOR_YAW_ID,
            (int16_t)(g_rate_iq_ma * 0.004f)
        );
    }

    yuntai_mark_stage(34u);
    if (pitch_fb_ok == 0u)
    {
        PID_Reset(&g_pitch_pid);
        g_pitch_tq_i = 0.0f;
        g_pitch_iq_ma = 0.0f;
        g_pitch_cmd_rpm = 0.0f;
        g_fault |= FAULT_PITCH_FB_LOST;
    }
    else
    {
        float platform_deg = clampf(
            g_plat_dtheta * 57.29578f * PITCH_PLATFORM_SIGN,
            -PITCH_TRAVEL_LIMIT_DEG, PITCH_TRAVEL_LIMIT_DEG
        );
        float visual_deg = aim_mode ? g_vis_pitch_angle_deg : 0.0f;
        float pitch_error;
        float speed_error;
        float current_limit_ma = PITCH_CURRENT_LIMIT_A * 1000.0f;

        g_pitch_tgt_deg = clampf(
            g_pitch_motor_lock + platform_deg + visual_deg,
            g_pitch_motor_lock - PITCH_TRAVEL_LIMIT_DEG,
            g_pitch_motor_lock + PITCH_TRAVEL_LIMIT_DEG
        );
        pitch_error = g_pitch_tgt_deg - g_pitch_fb.cont;
        PID_Update(&g_pitch_pid, pitch_error, 0.0f, dt);
        g_pitch_cmd_rpm = clampf(
            g_pitch_pid.out, -PITCH_OUT_RPM, PITCH_OUT_RPM
        );

        speed_error = g_pitch_cmd_rpm - g_pitch_fb.spd;
        g_pitch_tq_i += PITCH_RATE_KI_MA_PER_RPM_S * speed_error * dt;
        g_pitch_tq_i = clampf(
            g_pitch_tq_i, -current_limit_ma, current_limit_ma
        );
        g_pitch_iq_ma = clampf(
            PITCH_RATE_KP_MA_PER_RPM * speed_error + g_pitch_tq_i,
            -current_limit_ma, current_limit_ma
        );

        if (fabsf(g_pitch_fb.spd) > PITCH_MAX_SPEED_RPM)
        {
            g_pitch_iq_ma = 0.0f;
            g_pitch_tq_i = 0.0f;
            g_fault |= FAULT_PITCH_RUNAWAY;
        }
        else
        {
            g_fault &= (uint8_t)~FAULT_PITCH_RUNAWAY;
        }
        jc_set_torque_x100(
            &hfdcan2, YUNTAI_MOTOR_PITCH_ID,
            (int16_t)(g_pitch_iq_ma * 0.004f)
        );
    }

    g_pitch_pos_deg = g_pitch_fb.cont;
    if (pitch_fb_ok != 0u)
    {
        g_retained.yaw_torque_x100 =
            (int16_t)clampf(g_rate_iq_ma * 0.004f, -30000.0f, 30000.0f);
        g_retained.pitch_torque_x100 =
            (int16_t)clampf(g_pitch_iq_ma * 0.004f, -30000.0f, 30000.0f);
        g_retained.sequence++;
        g_retained.magic = YUNTAI_RETAINED_MAGIC;
        __DSB();
    }

}

static void uart10_append_float(char *buffer, size_t capacity, size_t *position,
                                float value, uint8_t decimals)
{
    char text[16];
    int written;

    fmt_f(text, sizeof(text), value, decimals);
    written = snprintf(buffer + *position, capacity - *position,
                       ",%s", text);
    if (written > 0)
    {
        *position += (size_t)written;
        if (*position >= capacity)
        {
            *position = capacity - 1u;
        }
    }
}

static void yuntai_uart10_debug(uint32_t now)
{
    const GimbalCmd_t *cmd = gimbal_link_cmd();
    char line[240];
    size_t position;
    float visual_age;
    float yaw_relative_deg;
    int written;

    if (g_uart10_boot_logged == 0u)
    {
        written = snprintf(
            line, sizeof(line),
            "BOOT,rst=%lu,stage=%lu,stms=%lu,flt=%lu,"
            "cfsr=%08lX,hfsr=%08lX\r\n",
            (unsigned long)g_boot_reset_reason,
            (unsigned long)g_boot_stage,
            (unsigned long)g_boot_last_ms,
            (unsigned long)g_boot_fault_type,
            (unsigned long)g_boot_cfsr,
            (unsigned long)g_boot_hfsr
        );
        if ((written > 0) && debug_uart10_send(line))
        {
            g_uart10_boot_logged = 1u;
        }
        return;
    }

    if ((now - g_last_uart10_ms) < UART10_LOG_PERIOD_MS)
    {
        return;
    }
    g_last_uart10_ms = now;

    if (cmd->mode != g_vis_fps_mode)
    {
        g_vis_fps_mode = cmd->mode;
        g_vis_fps_win_ms = now;
        g_vis_fps_frames = 0u;
        g_vis_fps_seq = cmd->vision_seq;
        g_vis_fps = 0.0f;
    }
    else
    {
        g_vis_fps_frames += (uint8_t)(cmd->vision_seq - g_vis_fps_seq);
        g_vis_fps_seq = cmd->vision_seq;
        if ((now - g_vis_fps_win_ms) >= 1000u)
        {
            g_vis_fps = (float)g_vis_fps_frames * 1000.0f /
                        (float)(now - g_vis_fps_win_ms);
            g_vis_fps_win_ms = now;
            g_vis_fps_frames = 0u;
        }
    }

    visual_age = vis_total_age_ms(cmd, now);
    yaw_relative_deg =
        angle_diff_rad(imuAngle[0], g_yaw_lock_rad) * 57.29578f;

    written = snprintf(
        line, sizeof(line), "%lu,%u,%u,%lu,%u,%u,%u,%u,%u",
        (unsigned long)now,
        (unsigned)cmd->mode,
        (unsigned)cmd->aim_valid,
        (unsigned long)visual_age,
        (unsigned)cmd->vision_seq,
        (unsigned)cmd->px_ut,
        (unsigned)cmd->px_vt,
        (unsigned)cmd->px_us,
        (unsigned)cmd->px_vs
    );
    if (written <= 0)
    {
        return;
    }
    position = (size_t)written;
    uart10_append_float(line, sizeof(line), &position,
                        g_vis_yaw_error_deg, 2);
    uart10_append_float(line, sizeof(line), &position,
                        g_vis_pitch_error_deg, 2);
    uart10_append_float(line, sizeof(line), &position,
                        g_vis_yaw_rate_dps, 1);
    uart10_append_float(line, sizeof(line), &position,
                        g_vis_pitch_rate_dps, 1);
    uart10_append_float(line, sizeof(line), &position,
                        g_vis_yaw_angle_deg, 2);
    uart10_append_float(line, sizeof(line), &position,
                        g_vis_pitch_angle_deg, 2);
    uart10_append_float(line, sizeof(line), &position,
                        yaw_relative_deg, 2);
    uart10_append_float(line, sizeof(line), &position,
                        gyro_c[2] * 57.29578f, 2);
    uart10_append_float(line, sizeof(line), &position, g_rate_iq_ma, 0);
    uart10_append_float(line, sizeof(line), &position, g_pitch_pos_deg, 2);
    uart10_append_float(line, sizeof(line), &position, g_pitch_tgt_deg, 2);
    uart10_append_float(line, sizeof(line), &position, g_pitch_cmd_rpm, 1);
    uart10_append_float(line, sizeof(line), &position, g_pitch_iq_ma, 0);
    uart10_append_float(line, sizeof(line), &position, g_vis_fps, 1);
    snprintf(line + position, sizeof(line) - position,
             ",%lu\r\n", (unsigned long)g_fault);
    (void)debug_uart10_send(line);
}

static void yuntai_hold_torque(uint32_t now)
{
    if ((now - g_last_hold_ms) < 50u)
    {
        return;
    }
    g_last_hold_ms = now;
    jc_set_torque_x100(&hfdcan1, YUNTAI_MOTOR_YAW_ID, 0);
    jc_set_torque_x100(&hfdcan2, YUNTAI_MOTOR_PITCH_ID, 0);
}

void yuntai_init(void)
{
    uint32_t reset_flags;

    can_bsp_init();
    reset_flags = RCC->RSR & ~RCC_RSR_RMVF;
    if ((reset_flags & RCC_RSR_IWDG1RSTF) != 0u)
    {
        g_boot_reset_reason = 1u;
    }
    else if ((reset_flags & RCC_RSR_SFTRSTF) != 0u)
    {
        g_boot_reset_reason = 2u;
    }
    else if ((reset_flags & RCC_RSR_BORRSTF) != 0u)
    {
        g_boot_reset_reason = 3u;
    }
    else if ((reset_flags & RCC_RSR_PORRSTF) != 0u)
    {
        g_boot_reset_reason = 4u;
    }
    else if ((reset_flags & RCC_RSR_PINRSTF) != 0u)
    {
        g_boot_reset_reason = 5u;
    }
    else
    {
        g_boot_reset_reason = 9u;
    }
    g_fast_recovery = (g_boot_reset_reason == 1u) ? 1u : 0u;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    if (g_fast_recovery != 0u)
    {
        g_last_hold_ms = HAL_GetTick() - 50u;
        if (g_retained.magic == YUNTAI_RETAINED_MAGIC)
        {
            g_boot_stage = g_retained.last_stage;
            g_boot_last_ms = g_retained.last_ms;
            g_boot_fault_type = g_retained.fault_type;
            g_boot_cfsr = g_retained.cfsr;
            g_boot_hfsr = g_retained.hfsr;
        }
    }

    jc_set_torque_mode(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
    jc_set_torque_mode(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
    jc_enter_closed_loop(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
    jc_enter_closed_loop(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
    if ((g_fast_recovery != 0u) &&
        (g_retained.magic == YUNTAI_RETAINED_MAGIC))
    {
        jc_set_torque_x100(&hfdcan1, YUNTAI_MOTOR_YAW_ID,
                           g_retained.yaw_torque_x100);
        jc_set_torque_x100(&hfdcan2, YUNTAI_MOTOR_PITCH_ID,
                           g_retained.pitch_torque_x100);
    }
    else
    {
        jc_set_torque_x100(&hfdcan1, YUNTAI_MOTOR_YAW_ID, 0);
        jc_set_torque_x100(&hfdcan2, YUNTAI_MOTOR_PITCH_ID, 0);
    }

    if (g_fast_recovery == 0u)
    {
        g_retained.fault_type = 0u;
        g_retained.cfsr = 0u;
        g_retained.hfsr = 0u;
    }

    mahonySampleFreq = 1000.0f / (float)IMU_PERIOD_MS;
    ahrs_init(imuQuat);
    g_fault = FAULT_NONE;
    g_state = YUNTAI_STATE_BMI088_INIT;
    g_state_ms = HAL_GetTick();
    g_cmd_sent = 0u;
}

void yuntai_control_loop(void)
{
    uint32_t now = HAL_GetTick();

    yuntai_mark_stage(10u);
    switch (g_state)
    {
    case YUNTAI_STATE_BMI088_INIT:
        if ((now - g_state_ms) >= (g_fast_recovery ? 1u : BMI_RETRY_MS))
        {
            if (BMI088_init() == BMI088_NO_ERROR)
            {
                g_imu_ok = 1u;
                ahrs_init(imuQuat);
                g_gyro_sum[0] = 0.0f;
                g_gyro_sum[1] = 0.0f;
                g_gyro_sum[2] = 0.0f;
                g_gyro_n = 0u;
                g_bias_cal = 1u;
                g_state = YUNTAI_STATE_MOTOR_BOOT;
                g_state_ms = now;
            }
            else
            {
                g_state_ms = now;
            }
        }
        break;

    case YUNTAI_STATE_MOTOR_BOOT:
        if (g_fast_recovery != 0u)
        {
            yuntai_hold_torque(now);
        }
        yuntai_imu_update(now);
        if ((now - g_state_ms) >
            (g_fast_recovery ? 120u : MOTOR_BOOT_DELAY_MS))
        {
            gyro_bias_finalize();
            g_state = YUNTAI_STATE_CLOSED_LOOP;
            g_state_ms = now;
            g_cmd_sent = 0u;
        }
        break;

    case YUNTAI_STATE_CLOSED_LOOP:
        if (g_cmd_sent == 0u)
        {
            g_cmd_sent = 1u;
            jc_enter_closed_loop(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
            jc_enter_closed_loop(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
        }
        if ((now - g_state_ms) >
            (g_fast_recovery ? 80u : CLOSED_LOOP_DELAY_MS))
        {
            g_state = YUNTAI_STATE_SET_TORQUE_MODE;
            g_state_ms = now;
            g_cmd_sent = 0u;
        }
        break;

    case YUNTAI_STATE_SET_TORQUE_MODE:
        if (g_cmd_sent == 0u)
        {
            g_cmd_sent = 1u;
            jc_set_torque_mode(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
            jc_set_torque_mode(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
            jc_enter_closed_loop(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
            jc_enter_closed_loop(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
        }
        if ((now - g_state_ms) >
            (g_fast_recovery ? 80u : MODE_DELAY_MS))
        {
            g_state = YUNTAI_STATE_LOCK;
            g_state_ms = now;
            g_last_imu_ms = now - IMU_PERIOD_MS;
            g_last_hold_ms = now - 50u;
            g_last_read_ms = now - 1u;
            g_gyro_sum[0] = 0.0f;
            g_gyro_sum[1] = 0.0f;
            g_gyro_sum[2] = 0.0f;
            g_gyro_n = 0u;
            g_bias_cal = 1u;
        }
        break;

    case YUNTAI_STATE_LOCK:
        if ((g_bias_cal != 0u) &&
            ((now - g_state_ms) >
             (g_fast_recovery ? 120u : LOCK_BIAS_CAL_MS)))
        {
            gyro_bias_finalize();
        }
        yuntai_imu_update(now);
        if ((now - g_last_read_ms) >= 1u)
        {
            g_last_read_ms = now;
            jc_read_position(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
        }
        yuntai_hold_torque(now);
        yuntai_read_pitch_feedback(now);

        if (((now - g_state_ms) > LOCK_DELAY_MS) &&
            (g_pitch_fb.seen != 0u))
        {
            PID_Init(&g_pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD,
                     PITCH_OUT_RPM);
            g_pitch_pid.integral_limit = PITCH_I_LIMIT_RPM;
            g_pitch_pid.dead_zone = PITCH_DEADZONE_DEG;
            yuntai_lock_reference();
            g_state = YUNTAI_STATE_RUNNING;
            g_state_ms = now;
            g_last_ctrl_ms = now - CONTROL_PERIOD_MS;
        }
        break;

    case YUNTAI_STATE_RUNNING:
        yuntai_mark_stage(11u);
        yuntai_imu_update(now);
        if ((now - g_last_ctrl_ms) >= CONTROL_PERIOD_MS)
        {
            g_last_ctrl_ms = now;
            yuntai_mark_stage(13u);
            yuntai_control(now);
        }
        yuntai_uart10_debug(now);
        break;

    default:
        g_state = YUNTAI_STATE_BMI088_INIT;
        g_state_ms = now;
        break;
    }

    yuntai_mark_stage(15u);
    gimbal_link_poll(now);
}
