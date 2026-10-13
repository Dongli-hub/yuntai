#include "pid.h"
#include <math.h>

/* ⚠⚠ 2026-10-12：微分项加一阶低通（原来是**裸微分**，这是"猛地一甩"的来源之一）
 *
 * 为什么必须加：微分 = (e - e_prev)/dt。之前 dt=2ms 时还好，现在控制环提到
 * 1kHz（dt=1ms），同样的角度噪声算出来的"变化率"直接翻倍 —— IMU 角度上
 * 几百分之一度的抖动，经过 /0.001s 就变成几十 °/s，再乘 kd(=10) 就是几十 rpm
 * 的尖峰指令，表现就是"平时不动、偶尔猛甩一下"。
 *
 * 参考代码（QGimbal）的角度环 kd 敢开到 110（比 kp 大 20 倍），就是因为它跑在
 * 电流环上、且微分来自陀螺（本身就低噪）；用户之前给的那段网上例程也明确写了
 * "重点是要做积分分离和微分滤波，否则云台会震动"。
 *
 * 这里用一阶低通：d_filt += (d_raw - d_filt) * dt/(tau+dt)，tau = 5ms（≈32Hz）。
 * 群延迟 ~5ms，相对"猛甩"完全是赚的。
 */
#define PID_D_TAU_S     0.005f

// PID初始化
void PID_Init(PID_Controller* pid, float kp, float ki, float kd, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out = 0.0f;
    pid->output_limit = output_limit;
    pid->integral_limit = output_limit * 0.3f;
    pid->dead_zone = 0.0f;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filt = 0.0f;
    pid->last_time = HAL_GetTick();
}

// PID参数设置
void PID_SetParams(PID_Controller* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// PID重置
void PID_Reset(PID_Controller* pid)
{
    pid->out = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filt = 0.0f;
    pid->last_time = HAL_GetTick();
}

// PID更新计算
void PID_Update(PID_Controller* pid, float setpoint, float measurement, float dt)
{
    uint32_t current_time = HAL_GetTick();
    float actual_dt = (current_time - pid->last_time) / 1000.0f;
    pid->last_time = current_time;

    if (actual_dt > dt * 1.5f || actual_dt < dt * 0.5f) {
        actual_dt = dt;
    }

    float error = setpoint - measurement;

    if (fabsf(error) < pid->dead_zone) {
        error = 0.0f;
    }

    // 比例项
    float p_term = pid->kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * actual_dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->ki * pid->integral;

    // 微分项（带一阶低通，见文件顶部说明）
    {
        float d_raw = (error - pid->prev_error) / actual_dt;
        float a = actual_dt / (PID_D_TAU_S + actual_dt);
        if (a > 1.0f)
        {
            a = 1.0f;
        }
        pid->d_filt += a * (d_raw - pid->d_filt);
    }
    float d_term = pid->kd * pid->d_filt;

    pid->prev_error = error;
    pid->prev_measurement = measurement;

    float output = p_term + i_term + d_term;

    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    pid->out = output;
}
