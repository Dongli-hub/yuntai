#include "pid.h"
#include <math.h>

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

    // 微分项
    float derivative = (error - pid->prev_error) / actual_dt;
    float d_term = pid->kd * derivative;

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
