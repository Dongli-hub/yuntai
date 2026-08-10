/*
 * pid.h
 *
 * PID Controller for motor control
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include "main.h"

// PID控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数

    float out;          // 当前输出

    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float prev_measurement; // 上一次测量值

    float output_limit; // 输出限幅
    float integral_limit; // 积分限幅
    float dead_zone;    // 死区

    uint32_t last_time; // 上次更新时间
} PID_Controller;

// 函数声明
void PID_Init(PID_Controller* pid, float kp, float ki, float kd, float output_limit);
void PID_Update(PID_Controller* pid, float setpoint, float measurement, float dt);
void PID_Reset(PID_Controller* pid);
void PID_SetParams(PID_Controller* pid, float kp, float ki, float kd);

#endif /* INC_PID_H_ */
