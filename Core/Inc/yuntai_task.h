#ifndef __YUNTAI_TASK_H__
#define __YUNTAI_TASK_H__

#include "main.h"

// 水平电机ID (FDCAN1)
#define YUNTAI_MOTOR_YAW_ID     1
// 俯仰电机ID (FDCAN2) — 两个电机ID都是1，分别在不同CAN总线
#define YUNTAI_MOTOR_PITCH_ID   1

void yuntai_init(void);
void yuntai_control_loop(void);

#endif /* __YUNTAI_TASK_H__ */
