#ifndef __YUNTAI_TASK_H__
#define __YUNTAI_TASK_H__

#include "main.h"

// 水平电机ID (FDCAN1)
#define YUNTAI_MOTOR_YAW_ID     1
// 俯仰电机ID (FDCAN2) — 两个电机ID都是1，分别在不同CAN总线
#define YUNTAI_MOTOR_PITCH_ID   1

void yuntai_init(void);
void yuntai_control_loop(void);

/* Reset-preserved diagnostics used by the exception handlers. */
#define YUNTAI_FAULT_HARD       1u
#define YUNTAI_FAULT_MEMMANAGE  2u
#define YUNTAI_FAULT_BUS        3u
#define YUNTAI_FAULT_USAGE      4u

void yuntai_record_fault(uint32_t fault_type);
void yuntai_mark_stage(uint32_t stage);

#endif /* __YUNTAI_TASK_H__ */
