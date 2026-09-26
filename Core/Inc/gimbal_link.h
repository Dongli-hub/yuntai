/**
  ******************************************************************************
  * @file    gimbal_link.h
  * @brief   H723(USART1) <-> 机载计算机（地瓜派）的链路层
  *
  * 分工：
  *   gimbal_proto.c  —— 纯协议（CRC / 组帧 / 拆帧），不含 HAL，可在 PC 上单测
  *   gimbal_link.c   —— 本文件：USART1 中断收发、命令解释、遥测上报、看门狗
  *
  * 三种数据方向：
  *   下行（地瓜派 -> H723）：AIM 偏置(100Hz) / MODE / SET_ZERO / UNWIND / 心跳
  *   上行（H723 -> 地瓜派）：GIMBAL_STATE 遥测(50Hz) / ACK / TEXT 调试文本
  *
  * 为什么调试文本要走协议发（而不是直接 printf 到串口）：
  *   USART1 现在承载的是二进制协议流，直接 printf 会把帧冲乱。
  *   把文本包成 0x93 TEXT 消息发出去，上位机照样能实时打印，
  *   既保住了"每一步都看得见"的调试习惯，又不破坏协议。
  ******************************************************************************
  */

#ifndef __GIMBAL_LINK_H__
#define __GIMBAL_LINK_H__

#include "main.h"

/* 上位机下发的命令，yuntai_task 只读（内部已限幅） */
typedef struct
{
    uint8_t mode;              /* GP_MODE_xxx */
    uint8_t laser;             /* 1 = 请求点激光 */
    uint8_t aim_valid;         /* 上位机本帧视觉有效 */
    uint8_t flags;             /* 原始 flags，备用 */
    uint8_t boost;             /* 拐角加强中 */
    uint8_t drawing;           /* 正在画圆 */
    uint8_t locked;            /* 上位机已锁定靶心 */
    uint8_t quality;           /* 0~255 跟踪质量 */
    uint8_t unwind_dps;        /* 解绕速度 (°/s)，来自 UNWIND 消息 */
    float   yaw_offset_deg;    /* 偏航偏置（加在上电锁定值上） */
    float   pitch_offset_deg;  /* 俯仰偏置 */
    uint32_t rx_count;         /* 收到多少帧 AIM（用来确认链路真的在动） */
} GimbalCmd_t;

/* 上报给上位机的状态（由 yuntai_task 填充） */
typedef struct
{
    uint8_t  state;            /* 云台启动状态机当前状态 */
    uint8_t  fault;            /* 0 = 正常 */
    float    yaw_deg;
    float    pitch_deg;
    float    roll_deg;
    float    yaw_motor_deg;    /* 相对车身的偏航电机角（绕线监视） */
    float    pitch_motor_deg;
    float    gyro_y_dps;
    float    gyro_z_dps;
    uint8_t  flags;            /* GP_ST_xxx */
    uint32_t uptime_ms;
} GimbalTelem_t;

void               gimbal_link_init(void);
void               gimbal_link_poll(uint32_t now_ms);
void               gimbal_link_rx_isr(void);          /* 在 USART1_IRQHandler 里调用 */
void               gimbal_link_set_telem(const GimbalTelem_t *t);
const GimbalCmd_t *gimbal_link_cmd(void);
uint8_t            gimbal_link_alive(void);           /* 1 = 最近 0.5s 收到过 AIM */
uint8_t            gimbal_link_take_zero_req(void);   /* 读并清"重新锁零"请求 */
void               gimbal_link_log(const char *text); /* 调试文本 -> 上位机 */
/* 同上，但绕过"50ms 最快一条"的限速：调参回执必须发出去 */
void               gimbal_link_log_force(const char *text);
void               gimbal_link_on_param(uint8_t pid, int16_t value); /* 由 yuntai_task 实现 */
const char        *gimbal_link_stats(void);           /* 一行统计，便于随遥测发出 */

#endif /* __GIMBAL_LINK_H__ */
