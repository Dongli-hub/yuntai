/**
  ******************************************************************************
  * @file    gimbal_link.h
  * @brief   K230 -> H723 单向视觉链路
  *
  * 分工：
  *   gimbal_link.c   —— USART1 接收、ASCII 帧解析、视觉看门狗
  *
  * 数据方向只有一条：
  *   K230 -> H723：
  *     cx,cy,lx,ly,target_vx,target_vy,image_age
  *   丢失帧：
  *     404,404,0,0,0,0,0
  *
  * 像素到角度的换算只在 H723 做一次，避免两端各用一套系数。
  ******************************************************************************
  */

#ifndef __GIMBAL_LINK_H__
#define __GIMBAL_LINK_H__

#include "main.h"

#define GP_MODE_STAB 0u
#define GP_MODE_AIM  1u

/* 上位机下发的命令，yuntai_task 只读。 */
typedef struct
{
    uint8_t mode;              /* GP_MODE_xxx */
    uint8_t aim_valid;         /* 上位机本帧视觉有效 */
    uint8_t vision_seq;        /* AIM 帧序号：K230 每产生一帧新视觉结果 +1 */
    uint16_t px_ut, px_vt;     /* 靶心像素坐标（K230 上报） */
    uint16_t px_us, px_vs;     /* 激光光斑像素坐标 */
    float   vis_yaw_deg;       /* H723 换算后的偏航角误差 */
    float   vis_pitch_deg;     /* H723 换算并按相机方向取反后的俯仰角误差 */
    float   target_yaw_rate_dps;   /* 靶心横向角速度（H723 统一换算） */
    float   target_pitch_rate_dps; /* 靶心纵向角速度（H723 统一换算） */
    uint8_t target_velocity_valid; /* 本帧是否携带目标角速度 */
    uint16_t vis_proc_age_ms;  /* K230从取图完成到发送该帧的处理年龄 */
    uint32_t px_ms;            /* 本帧到达时刻（视觉帧间隔/年龄） */
    uint32_t rx_count;         /* 收到多少帧 AIM（用来确认链路真的在动） */
} GimbalCmd_t;

typedef struct
{
    uint32_t rx_bytes;
    uint32_t lines;
    uint32_t rejected_lines;
} GimbalLinkStats_t;

void               gimbal_link_init(void);
void               gimbal_link_poll(uint32_t now_ms);
void               gimbal_link_rx_isr(void);
const GimbalCmd_t *gimbal_link_cmd(void);
GimbalLinkStats_t  gimbal_link_stats(void);
uint8_t            gimbal_link_take_zero_req(void);   /* 读并清"重新锁零"请求 */

#endif /* __GIMBAL_LINK_H__ */
