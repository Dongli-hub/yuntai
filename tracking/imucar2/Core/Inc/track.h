#ifndef __TRACK_H__
#define __TRACK_H__

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 *  循迹控制 (参考 25-aiming-cart, TURN到90°后直接回LINE)
 * --------------------------------------------------------------------------
 *  位置层统一产生车体 v(m/s) 和 w(rad/s), 再换算左右轮目标速度:
 *      left  = v - wheel_base/2 * w
 *      right = v + wheel_base/2 * w
 * ========================================================================== */

typedef enum {
    TRACK_LINE = 0,       /* 正常巡线和普通弯道 */
    TRACK_PRE_TURN,       /* 轮轴继续走到起弧点 */
    TRACK_TURN,           /* IMU角度/角速度转弯 */
    TRACK_LOST,           /* 丢线, 低速找线 */
    TRACK_STOP            /* 停车 */
} TrackState;

void Track_Init(void);
void Track_Update(void);        /* 5ms 周期调用(放在 TIM2 中断里) */

/* 调试 / 遥测 */
TrackState Track_GetState(void);
float      Track_GetError(void);        /* 横向误差, mm */
float      Track_GetTurnTargetDeg(void);/* 本次转弯的目标角(带符号) */
int8_t     Track_GetTurnDir(void);      /* +1 = 左转, -1 = 右转, 0 = 无 */
float      Track_GetBodyTargetMps(void);
float      Track_GetBodyTargetRadps(void);
float      Track_GetBodyMeasuredMps(void);
float      Track_GetTurnAngleDeg(void);
float      Track_GetHeadingDeg(void);
float      Track_GetLeftTargetMps(void);
float      Track_GetRightTargetMps(void);

/*
 * 打印最近一次转弯的实测数据(弧长 / 转过的角度 / 耗时)。
 * 主循环周期调用即可 —— 这样测试时不必一直盯着串口, 跑完接上串口再看。
 */
void       Track_PrintLastTurn(void);

#endif /* __TRACK_H__ */
