#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 *  电机 + 编码器驱动 (带速度闭环)
 * --------------------------------------------------------------------------
 *  硬件:
 *      PA6 = TIM3_CH1  左电机 IN1        PB0 = TIM3_CH3  右电机 IN2
 *      PA7 = TIM3_CH2  左电机 IN2        PB1 = TIM3_CH4  右电机 IN1
 *      PB6/PB7 = TIM4 编码器(左)         PC6/PC7 = TIM8 编码器(右)
 *
 *  接口单位是"最大速度的百分比"(-100 ~ +100):
 *      Motor_SetTargetPercent() 给出期望轮速, Motor_ControlUpdate() 每 5ms
 *      用编码器测速做 PI 修正, 最后输出 PWM。
 *      这样外层的巡线/转弯只需要关心"要多快", 不用管电池电压和死区。
 * ========================================================================== */

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} MotorID;

/* --------------------------- 硬件参数 --------------------------- */
#define MOTOR_PWM_PERIOD        999.0f    /* TIM3 的 ARR, 72MHz/1000 = 72kHz PWM */
#define MOTOR_CTRL_DT           0.005f    /* 控制周期(s), 与 TIM2 一致 */

/* 轮子转一圈的编码器计数: 13线 * 30减速比 * 4倍频 = 1560。 */
#define MOTOR_ENCODER_CPR       1560.0f
/* 100% 时对应的轮速(rpm)。用来把百分比换算成 rpm, 只影响速度刻度 */
#define MOTOR_RPM_AT_FULL       342.0f

/* 实测轮子直径 6.7cm -> 半径 3.35cm。
 * 如果现场确认 6.7cm 是半径而不是直径, 需要把直径改成 13.4cm。 */
#define MOTOR_WHEEL_DIAMETER_CM 6.7f
#define MOTOR_WHEEL_RADIUS_CM   (MOTOR_WHEEL_DIAMETER_CM * 0.5f)
/* 每厘米多少编码器计数 = 每圈计数 / 轮子周长 */
#define MOTOR_CNT_PER_CM        (MOTOR_ENCODER_CPR / (6.2831853f * MOTOR_WHEEL_RADIUS_CM))

/* 编码器方向: 实测"给正转命令但速度为负"时把对应符号改成 -1 */
#define MOTOR_LEFT_ENC_SIGN     (+1.0f)
#define MOTOR_RIGHT_ENC_SIGN    (-1.0f)

/*
 * 电机启动死区(占空比百分比): 用于标定和调试, 不再作为速度环硬输出下限。
 *
 * **这个值必须实测**, 而且它会随电池电压变化(电压低 -> 死区大)。
 * 测法有两种:
 *   1) 把 main.c 里的 MOTOR_DEADZONE_SWEEP 改成 1, 上电扫一遍(见那边注释);
 *   2) 把 MOTOR_SPEED_LOOP_ENABLE 改成 0, 慢慢加目标值, 记下轮子刚转起来的数。
 * 宁可填大一点: 填大了速度环能把输出压回来, 填小了两轮会卡在死区里不动。
 * 下面还有"卡住自动补偿", 填不准也能自己找补回来, 但填准了起步最干净。
 */
/*
 * 实测值(架空扫描): 56% 时几乎不动, 58% 开始转 -> 死区约 55~56%。
 * 实测轮速 = (占空比 - 55) * 7.55 rpm/%, 100% 时约 342rpm(和 RPM_AT_FULL 吻合)。
 *
 * 必须填准! 速度环用非零积分补偿实际死区, 这个实测值用于判断电池/负载变化。
 */
#define MOTOR_DEADZONE_PERCENT  58.0f

/*
 * 反馈失效保护: 积分已经到上限、轮子还是完全不动, 持续这么久(400*5ms=2秒)
 * 就认为"电机没电 / 编码器坏了", 该轮停止输出并置起卡住标志。
 * 目标速度回到 0 时自动清除。
 */
#define MOTOR_STALL_TICKS       400u

/* --------------------------- 速度环参数 --------------------------- */
/* 1 = 使用编码器速度闭环, 0 = 不输出(仅供闭环调试) */
#define MOTOR_SPEED_LOOP_ENABLE 1

#define MOTOR_SPEED_KP          6.00f
#define MOTOR_SPEED_KI          1.60f
#define MOTOR_SPEED_KD          0.06f
#define MOTOR_SPEED_MAX_INT     100.0f    /* 积分必须能单独越过启动死区 */
#define MOTOR_SPEED_FILTER      0.45f     /* 测速低通系数(0~1, 越小越平滑) */
#define MOTOR_SPEED_D_FILTER    0.15f     /* 速度微分低通 */

/* 目标进入超低速区时清掉直线段积累的积分, 防止内轮继续被旧占空比托着。 */
#define MOTOR_INTEGRAL_RESET_BELOW_PCT 5.0f

/* 参考工程速度环使用前馈+PID; 本车用有负载死区附近的55%作为前馈基准。 */
#define MOTOR_FF_BASE_PERCENT       55.0f
#define MOTOR_FF_SLOPE_PER_PCT      0.45f
#define MOTOR_FF_FADE_PCT           5.0f

/* 实际速度高于目标但目标仍为正时, 允许短接制动, 不允许反向驱动。 */
#define MOTOR_BRAKE_MAX_PERCENT     100.0f
#define MOTOR_BRAKE_MIN_PERCENT     5.0f
#define MOTOR_BRAKE_GAIN            2.0f

/* 目标速度的最大变化率(每个控制周期允许改变的百分比), 让起步/换向更顺 */
#define MOTOR_TARGET_SLEW       2.5f

/*
 * 输出上升限速(%/周期): 只限占空比"变大"的速度, 下降不限(转向、刹车要快)。
 * 因为死区占到 58%, 轮子要动就得一步跨到 58% 以上, 起步本来是"跳"上去的;
 * 加了这条之后占空比是 300%/秒 爬上去, 0 -> 66% 大约 220ms,
 * 齿轮间隙和静摩擦会被柔和地顶开, 起步就不那么蹿了。
 */
#define MOTOR_OUT_UP_SLEW       1.5f

/* ------------------------- 电机使能开关 (SW4) ------------------------- */
/*
 * 开关一端把 PC13 经 R15 10k 拉到 3V3, 另一端把 PC13 直接拉到 GND。
 * 默认按 "拨到 GND 一侧(PC13 = 低) 即使能" 处理:
 *   0 = 低电平使能(默认)
 *   1 = 高电平使能(实测相反时改成 1)
 * 上电时把开关拨到两个位置各看一次遥测里的 EN 值即可确定。
 */
#define MOTOR_ENABLE_ACTIVE_HIGH    0
/* 开关抖动滤波: 连续多少个控制周期读数一致才认(4*5ms = 20ms) */
#define MOTOR_ENABLE_DEBOUNCE_TICKS 4u

/* ------------------------------ 接口 ------------------------------ */

void  Motor_Init(void);

/* 设置期望轮速(最大速度的百分比, -100 ~ +100), 内部会做斜率限制 */
void  Motor_SetTargetPercent(MotorID motor, float percent);

/* 5ms 周期调用(放在 TIM2 中断里): 测速 + PI + 输出 PWM */
void  Motor_ControlUpdate(void);

/* 立即停车(滑行), 同时清掉速度环积分 */
void  Motor_Stop(void);

/* 立即刹车(两个半桥同时导通), 用于急停 */
void  Motor_Brake(void);

/* 调试用: 直接给占空比(%), 绕过速度环。上电死区扫描用它 */
void  Motor_SetDutyRaw(MotorID motor, float duty_percent);

/*
 * 让某个轮子保持一个固定占空比(不经过速度环), 直到该轮被重新设置目标速度为止。
 * 用途: 转弯时给内侧轮加一个"弱反向制动", 让它产生向后的阻力 —— 只有这样才能
 * 把弧线半径压到 2k²/L 以下。传 0 就等于松开该轮。
 */
void  Motor_SetDutyHold(MotorID motor, float duty_percent);

/*
 * 保持刹车: 两个半桥同时导通, 电机短接, 轮子立刻停住。
 * 之后只要给该轮一个非零目标速度(或调用 Motor_Stop)就自动解除。
 * 用途: 原地转刚好到 90° 时刹住, 不再靠滑行, 避免多转 1~2°。
 */
void  Motor_SetBrakeHold(MotorID motor, uint8_t on);

/* 电机使能开关状态: 1 = 允许转动, 0 = 禁止(禁止时输出恒为 0) */
uint8_t Motor_IsEnabled(void);

/* 使能开关引脚上的原始电平(0/1), 用来判断开关拨到哪边是哪一边 */
uint8_t Motor_GetEnablePinLevel(void);

/* 卡住标志: bit0 = 左轮, bit1 = 右轮 (1 = 反馈失效已停机) */
uint8_t Motor_GetStallFlag(void);

/*
 * 累计里程(编码器计数, 前进为正)。
 * 对"走够多少厘米"这种判断很有用: 厘米数 = 计数 / (每圈计数 / 轮子周长)。
 */
int32_t Motor_GetEncoderTotal(MotorID motor);

/* 累计里程(cm, 前进为正) */
float Motor_GetDistanceCm(MotorID motor);

float Motor_GetMeasuredPercent(MotorID motor);
float Motor_GetOutputPercent(MotorID motor);
float Motor_GetTargetPercent(MotorID motor);
float Motor_GetRampedTargetPercent(MotorID motor);
float Motor_GetIntegralPercent(MotorID motor);
float Motor_GetBrakePercent(MotorID motor);
int16_t Motor_GetDeltaCounts(MotorID motor);

#endif /* __MOTOR_H__ */
