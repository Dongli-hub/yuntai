#include "motor.h"
#include "tim.h"
#include <math.h>

/* ============================ 内部参数 ============================ */
/* 左轮: CH1 = 正转(IN1), CH2 = 反转(IN2) */
#define LEFT_FWD_CH             TIM_CHANNEL_1
#define LEFT_BWD_CH             TIM_CHANNEL_2
/* 右轮: CH4 = 正转(IN1), CH3 = 反转(IN2) */
#define RIGHT_FWD_CH            TIM_CHANNEL_4
#define RIGHT_BWD_CH            TIM_CHANNEL_3

/* ============================ 内部状态 ============================ */
static uint16_t enc_prev[2];        /* 上一次的编码器计数 */
static int32_t  enc_total[2];       /* 累计计数(前进为正) */
static int16_t  enc_delta[2];       /* 最近一个5ms周期的编码器增量 */
static float    measured[2];        /* 测得的轮速(%) */
static float    output[2];          /* 实际输出的 PWM(%) */
static float    target[2];          /* 上层给的期望(%) */
static float    ramped[2];          /* 斜率限制之后的期望(%) */
static float    integral[2];        /* 速度环积分项 */
static float    speed_d_filtered[2];/* 实测速度的微分(低通后) */
static float    brake_pwm[2];       /* 方向保持短接制动强度(%) */
static uint16_t stall_cnt[2];        /* "抬到上限还是不动"的持续周期数 */
static uint8_t  stall_fault;         /* bit0/bit1 = 左右轮卡住 */
static uint8_t  hold_on[2];          /* 该轮是否被"固定占空比"接管 */
static float    hold_duty[2];        /* 固定占空比(%) */
static uint8_t  brake_hold[2];       /* 该轮是否保持"短接刹车" */

/* 电机使能开关 */
static uint8_t  enable_state;       /* 去抖之后: 1 = 使能 */
static uint8_t  enable_raw;         /* 上一次的原始读数(去抖之前) */
static uint8_t  enable_cnt;         /* 连续一致的次数 */

/* ============================ 工具函数 ============================ */

static float motor_clampf(float v, float lo, float hi)
{
    if (v > hi) {
        return hi;
    }
    if (v < lo) {
        return lo;
    }
    return v;
}

/*
 * 参考工程的前馈结构: 固定死区偏移 + 目标速度比例项。
 * 低目标时死区偏移按比例淡出, 避免弯心内轮被55%的基准占空比托快。
 */
static float motor_feedforward(float percent)
{
    float a = fabsf(percent);
    float offset = MOTOR_FF_BASE_PERCENT;
    float out;

    if (a < 0.0001f) {
        return 0.0f;
    }
    if (a < MOTOR_FF_FADE_PCT) {
        offset *= a / MOTOR_FF_FADE_PCT;
    }
    out = offset + a * MOTOR_FF_SLOPE_PER_PCT;
    return (percent > 0.0f) ? out : -out;
}

/* 把百分比输出变成 TIM3 的两路比较值 */
static void motor_apply_pwm(MotorID m, float percent)
{
    uint32_t fwd_ch = (m == MOTOR_LEFT) ? LEFT_FWD_CH : RIGHT_FWD_CH;
    uint32_t bwd_ch = (m == MOTOR_LEFT) ? LEFT_BWD_CH : RIGHT_BWD_CH;
    uint32_t duty;
    float a = fabsf(percent);

    if (a > 100.0f) {
        a = 100.0f;
    }
    duty = (uint32_t)((a * MOTOR_PWM_PERIOD) / 100.0f);
    if (duty > (uint32_t)MOTOR_PWM_PERIOD) {
        duty = (uint32_t)MOTOR_PWM_PERIOD;
    }

    /* 上升限速: 占空比只能一点点往上爬, 起步才柔和(下降不限速) */
    if (fabsf(percent) > fabsf(output[m])) {
        float lim = fabsf(output[m]) + MOTOR_OUT_UP_SLEW;
        if (fabsf(percent) > lim) {
            percent = (percent > 0.0f) ? lim : -lim;
            a = fabsf(percent);
            duty = (uint32_t)((a * MOTOR_PWM_PERIOD) / 100.0f);
            if (duty > (uint32_t)MOTOR_PWM_PERIOD) {
                duty = (uint32_t)MOTOR_PWM_PERIOD;
            }
        }
    }

    if (percent > 0.0f) {
        __HAL_TIM_SET_COMPARE(&htim3, fwd_ch, duty);
        __HAL_TIM_SET_COMPARE(&htim3, bwd_ch, 0u);
    } else if (percent < 0.0f) {
        __HAL_TIM_SET_COMPARE(&htim3, fwd_ch, 0u);
        __HAL_TIM_SET_COMPARE(&htim3, bwd_ch, duty);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, fwd_ch, 0u);
        __HAL_TIM_SET_COMPARE(&htim3, bwd_ch, 0u);
    }
    output[m] = percent;
    brake_pwm[m] = 0.0f;
}

/* 两个半桥同时PWM导通, 产生短接制动但不能反向驱动。 */
static void motor_apply_brake_pwm(MotorID m, float percent)
{
    uint32_t fwd_ch = (m == MOTOR_LEFT) ? LEFT_FWD_CH : RIGHT_FWD_CH;
    uint32_t bwd_ch = (m == MOTOR_LEFT) ? LEFT_BWD_CH : RIGHT_BWD_CH;
    uint32_t duty;

    percent = motor_clampf(percent, 0.0f, MOTOR_BRAKE_MAX_PERCENT);
    duty = (uint32_t)((percent * MOTOR_PWM_PERIOD) / 100.0f);
    __HAL_TIM_SET_COMPARE(&htim3, fwd_ch, duty);
    __HAL_TIM_SET_COMPARE(&htim3, bwd_ch, duty);
    output[m] = 0.0f;
    brake_pwm[m] = percent;
}

/*
 * 读编码器算轮速。
 * 计数器是 16 位, 用 int16_t 强转做差可以自动处理正反两个方向的回卷。
 */
static float motor_read_speed(MotorID m)
{
    TIM_TypeDef *tim = (m == MOTOR_LEFT) ? TIM4 : TIM8;
    float sign = (m == MOTOR_LEFT) ? MOTOR_LEFT_ENC_SIGN : MOTOR_RIGHT_ENC_SIGN;
    uint16_t now = (uint16_t)(tim->CNT);
    int16_t  delta = (int16_t)((uint16_t)(now - enc_prev[m]));
    float    rpm;
    float    percent;

    enc_prev[m] = now;
    enc_delta[m] = delta;

    /* rpm = 每周期脉冲数 * (60 / (每圈脉冲数 * 周期)) */
    rpm = (float)delta * (60.0f / (MOTOR_ENCODER_CPR * MOTOR_CTRL_DT));
    rpm *= sign;

    /* 累计里程: 按同一个方向符号累加, 前进为正 */
    enc_total[m] += (sign >= 0.0f) ? (int32_t)delta : -(int32_t)delta;

    percent = rpm * 100.0f / MOTOR_RPM_AT_FULL;

    /* 低通, 抑制量化噪声; 再对滤波后的轮速做微分供 D 项阻尼。 */
    {
        float previous = measured[m];
        measured[m] += (percent - measured[m]) * MOTOR_SPEED_FILTER;
        speed_d_filtered[m] +=
            (((measured[m] - previous) / MOTOR_CTRL_DT) -
             speed_d_filtered[m]) * MOTOR_SPEED_D_FILTER;
    }
    return measured[m];
}

/* 读使能开关引脚原始电平: 0 或 1 */
static uint8_t motor_enable_level(void)
{
    return (HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

/* 把原始电平翻译成"是否使能" */
static uint8_t motor_enable_judge(uint8_t level)
{
#if MOTOR_ENABLE_ACTIVE_HIGH
    return (level != 0u) ? 1u : 0u;
#else
    return (level == 0u) ? 1u : 0u;
#endif
}

/* 使能开关去抖, 每个控制周期调用一次 */
static void motor_enable_update(void)
{
    uint8_t raw = motor_enable_judge(motor_enable_level());

    if (raw == enable_raw) {
        if (enable_cnt < 255u) {
            enable_cnt++;
        }
    } else {
        enable_raw = raw;
        enable_cnt = 0u;
    }

    if (enable_cnt >= MOTOR_ENABLE_DEBOUNCE_TICKS) {
        enable_state = raw;
    }
}

/* ============================ 对外接口 ============================ */

void Motor_Init(void)
{
    MotorID m;

    /* 先确定上电瞬间的开关状态, 不要等去抖 */
    enable_raw = motor_enable_judge(motor_enable_level());
    enable_state = enable_raw;
    enable_cnt = MOTOR_ENABLE_DEBOUNCE_TICKS;

    HAL_TIM_PWM_Start(&htim3, LEFT_FWD_CH);
    HAL_TIM_PWM_Start(&htim3, LEFT_BWD_CH);
    HAL_TIM_PWM_Start(&htim3, RIGHT_FWD_CH);
    HAL_TIM_PWM_Start(&htim3, RIGHT_BWD_CH);

    __HAL_TIM_SET_COMPARE(&htim3, LEFT_FWD_CH, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, LEFT_BWD_CH, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, RIGHT_FWD_CH, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, RIGHT_BWD_CH, 0u);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    for (m = MOTOR_LEFT; m <= MOTOR_RIGHT; m = (MotorID)(m + 1)) {
        enc_prev[m] = (uint16_t)(((m == MOTOR_LEFT) ? TIM4 : TIM8)->CNT);
        enc_total[m] = 0;
        enc_delta[m] = 0;
        hold_on[m] = 0u;
        hold_duty[m] = 0.0f;
        brake_hold[m] = 0u;
        measured[m] = 0.0f;
        output[m] = 0.0f;
        target[m] = 0.0f;
        ramped[m] = 0.0f;
        integral[m] = 0.0f;
        speed_d_filtered[m] = 0.0f;
        brake_pwm[m] = 0.0f;
        stall_cnt[m] = 0u;
    }
    stall_fault = 0u;
}

void Motor_SetTargetPercent(MotorID motor, float percent)
{
    float old_abs;
    float new_abs;

    if (motor > MOTOR_RIGHT) {
        return;
    }
    /* 一旦给回正常的"目标速度", 就解除该轮的固定占空比(制动) */
    if (fabsf(percent) > 0.0001f) {
        hold_on[motor] = 0u;
        brake_hold[motor] = 0u;
    }
    old_abs = fabsf(target[motor]);
    target[motor] = motor_clampf(percent, -100.0f, 100.0f);
    new_abs = fabsf(target[motor]);

    /*
     * 从正常速度突然进入超低速区时, 清掉直线段学到的占空比积分。
     * 否则内轮会继续被旧的约55%占空比积分托着, 速度压不下来。
     */
    if ((new_abs < MOTOR_INTEGRAL_RESET_BELOW_PCT) &&
        (old_abs >= MOTOR_INTEGRAL_RESET_BELOW_PCT)) {
        integral[motor] = 0.0f;
        speed_d_filtered[motor] = 0.0f;
    }
}

void Motor_SetDutyHold(MotorID motor, float duty_percent)
{
    if (motor > MOTOR_RIGHT) {
        return;
    }
    brake_hold[motor] = 0u;
    hold_duty[motor] = motor_clampf(duty_percent, -100.0f, 100.0f);
    hold_on[motor] = 1u;
}

/*
 * 保持刹车: 两个半桥同时导通(电机两端短接), 轮子被强行停住。
 * 用途: 原地转刚好到 90° 时立刻刹住, 而不是滑行 —— 否则车还会多转 1~2°。
 * 之后只要给这个轮子一个非零目标速度(或 Motor_Stop), 刹车自动解除。
 */
void Motor_SetBrakeHold(MotorID motor, uint8_t on)
{
    if (motor > MOTOR_RIGHT) {
        return;
    }
    brake_hold[motor] = (on != 0u) ? 1u : 0u;
    if (on != 0u) {
        target[motor] = 0.0f;
        ramped[motor] = 0.0f;
        integral[motor] = 0.0f;
        speed_d_filtered[motor] = 0.0f;
        hold_on[motor] = 0u;
    }
}

void Motor_ControlUpdate(void)
{
    MotorID m;

    /* 0) 使能开关: 关掉时立刻切断输出, 并且不累积积分 */
    motor_enable_update();
    if (enable_state == 0u) {
        for (m = MOTOR_LEFT; m <= MOTOR_RIGHT; m = (MotorID)(m + 1)) {
            ramped[m] = 0.0f;
            integral[m] = 0.0f;
            speed_d_filtered[m] = 0.0f;
            motor_apply_pwm(m, 0.0f);
        }
        return;
    }

    /*
     * 0.5) 任一车轮被判定"推不动" -> 整车停机, 免得单边驱动把车带偏。
     *      等目标速度都回到 0(状态机进了停车) 自动解除, 允许重新再试一次。
     */
    if ((stall_fault != 0u) &&
        (fabsf(target[MOTOR_LEFT]) < 0.05f) && (fabsf(target[MOTOR_RIGHT]) < 0.05f)) {
        stall_fault = 0u;
        stall_cnt[0] = 0u;
        stall_cnt[1] = 0u;
    }
    if (stall_fault != 0u) {
        for (m = MOTOR_LEFT; m <= MOTOR_RIGHT; m = (MotorID)(m + 1)) {
            ramped[m] = 0.0f;
            integral[m] = 0.0f;
            speed_d_filtered[m] = 0.0f;
            motor_apply_pwm(m, 0.0f);
        }
        return;
    }

    for (m = MOTOR_LEFT; m <= MOTOR_RIGHT; m = (MotorID)(m + 1)) {
        float tgt;
        float ff;
        float err;
        float out;
        float brake_cmd = 0.0f;

        /*
         * 先测速 —— 即使这个轮子被"固定占空比"接管(比如转弯时给内侧轮制动),
         * 也要读编码器, 否则里程和 [LT] 的弧长会缺一段。
         */
        (void)motor_read_speed(m);

        /* 保持刹车: 两个半桥同时导通, 电机短接 -> 轮子立刻停 */
        if (brake_hold[m] != 0u) {
            uint32_t fwd_ch = (m == MOTOR_LEFT) ? LEFT_FWD_CH : RIGHT_FWD_CH;
            uint32_t bwd_ch = (m == MOTOR_LEFT) ? LEFT_BWD_CH : RIGHT_BWD_CH;

            target[m] = 0.0f;
            ramped[m] = 0.0f;
            integral[m] = 0.0f;
            speed_d_filtered[m] = 0.0f;
            __HAL_TIM_SET_COMPARE(&htim3, fwd_ch, (uint32_t)MOTOR_PWM_PERIOD);
            __HAL_TIM_SET_COMPARE(&htim3, bwd_ch, (uint32_t)MOTOR_PWM_PERIOD);
            output[m] = 0.0f;
            continue;
        }

        /*
         * 被"固定占空比"接管的轮子, 直接输出, 不跑速度环。
         * 注意: 默认 TURN_INNER_BRAKE_DUTY = 0, 也就是内侧轮松开(占空比 0)。
         */
        if (hold_on[m] != 0u) {
            target[m] = 0.0f;
            ramped[m] = 0.0f;
            integral[m] = 0.0f;
            speed_d_filtered[m] = 0.0f;
            motor_apply_pwm(m, hold_duty[m]);
            continue;
        }

        /* 1) 目标斜率限制: 让启停、换向都平滑 */
        tgt = ramped[m];
        if (target[m] > (tgt + MOTOR_TARGET_SLEW)) {
            tgt += MOTOR_TARGET_SLEW;
        } else if (target[m] < (tgt - MOTOR_TARGET_SLEW)) {
            tgt -= MOTOR_TARGET_SLEW;
        } else {
            tgt = target[m];
        }
        ramped[m] = tgt;

        if (fabsf(tgt) < 0.0001f) {
            /* 目标为 0: 清掉积分, 直接停 */
            integral[m] = 0.0f;
            speed_d_filtered[m] = 0.0f;
            stall_cnt[m] = 0u;
            motor_apply_pwm(m, 0.0f);
            continue;
        }

        /* 速度环: 参考工程结构, 前馈 + 编码器PID。 */
        ff = motor_feedforward(tgt);
#if MOTOR_SPEED_LOOP_ENABLE
        err = tgt - measured[m];
        {
            /*
             * 两个轮子共用同一套 PID:
             *   P 提供基本跟随, I 补偿死区和稳态误差,
             *   D 对实测轮速变化提供阻尼, 抑制速度突变/超调。
             *   速度高于目标时允许把占空比降到 0 松油门/制动;
             *   但正目标绝不输出反向 PWM, 负目标也绝不输出正向 PWM。
             */
            float ni = integral[m] + err * MOTOR_SPEED_KI * MOTOR_CTRL_DT;
            ni = motor_clampf(ni, -MOTOR_SPEED_MAX_INT, MOTOR_SPEED_MAX_INT);
            out = ff + MOTOR_SPEED_KP * err + ni
                - MOTOR_SPEED_KD * speed_d_filtered[m];
            if (tgt > 0.0f) {
                if (out > 100.0f) {
                    out = 100.0f;
                    ni = integral[m];
                } else if (out < 0.0f) {
                    /* 目标为正时不能反向驱动; 用短接制动快速降低实际速度。 */
                    brake_cmd = -out;
                    out = 0.0f;
                    ni = integral[m];
                }
            } else {
                if (out < -100.0f) {
                    out = -100.0f;
                    ni = integral[m];
                } else if (out > 0.0f) {
                    brake_cmd = out;
                    out = 0.0f;
                    ni = integral[m];
                }
            }
            integral[m] = ni;
        }
#else
        out = ff;
#endif

        /* 积分已经到上限且仍有正向目标时, 持续两秒不动才判反馈失效。 */
        if ((fabsf(measured[m]) < 2.0f) && (fabsf(tgt) > 5.0f) &&
            (fabsf(integral[m]) >= (MOTOR_SPEED_MAX_INT - 0.1f))) {
            if (stall_cnt[m] < MOTOR_STALL_TICKS) {
                stall_cnt[m]++;
            }
            if (stall_cnt[m] >= MOTOR_STALL_TICKS) {
                stall_fault |= (uint8_t)(1u << m);
            }
        } else {
            stall_cnt[m] = 0u;
        }

        if (brake_cmd >= MOTOR_BRAKE_MIN_PERCENT) {
            motor_apply_brake_pwm(m, brake_cmd * MOTOR_BRAKE_GAIN);
        } else {
            motor_apply_pwm(m, motor_clampf(out, -100.0f, 100.0f));
        }
    }
}

void Motor_SetDutyRaw(MotorID motor, float duty_percent)
{
    if (motor > MOTOR_RIGHT) {
        return;
    }
    brake_hold[motor] = 0u;
    target[motor] = 0.0f;
    ramped[motor] = 0.0f;
    integral[motor] = 0.0f;
    speed_d_filtered[motor] = 0.0f;
    brake_pwm[motor] = 0.0f;
    motor_apply_pwm(motor, motor_clampf(duty_percent, -100.0f, 100.0f));
}

void Motor_Stop(void)
{
    MotorID m;

    for (m = MOTOR_LEFT; m <= MOTOR_RIGHT; m = (MotorID)(m + 1)) {
        target[m] = 0.0f;
        ramped[m] = 0.0f;
        integral[m] = 0.0f;
        speed_d_filtered[m] = 0.0f;
        brake_pwm[m] = 0.0f;
        hold_on[m] = 0u;
        brake_hold[m] = 0u;
        motor_apply_pwm(m, 0.0f);
    }
}

void Motor_Brake(void)
{
    MotorID m;

    for (m = MOTOR_LEFT; m <= MOTOR_RIGHT; m = (MotorID)(m + 1)) {
        target[m] = 0.0f;
        ramped[m] = 0.0f;
        integral[m] = 0.0f;
        speed_d_filtered[m] = 0.0f;
        brake_pwm[m] = 0.0f;
        if (m == MOTOR_LEFT) {
            __HAL_TIM_SET_COMPARE(&htim3, LEFT_FWD_CH, (uint32_t)MOTOR_PWM_PERIOD);
            __HAL_TIM_SET_COMPARE(&htim3, LEFT_BWD_CH, (uint32_t)MOTOR_PWM_PERIOD);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, RIGHT_FWD_CH, (uint32_t)MOTOR_PWM_PERIOD);
            __HAL_TIM_SET_COMPARE(&htim3, RIGHT_BWD_CH, (uint32_t)MOTOR_PWM_PERIOD);
        }
        output[m] = 0.0f;
    }
}

float Motor_GetMeasuredPercent(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? measured[motor] : 0.0f;
}

uint8_t Motor_IsEnabled(void)
{
    return enable_state;
}

uint8_t Motor_GetEnablePinLevel(void)
{
    return motor_enable_level();
}

uint8_t Motor_GetStallFlag(void)
{
    return stall_fault;
}

int32_t Motor_GetEncoderTotal(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? enc_total[motor] : 0;
}

float Motor_GetDistanceCm(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? ((float)enc_total[motor] / MOTOR_CNT_PER_CM) : 0.0f;
}

float Motor_GetOutputPercent(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? output[motor] : 0.0f;
}

float Motor_GetTargetPercent(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? target[motor] : 0.0f;
}

float Motor_GetRampedTargetPercent(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? ramped[motor] : 0.0f;
}

float Motor_GetIntegralPercent(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? integral[motor] : 0.0f;
}

float Motor_GetBrakePercent(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? brake_pwm[motor] : 0.0f;
}

int16_t Motor_GetDeltaCounts(MotorID motor)
{
    return (motor <= MOTOR_RIGHT) ? enc_delta[motor] : 0;
}
