/**
  ******************************************************************************
  * @file    yuntai_task.c
  * @brief   双轴云台控制：稳定 + 接受机载计算机的瞄准偏置
  *
  * ============================================================================
  * 一句话讲清这套控制：
  *
  *   云台自己负责"稳"（快，200~500Hz，用陀螺/IMU 抗车体晃动），
  *   机载计算机负责"准"（慢，30~60Hz，看图像把激光逼到靶心上），
  *   两者靠"角度偏置"这一个低频小量连接。
  *
  * 所以本文件的核心改动只有一句：
  *
  *       偏航目标 = 上电锁定的绝对方位 + 上位机给的偏航偏置
  *       俯仰目标 = 上电锁定的电机角度 + 上位机给的俯仰偏置
  *
  * 稳定能力分毫未动 —— 偏置只是把"要稳定到的那个方向"挪一点。
  *
  * ============================================================================
  * 两个轴为什么用完全不同的控制方式（这是整个文件里最容易搞错的地方）：
  *
  *   IMU 装在【中间平台】上 —— 偏航电机之上、俯仰电机之下。
  *
  *   |            | 偏航电机转 | 俯仰电机转 |
  *   | IMU 读数    | 跟着变     | 完全不变   |
  *   | 电机编码器  | 跟着变     | 跟着变     |
  *
  *   判据：我让这个电机动一下，这个传感器会不会跟着变？
  *     · 偏航：IMU 看得见偏航电机的动作  -> 直接拿 IMU 角度做闭环反馈
  *     · 俯仰：IMU【看不见】俯仰电机的动作 -> 绝对不能拿 IMU 角度做反馈！
  *       硬这么做的话，控制器会认为"我推了它却毫无反应"，于是拼命加大输出，
  *       电机朝一个方向狂奔（这个现象历史上实测复现过）。
  *       -> 俯仰只能用【电机自己的编码器】做位置反馈。
  *
  *   IMU 在俯仰环里的角色是"扰动测量器"：它测到的俯仰角变化只可能来自
  *   车体倾斜，所以它负责回答"目标角应该是多少"，不回答"现在到哪儿了"。
  *
  * ============================================================================
  * 启动路径的铁律（踩过的坑，写在这里防止再犯）：
  *
  *   启动路径上只允许【纯延时】推进。
  *   凡是"要满足某条件才能往下走"的逻辑，一旦条件因噪声/环境永远不满足，
  *   整个状态机就停在原地，而表现是"云台一直是软的、能被手掰动" —— 极难定位。
  *   带条件的逻辑只能放在 RUNNING 里做后台降级。
  ******************************************************************************
  */

#include "yuntai_task.h"
#include "can_bsp.h"
#include "jc4310.h"
#include "fdcan.h"
#include "gimbal_link.h"
#include "gimbal_proto.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "user_lib.h"
#include "pid.h"
#include "debug_uart.h"
#include <math.h>
#include <stdio.h>

/* ==========================================================================
 * 一、可调参数
 * ========================================================================== */

/* ---- 启动时序（每一步都必须与"上电到能接收偏置"的时间预算对齐）----
 * 注意：机载计算机那侧有"2s 内打中靶心"的指标（赛题基本要求 2）。
 * 上电到 T1 结束的时间越短越好，但下面这几条是实测出来的底线，别乱砍：
 *   MOTOR_BOOT_DELAY_MS：驱动器上电自检需要时间，控制模式指令发太早会被忽略
 *                        （历史上"上电偶尔完全失去控制"就是这个原因）
 *   CLOSED_LOOP/SET_MODE：各留 0.5s 等驱动器把模式切换吃进去
 */
#define BMI_RETRY_MS             500u    /* BMI088 初始化失败时的重试间隔 */
#define MOTOR_BOOT_DELAY_MS     2000u    /* 等驱动器上电自检：纯延时，不要改小 */
#define CLOSED_LOOP_DELAY_MS     500u    /* 进闭环后等 0.5s */
#define SET_MODE_DELAY_MS        500u    /* 切速度模式后等 0.5s */
#define LOCK_DELAY_MS            500u    /* 锁基准前的观察时间，同时预热编码器 */

/* ---- 节拍 ---- */
#define IMU_PERIOD_MS              2u    /* 500Hz 读 IMU */
#define CONTROL_PERIOD_MS          5u    /* 200Hz 控制 */
#define DEBUG_PERIOD_MS          500u    /* 慢速调试文本周期 */
#define FB_WARN_PERIOD_MS       1000u    /* 反馈异常告警周期 */

/* ---- 偏航：IMU 角度环 ----
 * PID 输出单位是 rpm；误差单位是 rad；前馈单位是 rpm/(rad/s)。
 * 静摩擦脱困转速在 10rpm 量级，所以比例项在小误差时推不动电机，
 * 必须靠前馈（陀螺转速）先把电机"唤醒"，角度环只负责慢慢精修。
 */
#define YAW_KP                 200.0f   /* rpm/rad */
#define YAW_KI                  30.0f   /* rpm/(rad*s) */
#define YAW_KD                  10.0f   /* rpm/(rad/s) */
#define YAW_OUT_RPM             50.0f   /* 输出限幅；约 300°/s 的补偿能力 */
#define YAW_ILIM                 1.0f   /* 积分限幅：ki*ilim 要小于脱困转速，
                                         * 否则积分单独就能把输出顶到满 */
#define YAW_FF_RPM_PER_RADS     66.85f  /* 66.85 rpm per (rad/s) ≈ 7 倍理论抵消量，
                                         * 故意过量才能保证越过静摩擦 */
#define YAW_FF_SIGN            (-1.0f)

/* ---- 俯仰：电机编码器位置环 + 陀螺前馈 ----
 * ⚠⚠ 比例增益的稳定上限（这条公式很重要，之前算错过一次，代价是"手一碰
 *    俯仰就前后越抖越大"）：
 *
 *     每拍的环路增益 = 6 × KP × dt
 *        （6 是 rpm -> °/s 的换算：1 rpm = 6 °/s）
 *     稳定要求它 < 2，即  KP < 1/(3·dt)
 *
 *   代入 dt = 5ms：
 *        KP = 100 -> 每拍增益 3.0  -> 【发散】：1° 误差一拍推过 3°，
 *                    下一拍反向推 6°，再下一拍 12°……手一碰就越抖越大
 *        KP =  15 -> 每拍增益 0.45 -> 稳定，且能覆盖驱动器速度环
 *                    延迟到 30ms 的最坏情况
 *
 *   之前误写成"环路带宽 = KP/6"（把换算关系搞反了），得出"100 没问题"，
 *   实际是 6×KP=600 rad/s 的带宽，而采样只有 200Hz —— 差了 3 倍，
 *   必然振荡。**这个教训写在 docs/05 里。**
 *
 *   注意：KP 调小不等于"变慢"。真正的定位精度由积分项和
 *   外层视觉环（积分型）保证，内环只需要稳、能抗扰。
 *   仿真（驱动器速度环 5~30ms、往返延迟 1~2 拍）末段残差 <=0.063°，
 *   折算到 1.1m 处 <=1.2mm。
 *
 *   现场若听到电机持续"滋滋"响 -> KP 再往小调；
 *   若俯仰被扰动后回不来 -> KI 往大调（不是 KP）。
 */
#define PITCH_KP                15.0f   /* rpm/deg；上限见上面的公式 */
#define PITCH_KI                45.0f   /* rpm/(deg*s)；消静差、负责"推得动" */
#define PITCH_KD                 0.10f  /* 编码器测速做一点阻尼；大了会被量化噪声放大 */
#define PITCH_OUT_RPM          120.0f   /* 约 720°/s，够甩开了 */
#define PITCH_ILIM               0.40f  /* 积分限幅；ki*ilim = 18rpm，
                                         * 要略大于脱困转速(约10rpm)才能顶住静摩擦，
                                         * 但又不能大到积分自己就能把输出顶满 */
#define PITCH_DEADZONE_DEG       0.03f  /* 死区，抑制编码器量化噪声 */
#define PITCH_FF_RPM_PER_RADS   15.0f   /* ≈1.6 倍理论抵消量 */
#define PITCH_FF_SIGN          (-1.0f)
#define PITCH_TRAVEL_LIMIT_DEG  80.0f   /* 相对上电位置的行程软限位 */
#define PITCH_FB_TIMEOUT_MS     50u     /* 超过这么久没反馈就降级为纯前馈 */
#define PITCH_RUNAWAY_DEG       25.0f   /* 跟踪误差持续这么大 = 方向反了或卡死 */
#define PITCH_RUNAWAY_MS       400u     /* 持续这么久就锁故障并降级 */

/* ---- 平台俯仰补偿（"云台整体低头/抬头时，镜头要自己找回来"）----
 *
 * 为什么必须有这一条：俯仰位置环的反馈是【电机自己的编码器】，量的是
 * "电机相对底座"的角度。底座整体倾斜时这个角压根不变 -> 位置环误差为 0
 * -> 电机一动不动 -> 镜头跟着底座一起低头，激光就从靶心跑掉了。
 * 位置环不会自己发现这件事，必须由 IMU 告诉它"世界倾斜了多少"，
 * 再把俯仰目标反向挪同样的角度 —— 这就是本段的作用。
 *
 * 前馈（陀螺）和这里的分工：
 *   陀螺前馈：只对【正在倾斜】的那一瞬间起作用（快，但没有绝对基准）
 *   平台补偿：负责【倾斜之后的持续角度】（慢，但有绝对基准）
 * 两个都要有，缺一个都会"倾斜完就偏"。
 *
 * 符号：补偿方向 sign_c 与前馈符号 sign_ff 是【相反】的
 *   （sign_c = -sign_ff；老固件实测 FF=-1、COMP=+1 就是这个关系）
 *   前馈测法：前后快倾底座，工具算 corr(陀螺, 电机角速度)，应为【负】
 *   补偿测法：慢慢把底座倾斜 10°，靶面光斑应【基本不动】；
 *             若光斑动得比关闭时还大 -> 符号取反
 *
 * 现在默认【打开】（+1，恢复老固件的行为），因为之前漏掉它，
 * 表现就是"底座一低头，镜头跟着低，电机不管"。
 * 现场可以随时用 GP_MSG_PARAM(0x0A) 在线改：0 = 关，±1 = 开并定方向。
 */
#define PITCH_PLAT_COMP          1          /* 1 = 默认打开（老固件行为） */
#define PITCH_PLAT_SIGN         (1.0f)      /* 初始符号；在线可改，见上 */
#define PLAT_COMP_ACC_TAU_S      2.0f       /* 互补滤波：加速度计修正时间常数 */
#define PLAT_COMP_RATE_DPS      60.0f       /* 补偿目标变化率限幅（°/s） */

/* ---- 陀螺零偏标定（解决「上电后云台朝一个方向慢慢转」）----
 *
 * 为什么必须做：陀螺静止时读数不是精确的 0，而是一个固定小偏置 b。
 * Mahony 把陀螺积分成角度 -> 角度以恒定速率 b 漂走 -> 偏航 PID 看到
 * "角度在漂"就驱动电机去追 -> 平台真的以 b 的速度匀速转。
 * 实测表现就是"上电后云台慢慢朝一个方向转"。
 *
 * 在哪里标：直接塞进【本来就在等的 MOTOR_BOOT_DELAY_MS（2 秒）】里，
 * 不额外花一分钱时间。
 *
 * ⚠ 关键：全程【无条件累加、不判断、不重来】，时间到就用平均值。
 *   老代码是"必须连续采够 N 个、且每个都在静止阈值内，否则清零重来" ——
 *   一旦传感器零偏本身就超过阈值，就永远采不满，只能干等到超时；
 *   更糟的是会卡住整个启动流程（表现是云台一直是软的）。那个坑不能再踩。
 *
 * 代价：上电后这 2 秒内如果晃了云台，标定值会偏（用限幅兜住）。
 *      所以规矩是【上电后 2 秒内别碰云台】。
 */
#define GYRO_BIAS_MAX           0.10f   /* rad/s ≈5.7°/s；超过就限幅，防止标定跑飞 */
#define GYRO_BIAS_MIN_SAMPLES   100u    /* 采样数少于这个就当标定失败，用 0 */

/* ---- 解绕（跑完一圈后把相对偏航角转回 0）----
 * 小车跑一圈航向转 360°，稳定环要保持绝对方位不变 ->
 * 偏航电机相对车身必然净转 -360°/圈，不处理就会把线拧断。
 * 见 docs/03 风险 R2。
 */
#define UNWIND_RPM_MAX           60.0f  /* 解绕最高转速上限，约 360°/s */
#define UNWIND_RPM_MIN           15.0f  /* 下限：低于脱困转速就推不动，解绕会"卡住不动" */
#define UNWIND_KP                  8.0f /* rpm/deg，相对角闭环 */
#define UNWIND_TOL_DEG             3.0f /* 相对角进这个范围就算解绕完成 */

/* ---- 激光输出（可选）----
 *
 * 当前接法：激光直接接电源，【上电就常亮】-> LASER_ALWAYS_ON = 1。
 *   此时固件不去控制激光（也控制不了），但会【如实】把"激光是亮的"
 *   报给上位机（遥测 flags 的 LASER_ON 位恒为 1），
 *   也不再打印会误导人的 [LASER] ON/OFF 日志。
 *
 * 以后接了 GPIO 控制线（PC4 是空闲脚，接激光模块的 MOS/驱动输入）：
 *   把 LASER_GPIO_ENABLE 改成 1、LASER_ALWAYS_ON 改成 0 即可。
 *
 * 为什么以后要接：发挥部分(3) 画圆要求"起跑前关激光"，
 * 否则靶心会留下一个孤立深斑 + 一条径向拖痕，D2 直接超差。
 */
#define LASER_GPIO_ENABLE          0
#define LASER_ALWAYS_ON            1
#define LASER_GPIO_PORT         GPIOC
#define LASER_PIN               GPIO_PIN_4

/* ==========================================================================
 * 二、内部状态
 * ========================================================================== */

typedef enum
{
    YUNTAI_STATE_BMI088_INIT = 0,   /* 1. 初始化 IMU */
    YUNTAI_STATE_MOTOR_BOOT,        /* 2. 等驱动器上电自检（纯延时） */
    YUNTAI_STATE_CLOSED_LOOP,       /* 3. 进入闭环（只发一次） */
    YUNTAI_STATE_SET_MODE,          /* 4. 切速度模式（只发一次） */
    YUNTAI_STATE_LOCK,              /* 5. 锁基准 + 预热编码器反馈 */
    YUNTAI_STATE_RUNNING,           /* 6. 稳定 + 瞄准 */
} YuntaiState;

/* 故障码（上报给上位机，方便定位） */
#define FAULT_NONE              0x00u
#define FAULT_PITCH_FB_LOST     0x01u
#define FAULT_PITCH_RUNAWAY     0x02u
#define FAULT_YAW_FB_LOST       0x04u

/* 编码器反馈（含解卷绕，兼容"单圈 0~360"和"多圈连续"两种读数） */
typedef struct
{
    float    raw;        /* 最近一次原始读数（deg） */
    float    prev;       /* 上一帧原始读数 */
    float    cont;       /* 解卷绕后的连续角（deg） */
    float    zero;       /* 锁零时的连续角 */
    uint32_t t_ms;       /* 最近一次更新时间 */
    uint8_t  seen;       /* 是否收到过至少一帧 */
} EncFb_t;

static float gyro[3];               /* 陀螺原始读数（rad/s） */
static float gyro_c[3];             /* 扣掉零偏后的陀螺读数 —— 全工程都用这个 */
static float acc[3];
static float temp;
static float imuQuat[4];
static float imuAngle[3];           /* [0]=yaw [1]=pitch [2]=roll，单位 rad */

static YuntaiState g_state = YUNTAI_STATE_BMI088_INIT;
static uint32_t    g_state_ms;      /* 当前状态的进入时刻 */
static uint32_t    g_last_imu_ms;
static uint32_t    g_last_ctrl_ms;
static uint32_t    g_last_dbg_ms;
static uint32_t    g_last_fbwarn_ms;
static uint32_t    g_last_read_ms;
static uint32_t    g_last_stat_ms;
static float       g_gyro_bias[3];   /* 陀螺零偏（上电时在 MOTOR_BOOT 那 2s 里标定） */
static float       g_gyro_sum[3];
static uint32_t    g_gyro_n;
static uint8_t     g_cmd_sent;      /* 状态机里"只发一次"的指令是否已发 */
static uint8_t     g_imu_ok;
static uint32_t    g_bmi_retry;

static PID_Controller g_yaw_pid;
static PID_Controller g_pitch_pid;
static PID_Controller g_yaw_unwind_pid;

static EncFb_t    g_yaw_fb;         /* 偏航电机编码器（FDCAN1） */
static EncFb_t    g_pitch_fb;       /* 俯仰电机编码器（FDCAN2） */

static float      g_yaw_lock_rad;   /* 上电锁定的绝对方位（rad） */
static float      g_pitch_plat_lock;/* 上电锁定的平台俯仰（rad） */
static float      g_plat_dtheta;    /* 平台俯仰相对基准的变化量（rad），互补滤波估计 */
static float      g_plat_comp;      /* 当前生效的平台俯仰补偿量（deg，含变化率限幅） */
static float      g_pitch_motor_lock;/* 上电锁定的俯仰电机角（deg） */
static float      g_pitch_tgt_deg;  /* 当前俯仰目标（deg），仅用于调试打印 */

static uint8_t    g_fault;
static uint8_t    g_laser_on;
static uint32_t   g_runaway_ms;
static float      g_pitch_pos_deg;  /* 调试用 */
static uint8_t    g_unwind_done;    /* 本次解绕是否已完成并重锁过基准 */
static uint8_t    g_prev_mode = 0xFFu;
static float      g_yaw_cmd_rpm;
static float      g_pitch_cmd_rpm;

/* ---- 运行时可改的当前参数值（GP_MSG_PARAM 改写的就是这些）----
 * 上电时用上面 #define 的默认值初始化；之后现场用 tools/tune_pitch.py 改，
 * 立刻生效、掉电恢复默认。为什么要留这个口子：俯仰振荡这种问题只能靠
 * "改一个数 -> 掰一下 -> 看结果"来定位，改一次代码烧一次要两分钟，
 * 一轮下来什么也试不出来。
 */
static float g_yaw_ff_sign    = YAW_FF_SIGN;
static float g_yaw_ff_gain    = YAW_FF_RPM_PER_RADS;
static float g_pitch_ff_sign  = PITCH_FF_SIGN;
static float g_pitch_ff_gain  = PITCH_FF_RPM_PER_RADS;
static float g_pitch_kp       = PITCH_KP;
static float g_pitch_ki       = PITCH_KI;
static float g_pitch_kd       = PITCH_KD;
static float g_pitch_ilim     = PITCH_ILIM;
static float g_pitch_out_rpm  = PITCH_OUT_RPM;
#if PITCH_PLAT_COMP
static float g_pitch_plat_sign = PITCH_PLAT_SIGN;   /* 默认开（老固件行为） */
#else
static float g_pitch_plat_sign = 0.0f;             /* 编译期全局关掉 */
#endif

/* ==========================================================================
 * 三、小工具
 * ========================================================================== */

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

static float angle_diff_rad(float target, float current)
{
    return normalize_angle_rad(target - current);
}

static void ahrs_init(float q[4])
{
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
}

static void ahrs_update(float q[4], const float g[3], const float a[3])
{
    MahonyAHRSupdateIMU(q, g[0], g[1], g[2], a[0], a[1], a[2]);
}

static void get_angle(const float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                    2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

/* ==========================================================================
 * 定点格式化（重要）
 *
 * ⚠ STM32 用的是 newlib-nano，它默认【不支持 printf 的 %f】——
 *   链接时不带 -u _printf_float 的话，遇到 %f 会直接把输出截断，
 *   表现就是日志里"p="后面一片空白，看着像变量没赋值，实际是格式化失败。
 *   这个坑很难从现象联想到原因，所以这里干脆自己做定点格式化，
 *   完全不依赖 printf 的浮点支持。
 *
 * decimals 只支持 0 / 1 / 2。
 * ========================================================================== */
static void fmt_f(char *out, int cap, float v, int decimals)
{
    long mul   = (decimals == 2) ? 100L : ((decimals == 1) ? 10L : 1L);
    int  neg   = (v < 0.0f);
    long scaled;

    if (neg)
    {
        v = -v;
    }
    scaled = (long)(v * (float)mul + 0.5f);

    if (decimals == 2)
    {
        snprintf(out, (size_t)cap, "%s%ld.%02ld", neg ? "-" : "",
                 scaled / 100L, scaled % 100L);
    }
    else if (decimals == 1)
    {
        snprintf(out, (size_t)cap, "%s%ld.%ld", neg ? "-" : "",
                 scaled / 10L, scaled % 10L);
    }
    else
    {
        snprintf(out, (size_t)cap, "%s%ld", neg ? "-" : "", scaled);
    }
}

/* ==========================================================================
 * 四、激光输出
 * ========================================================================== */

static void laser_init(void)
{
#if LASER_GPIO_ENABLE
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, GPIO_PIN_RESET);
    gpio.Pin   = LASER_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_GPIO_PORT, &gpio);
#endif
}

static void laser_apply(uint8_t on)
{
#if LASER_ALWAYS_ON
    /* 激光硬件常亮：不控制，只如实上报状态 */
    (void)on;
    g_laser_on = 1u;
#else
    if (on == g_laser_on)
    {
        return;
    }
    g_laser_on = on;
#if LASER_GPIO_ENABLE
    HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    gimbal_link_log(on ? "[LASER] ON" : "[LASER] OFF");
#endif
}

/* ==========================================================================
 * 五、编码器反馈
 * ========================================================================== */

/** 把原始读数解卷绕成连续角（无论驱动器给的是单圈 0~360 还是多圈连续值） */
static void enc_update(EncFb_t *e, float raw, uint32_t now)
{
    if (!e->seen)
    {
        e->raw  = raw;
        e->prev = raw;
        e->cont = raw;
        e->seen = 1u;
        e->t_ms = now;
        return;
    }
    {
        float d = raw - e->prev;
        /* 把跳变折到 ±180° 内：真机在 5ms 内不可能转过 180°，
         * 所以超过 180° 的跳变一定是"过零"而不是真的转动 */
        if (d > 180.0f)
        {
            d -= 360.0f;
        }
        else if (d < -180.0f)
        {
            d += 360.0f;
        }
        e->cont += d;
    }
    e->raw  = raw;
    e->prev = raw;
    e->t_ms = now;
}

/** 把两条 CAN 总线的接收邮箱抽干，解析出电机位置 */
static void yuntai_read_feedback(uint32_t now)
{
    CanRxFrame_t f;
    float   pos;
    int16_t sp, cur;

    /* 偏航总线（FDCAN1）：位置只用来做"绕线监视"和解绕，不参与稳定环 */
    while (can_bsp_get_frame(&hfdcan1, &f))
    {
        if (jc_parse_speed_reply(f.data, f.len, &pos, &sp, &cur))
        {
            enc_update(&g_yaw_fb, pos, now);
        }
        else if (jc_parse_position_reply(f.data, f.len, &pos))
        {
            enc_update(&g_yaw_fb, pos, now);
        }
    }

    /* 俯仰总线（FDCAN2）：位置环的唯一反馈源 */
    while (can_bsp_get_frame(&hfdcan2, &f))
    {
        if (jc_parse_speed_reply(f.data, f.len, &pos, &sp, &cur))
        {
            enc_update(&g_pitch_fb, pos, now);
        }
        else if (jc_parse_position_reply(f.data, f.len, &pos))
        {
            enc_update(&g_pitch_fb, pos, now);
        }
    }
}

static uint8_t enc_fresh(const EncFb_t *e, uint32_t now, uint32_t timeout_ms)
{
    return (uint8_t)((e->seen != 0u) && ((now - e->t_ms) < timeout_ms));
}

/* ==========================================================================
 * 六、锁基准（上电一次；上位机发 SET_ZERO 时再来一次）
 * ========================================================================== */

static void yuntai_lock_reference(void)
{
    g_yaw_lock_rad      = imuAngle[0];       /* 绝对方位基准 */
    g_pitch_plat_lock   = imuAngle[1];       /* 平台俯仰基准 */
    g_plat_dtheta       = 0.0f;              /* 平台俯仰变化量从 0 重新算 */
    g_plat_comp         = 0.0f;              /* 补偿量也从 0 重新起步 */
    g_pitch_motor_lock  = g_pitch_fb.seen ? g_pitch_fb.cont : 0.0f;
    g_pitch_tgt_deg     = g_pitch_motor_lock;

    g_yaw_fb.zero       = g_yaw_fb.cont;     /* 相对偏航角从这里开始算 0 */
    g_pitch_fb.zero     = g_pitch_fb.cont;

    PID_Reset(&g_yaw_pid);
    PID_Reset(&g_pitch_pid);
    PID_Reset(&g_yaw_unwind_pid);
    g_runaway_ms = 0u;

    {
        char b0[16], b1[16];
        char buf[64];
        fmt_f(b0, sizeof(b0), g_yaw_lock_rad * 57.29578f, 2);
        fmt_f(b1, sizeof(b1), g_pitch_motor_lock, 2);
        snprintf(buf, sizeof(buf), "[REF] yaw=%s pitch_motor=%s", b0, b1);
        gimbal_link_log(buf);
    }
}

/* ==========================================================================
 * 七、运行态：稳定 + 瞄准
 * ========================================================================== */

/** 陀螺零偏累加（在 MOTOR_BOOT 那 2 秒里每 2ms 调一次） */
static void gyro_bias_accumulate(void)
{
    uint8_t i;
    BMI088_read(gyro, acc, &temp);
    for (i = 0u; i < 3u; i++)
    {
        g_gyro_sum[i] += gyro[i];
    }
    g_gyro_n++;
}

/** 2 秒到点：算出零偏并打日志。无论采样质量如何都必须走完，绝不重来 */
static void gyro_bias_finalize(void)
{
    char    buf[72];
    uint8_t i;

    if (g_gyro_n < GYRO_BIAS_MIN_SAMPLES)
    {
        for (i = 0u; i < 3u; i++)
        {
            g_gyro_bias[i] = 0.0f;
        }
        gimbal_link_log("[GYRO] 采样太少，零偏按 0 处理（偏航可能会慢慢漂）");
        return;
    }
    for (i = 0u; i < 3u; i++)
    {
        g_gyro_bias[i] = clampf(g_gyro_sum[i] / (float)g_gyro_n,
                                -GYRO_BIAS_MAX, GYRO_BIAS_MAX);
    }
    /* 这三个数很有用：正常应该在 ±1°/s 以内。
     * 如果明显偏大，说明上电那 2 秒里晃了云台，重新上电即可。
     * （用 fmt_f 而不是 %f —— newlib-nano 不支持 %f，见 fmt_f 的说明） */
    {
        char bx[16], by[16], bz[16];
        fmt_f(bx, sizeof(bx), g_gyro_bias[0] * 57.29578f, 2);
        fmt_f(by, sizeof(by), g_gyro_bias[1] * 57.29578f, 2);
        fmt_f(bz, sizeof(bz), g_gyro_bias[2] * 57.29578f, 2);
        snprintf(buf, sizeof(buf), "[GYRO] bias(dps) x=%s y=%s z=%s n=%lu",
                 bx, by, bz, (unsigned long)g_gyro_n);
    }
    gimbal_link_log(buf);
}

static void yuntai_imu_update(uint32_t now)
{
    uint8_t i;
    if ((now - g_last_imu_ms) < IMU_PERIOD_MS)
    {
        return;
    }
    g_last_imu_ms = now;
    BMI088_read(gyro, acc, &temp);
    /* 扣掉上电时标定出来的零偏。不做这一步的话，零偏会被积分成
     * "角度在匀速漂"，偏航环就会驱动平台朝一个方向慢慢转。 */
    for (i = 0u; i < 3u; i++)
    {
        gyro_c[i] = gyro[i] - g_gyro_bias[i];
    }
    ahrs_update(imuQuat, gyro_c, acc);
    get_angle(imuQuat, &imuAngle[0], &imuAngle[1], &imuAngle[2]);
}

/** 偏航轴：给一个速度指令（rpm） */
static void yuntai_set_yaw(float rpm)
{
    jc_set_speed_rpm_x100(&hfdcan1, YUNTAI_MOTOR_YAW_ID, (int32_t)(rpm * 100.0f));
}

/** 俯仰轴：给一个速度指令（rpm） */
static void yuntai_set_pitch(float rpm)
{
    jc_set_speed_rpm_x100(&hfdcan2, YUNTAI_MOTOR_PITCH_ID, (int32_t)(rpm * 100.0f));
}

static void yuntai_control(uint32_t now)
{
    const GimbalCmd_t *cmd = gimbal_link_cmd();
    const uint8_t mode = cmd->mode;
    const float   dt   = (float)CONTROL_PERIOD_MS / 1000.0f;
    const float   rad2deg = 57.29578f;
    float yaw_rpm, pitch_rpm;
    uint8_t pitch_fb_ok;

    /* 模式一变就清掉"解绕已完成"标志，让下一次解绕重新开始 */
    if (mode != g_prev_mode)
    {
        g_prev_mode   = mode;
        g_unwind_done = 0u;
    }

    /* ---- 主动查一次俯仰位置（20ms = 50Hz）----
     * 为什么不能只靠驱动器"主动上报"：上报周期由驱动器决定、我们看不见也改不了。
     * 一旦它偏慢（比如 50ms），位置环就等于多了 25~50ms 的纯延迟 ——
     * 位置环的自激（掰一下开始抖、手扶就稳）几乎都是这么来的。
     * 主动查询把反馈延迟钉死在 20ms 以内，CAN 上这点流量完全不算什么。 */
    if ((now - g_last_read_ms) >= 20u)
    {
        g_last_read_ms = now;
        jc_read_position(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
    }

    /* ---- 编码器反馈：每周期抽干一次，队列永远不会堆满 ---- */
    yuntai_read_feedback(now);
    pitch_fb_ok = enc_fresh(&g_pitch_fb, now, PITCH_FB_TIMEOUT_MS);

    /* ---- 平台俯仰变化量的估计（互补滤波）----
     * 只用陀螺积分会漂；只用加速度计会被车的加减速带偏（1m/s² ≈ 5.7°）。
     * 所以：陀螺负责"快"（不受线加速度影响），Mahony 姿态角负责"慢"
     * （有重力做绝对基准、不会长期漂）—— 这正是互补滤波。
     * 这条估计是"平台俯仰补偿"的输入，见 PITCH_PLAT_COMP 的注释。 */
    if (g_imu_ok)
    {
        g_plat_dtheta += gyro_c[1] * dt;
        g_plat_dtheta += (dt / PLAT_COMP_ACC_TAU_S) *
                         ((imuAngle[1] - g_pitch_plat_lock) - g_plat_dtheta);
        g_plat_dtheta = clampf(g_plat_dtheta, -1.75f, 1.75f);   /* ±100° */
    }

    /* ---- 上位机要求重新锁零（进 AIM 之前会发一次）---- */
    if (gimbal_link_take_zero_req())
    {
        yuntai_lock_reference();
    }

    /* ---- 激光：只有 AIM 模式且上位机请求时才亮 ---- */
    laser_apply((uint8_t)((mode == GP_MODE_AIM) && (cmd->laser != 0u)));

    /* ======================================================================
     * 偏航轴
     * ==================================================================== */
    if ((mode == GP_MODE_UNWIND) && (g_yaw_fb.seen != 0u))
    {
        /* 解绕：暂时不用 IMU，改用电机自己的编码器，把相对角慢慢转回 0。
         * 此时激光已关、也没有瞄准要求，所以绝对指向漂掉没关系。 */
        float rel_deg = g_yaw_fb.cont - g_yaw_fb.zero;
        float err = -rel_deg;
        /* 速度上限：协议给的是 °/s，换算成 rpm 后还要夹到"推得动"的区间。
         * 低于脱困转速（约 10rpm）就完全推不动，解绕会卡在原地不动。 */
        float lim = (cmd->unwind_dps > 0u) ? ((float)cmd->unwind_dps / 6.0f)
                                           : UNWIND_RPM_MAX;
        lim = clampf(lim, UNWIND_RPM_MIN, UNWIND_RPM_MAX);
        if (fabsf(rel_deg) < UNWIND_TOL_DEG)
        {
            PID_Reset(&g_yaw_unwind_pid);
            yaw_rpm = 0.0f;
            /* ---- 解绕完成：必须把基准搬到新姿态 ----
             * 为什么非做不可：解绕把电机相对角转回了 0，但平台的绝对方位
             * 也跟着转了（例如解了 270°）。如果还沿用旧的 yaw_lock，
             * 角度环会立刻把平台转回旧朝向 —— 线又被拧回去了，等于白解。
             * 所以这里重锁一次基准，让"当前姿态"成为新的保持目标。 */
            if (!g_unwind_done)
            {
                g_unwind_done = 1u;
                yuntai_lock_reference();
                gimbal_link_log("[UNWIND] done -> reference re-locked");
            }
        }
        else
        {
            g_unwind_done = 0u;
            PID_Update(&g_yaw_unwind_pid, err, 0.0f, dt);
            yaw_rpm = clampf(g_yaw_unwind_pid.out, -lim, lim);
        }
    }
    else if (mode == GP_MODE_ESTOP)
    {
        PID_Reset(&g_yaw_pid);
        yaw_rpm = 0.0f;
    }
    else
    {
        /* 注意：上面那个 if 多带了一个"有偏航编码器反馈"的条件。
         * 所以【没有编码器反馈时不会解绕】，会落到这里退化成 STAB
         * （保持上电朝向）。原因：不知道相对角在哪儿就盲解绕，
         * 很可能朝反方向继续绕，把线拧得更紧。 */
        /* 正常/安全模式：
         *   目标 = 上电锁定的绝对方位 + 偏置
         * 偏置只在 AIM 模式下生效；STAB/IDLE 时偏置恒为 0，
         * 也就是"保持上电时的朝向不动" —— 这是掉线后的安全行为。 */
        float off_rad = (mode == GP_MODE_AIM) ? (cmd->yaw_offset_deg / rad2deg) : 0.0f;
        float err = angle_diff_rad(g_yaw_lock_rad + off_rad, imuAngle[0]);
        PID_Update(&g_yaw_pid, err, 0.0f, dt);
        /* 前馈：车体怎么转，前馈就让电机反向跟多少。
         * 它才是"手一动电机立刻跟着动"的原因；角度环只负责精修。 */
        yaw_rpm = g_yaw_pid.out + g_yaw_ff_sign * gyro_c[2] * g_yaw_ff_gain;
    }
    g_yaw_cmd_rpm = clampf(yaw_rpm, -YAW_OUT_RPM, YAW_OUT_RPM);
    yuntai_set_yaw(g_yaw_cmd_rpm);

    /* ======================================================================
     * 俯仰轴
     * ==================================================================== */
    if (mode == GP_MODE_ESTOP)
    {
        PID_Reset(&g_pitch_pid);
        pitch_rpm = 0.0f;
    }
    else if (!pitch_fb_ok)
    {
        /* ---- 降级路径：编码器反馈丢失 ----
         * 退化成"纯角速度前馈"：只对【转动】有反应，对缓慢倾斜没有反应。
         * 表现就是"俯仰只对快动有响应、慢慢倾斜没反应" —— 遇到这种
         * 现象先怀疑编码器反馈断了，而不是 PID 没调好。 */
        PID_Reset(&g_pitch_pid);
        pitch_rpm = g_pitch_ff_sign * gyro_c[1] * g_pitch_ff_gain;
    }
    else if (mode == GP_MODE_UNWIND)
    {
        /* 解绕时不折腾俯仰，保持在【锁定位置】即可。
         * 注意绝对不能写成"转到编码器 0" —— 编码器零点是任意值，
         * 那样会让俯仰猛地转过去（可能好几十度）。 */
        float err = g_pitch_motor_lock - g_pitch_fb.cont;
        PID_Update(&g_pitch_pid, err, 0.0f, dt);
        pitch_rpm = g_pitch_pid.out + g_pitch_ff_sign * gyro_c[1] * g_pitch_ff_gain;
    }
    else
    {
        float off_deg = (mode == GP_MODE_AIM) ? cmd->pitch_offset_deg : 0.0f;
        float plat_comp = 0.0f;
        /* 平台俯仰补偿：底座倾了多少，俯仰目标就反向挪多少。
         * 符号和前馈相反（sign_c = -sign_ff），理由见 PITCH_PLAT_COMP 注释。
         * 这里对补偿量做变化率限幅（60°/s）：即使符号判错、或者 IMU 被
         * 车的加减速带偏了一下，也只会"慢慢歪"而不会猛地一甩。 */
        if (g_pitch_plat_sign != 0.0f && g_imu_ok)
        {
            float want = -g_plat_dtheta * rad2deg * g_pitch_plat_sign;
            float dmax = PLAT_COMP_RATE_DPS * dt;
            want = clampf(want, -PITCH_TRAVEL_LIMIT_DEG, PITCH_TRAVEL_LIMIT_DEG);
            g_plat_comp = clampf(want, g_plat_comp - dmax, g_plat_comp + dmax);
            plat_comp = g_plat_comp;
        }
        else if (g_pitch_plat_sign == 0.0f)
        {
            g_plat_comp = 0.0f;      /* 关掉时归零，重新打开时从 0 平滑起步 */
        }
        else
        {
            plat_comp = g_plat_comp; /* IMU 异常：冻结在最后一次补偿值 */
        }
        float tgt = g_pitch_motor_lock + off_deg + plat_comp;
        /* 行程软限位：别撞机构、别把线扯断 */
        tgt = clampf(tgt, g_pitch_motor_lock - PITCH_TRAVEL_LIMIT_DEG,
                          g_pitch_motor_lock + PITCH_TRAVEL_LIMIT_DEG);
        g_pitch_tgt_deg = tgt;

        float err = tgt - g_pitch_fb.cont;
        PID_Update(&g_pitch_pid, err, 0.0f, dt);
        pitch_rpm = g_pitch_pid.out + g_pitch_ff_sign * gyro_c[1] * g_pitch_ff_gain;

        /* ---- 失控保护 ----
         * 位置环方向反了、或者机构卡死时，误差会一直很大。
         * 这时继续输出就是"朝一个方向狂奔"，必须立刻降级成纯前馈并报故障。 */
        if (fabsf(err) > PITCH_RUNAWAY_DEG)
        {
            g_runaway_ms += CONTROL_PERIOD_MS;
            if (g_runaway_ms > PITCH_RUNAWAY_MS)
            {
                g_fault |= FAULT_PITCH_RUNAWAY;
                PID_Reset(&g_pitch_pid);
                pitch_rpm = g_pitch_ff_sign * gyro_c[1] * g_pitch_ff_gain;
            }
        }
        else
        {
            g_runaway_ms = 0u;
            g_fault &= (uint8_t)~FAULT_PITCH_RUNAWAY;
        }
    }
    g_pitch_cmd_rpm = clampf(pitch_rpm, -g_pitch_out_rpm, g_pitch_out_rpm);
    g_pitch_pos_deg = g_pitch_fb.cont;
    yuntai_set_pitch(g_pitch_cmd_rpm);

    /* ---- 反馈丢失告警（每秒最多一条，不刷屏） ---- */
    if ((now - g_last_fbwarn_ms) > FB_WARN_PERIOD_MS)
    {
        if (!pitch_fb_ok)
        {
            g_last_fbwarn_ms = now;
            g_fault |= FAULT_PITCH_FB_LOST;
            gimbal_link_log("[WARN] 俯仰编码器无反馈 -> 退化为纯前馈");
        }
        else
        {
            g_fault &= (uint8_t)~FAULT_PITCH_FB_LOST;
        }
    }
}

/* 慢速调试文本：现场调参主要看这一条 */
static void yuntai_debug(uint32_t now)
{
    char buf[96];
    const GimbalCmd_t *cmd = gimbal_link_cmd();
    if ((now - g_last_dbg_ms) < DEBUG_PERIOD_MS)
    {
        return;
    }
    g_last_dbg_ms = now;
    /* 每 5s 顺带报一次链路统计：crc 一直涨 = 波特率/线长/共地有问题；
     * ovf/txdrop 一直涨 = H723 这一侧处理不过来 */
    if ((now - g_last_stat_ms) > 5000u)
    {
        g_last_stat_ms = now;
        gimbal_link_log(gimbal_link_stats());
    }
    {
        /* 注意：这里全部用 fmt_f，不能写 %f（newlib-nano 会截断输出） */
        char b0[16], b1[16], b2[16], b3[16], b4[16], b5[16], b6[16];
        /* 俯仰编码器反馈的"新鲜度"（ms）。
         * 为什么必须显示：位置环的稳定性和这个数直接相关 —— 反馈越旧，
         * 等效延迟越大，同样的 KP 就越容易自激。上次数据没刷新时，
         * 位置环其实在拿"旧位置"当"现在"，表现出来就是低频抖动。
         * 正常应该在 10ms 以内；如果一直是几十毫秒，就要提高查询频率。 */
        uint32_t fb_age = (g_pitch_fb.seen != 0u) ? (now - g_pitch_fb.t_ms) : 9999u;
        fmt_f(b0, sizeof(b0), g_pitch_pos_deg, 1);
        fmt_f(b1, sizeof(b1), g_pitch_tgt_deg, 1);
        fmt_f(b2, sizeof(b2), g_pitch_cmd_rpm, 0);
        fmt_f(b3, sizeof(b3), g_yaw_cmd_rpm, 0);
        fmt_f(b4, sizeof(b4), cmd->yaw_offset_deg, 1);
        fmt_f(b5, sizeof(b5), cmd->pitch_offset_deg, 1);
        fmt_f(b6, sizeof(b6), g_plat_comp, 1);
        snprintf(buf, sizeof(buf),
                 "[DBG] P p=%s t=%s o=%s | Y o=%s | off=%s %s | c=%s | fb=%ums",
                 b0, b1, b2, b3, b4, b5, b6, (unsigned)fb_age);
    }
    gimbal_link_log(buf);
}

/* ==========================================================================
 * 八、状态机
 * ========================================================================== */

void yuntai_init(void)
{
    can_bsp_init();
    laser_init();
    ahrs_init(imuQuat);
    g_laser_on = 0u;
    g_fault    = FAULT_NONE;
    g_state    = YUNTAI_STATE_BMI088_INIT;
    g_state_ms = HAL_GetTick();
    g_cmd_sent = 0u;
    gimbal_link_log("=== YunTai Start ===");
}

/** 把当前状态填进遥测（所有状态都要填，否则上位机一直以为云台没起来） */
static void yuntai_update_telem(uint32_t now)
{
    static uint32_t last_ms;
    GimbalTelem_t t;
    const float rad2deg = 57.29578f;

    /* 主循环是空转轮询，跑得比 1MHz 还快；遥测没必要每圈都刷一遍 */
    if ((now - last_ms) < 2u)
    {
        return;
    }
    last_ms = now;

    t.state  = (uint8_t)g_state;
    t.fault  = g_fault;
    t.yaw_deg   = imuAngle[0] * rad2deg;
    t.pitch_deg = imuAngle[1] * rad2deg;
    t.roll_deg  = imuAngle[2] * rad2deg;
    /* 相对车身的电机角：这个值就是"绕线"的度量，每跑一圈会净变化 -360° */
    t.yaw_motor_deg   = g_yaw_fb.seen ? (g_yaw_fb.cont - g_yaw_fb.zero) : 0.0f;
    t.pitch_motor_deg = g_pitch_fb.seen ? g_pitch_fb.cont : 0.0f;
    t.gyro_y_dps = gyro_c[1] * rad2deg;
    t.gyro_z_dps = gyro_c[2] * rad2deg;
    t.flags = 0u;
    if (g_state == YUNTAI_STATE_RUNNING)      t.flags |= GP_ST_CLOSED_LOOP;
    if (g_state == YUNTAI_STATE_RUNNING)      t.flags |= GP_ST_READY;
    if (g_state >= YUNTAI_STATE_SET_MODE)     t.flags |= GP_ST_SPEED_MODE;
    if (enc_fresh(&g_pitch_fb, now, PITCH_FB_TIMEOUT_MS)) t.flags |= GP_ST_ENCODER_OK;
    if (g_laser_on)                           t.flags |= GP_ST_LASER_ON;
    t.uptime_ms = now;
    gimbal_link_set_telem(&t);
}

void yuntai_control_loop(void)
{
    uint32_t now = HAL_GetTick();

    switch (g_state)
    {
    /* ---- 1. 初始化 BMI088 ---- */
    case YUNTAI_STATE_BMI088_INIT:
        if ((now - g_state_ms) >= BMI_RETRY_MS)
        {
            uint8_t result;
            char buf[40];
            g_bmi_retry++;
            snprintf(buf, sizeof(buf), "[BMI088] try #%lu ...",
                     (unsigned long)g_bmi_retry);
            gimbal_link_log(buf);
            if (g_bmi_retry == 10u)
            {
                /* 一直起不来就说明硬件有问题，而不是"再等等就好" */
                gimbal_link_log("[WARN] BMI088 连试 10 次都失败："
                                "查 SPI2 预分频(=32)/CS(PC0,PC3)/供电");
            }
            result = BMI088_init();
            snprintf(buf, sizeof(buf), "[BMI088] result=%u", result);
            gimbal_link_log(buf);
            if (result == BMI088_NO_ERROR)
            {
                g_imu_ok = 1u;
                ahrs_init(imuQuat);
                gimbal_link_log("[BMI088] OK -> motor boot delay");
                g_state    = YUNTAI_STATE_MOTOR_BOOT;
                g_state_ms = now;
            }
            else
            {
                /* 这是启动路径上唯一剩下的条件判据 —— 没 IMU 就没法控制，
                 * 所以必须等它；其余步骤全是纯延时，不会卡住。 */
                g_state_ms = now;
            }
        }
        break;

    /* ---- 2. 等驱动器上电自检（纯延时，刻意不做任何判断）---- */
    case YUNTAI_STATE_MOTOR_BOOT:
        /* 顺手把陀螺零偏标定做了：每 2ms 采一次、无条件累加。
         * 这是"上电后云台朝一个方向慢慢转"的根因修复，且不额外花时间。
         * ⚠ 这里绝不能加"必须静止才计数/否则清零重来"之类条件 ——
         *   那种写法会卡死启动流程（见文件顶部铁律）。 */
        if ((now - g_last_imu_ms) >= IMU_PERIOD_MS)
        {
            g_last_imu_ms = now;
            gyro_bias_accumulate();
        }
        if ((now - g_state_ms) > MOTOR_BOOT_DELAY_MS)
        {
            gyro_bias_finalize();
            gimbal_link_log("[MOTOR] boot delay done");
            g_state    = YUNTAI_STATE_CLOSED_LOOP;
            g_state_ms = now;
            g_cmd_sent = 0u;
        }
        break;

    /* ---- 3. 进入闭环（只发一次；运行期反复写这个寄存器会清掉驱动器
     *         的内部状态，表现是水平电机完全不动）---- */
    case YUNTAI_STATE_CLOSED_LOOP:
        if (!g_cmd_sent)
        {
            g_cmd_sent = 1u;
            jc_enter_closed_loop(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
            jc_enter_closed_loop(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
            gimbal_link_log("[MOTOR] enter closed loop");
        }
        if ((now - g_state_ms) > CLOSED_LOOP_DELAY_MS)
        {
            g_state    = YUNTAI_STATE_SET_MODE;
            g_state_ms = now;
            g_cmd_sent = 0u;
        }
        break;

    /* ---- 4. 切速度模式（同样只发一次）---- */
    case YUNTAI_STATE_SET_MODE:
        if (!g_cmd_sent)
        {
            g_cmd_sent = 1u;
            jc_set_control_mode(&hfdcan1, YUNTAI_MOTOR_YAW_ID, JC_MODE_SPEED);
            jc_set_control_mode(&hfdcan2, YUNTAI_MOTOR_PITCH_ID, JC_MODE_SPEED);
            gimbal_link_log("[MOTOR] set speed mode");
        }
        if ((now - g_state_ms) > SET_MODE_DELAY_MS)
        {
            g_state    = YUNTAI_STATE_LOCK;
            g_state_ms = now;
            g_last_imu_ms = now;
            gimbal_link_log("[IMU] locking reference ...");
        }
        break;

    /* ---- 5. 锁基准 + 预热编码器反馈 ----
     * 期间给两个电机发 0 速（压住），并主动读一次俯仰位置。
     * 注意：这里【绝不能】等"必须拿到编码器反馈"才走 —— 反馈异常时
     * 状态机会永久停在这里，两个轴都不会发出任何指令。
     */
    case YUNTAI_STATE_LOCK:
        yuntai_imu_update(now);
        if ((now - g_last_read_ms) >= 50u)
        {
            g_last_read_ms = now;
            jc_read_position(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
        }
        yuntai_read_feedback(now);
        if ((now - g_state_ms) > LOCK_DELAY_MS)
        {
            /* 注意用 g_* 当前值而不是宏：上位机可能在启动阶段就下发了新参数 */
            PID_Init(&g_yaw_pid,   YAW_KP,   YAW_KI,   YAW_KD,   YAW_OUT_RPM);
            PID_Init(&g_pitch_pid, g_pitch_kp, g_pitch_ki, g_pitch_kd, g_pitch_out_rpm);
            PID_Init(&g_yaw_unwind_pid, UNWIND_KP, 0.0f, 0.0f, UNWIND_RPM_MAX);
            g_yaw_pid.integral_limit   = YAW_ILIM;
            g_pitch_pid.integral_limit = g_pitch_ilim;
            g_pitch_pid.dead_zone      = PITCH_DEADZONE_DEG;
            yuntai_lock_reference();
            g_state    = YUNTAI_STATE_RUNNING;
            g_state_ms = now;
            g_last_ctrl_ms = now;
            g_last_dbg_ms  = now;
            gimbal_link_log("=== STABILIZATION ACTIVE ===");
        }
        break;

    /* ---- 6. 稳定 + 瞄准 ---- */
    case YUNTAI_STATE_RUNNING:
        yuntai_imu_update(now);
        if ((now - g_last_ctrl_ms) >= CONTROL_PERIOD_MS)
        {
            g_last_ctrl_ms = now;
            yuntai_control(now);
        }
        yuntai_debug(now);
        break;

    default:
        g_state    = YUNTAI_STATE_BMI088_INIT;
        g_state_ms = now;
        break;
    }

    /* 遥测与链路必须在所有状态下都刷新：
     * 上位机靠 GIMBAL_STATE 判断"云台起来没有"，
     * 如果只在 RUNNING 里发，上位机会永远停在等启动。 */
    yuntai_update_telem(now);
    gimbal_link_poll(now);
}

/* ==========================================================================
 * 七、运行时就地调参（GP_MSG_PARAM 的落地实现）
 * ==========================================================================
 * 单位约定：value = 真实值 × 100（int16）。例：
 *     俯仰 KP 12.5      -> pid=0x05, value=1250
 *     俯仰前馈符号 -1   -> pid=0x03, value=-100
 *     俯仰前馈增益 0    -> pid=0x04, value=0     （= 关掉前馈）
 *     俯仰 KD 0.10      -> pid=0x07, value=10
 *
 * 三道安全：
 *   1) 每个参数都有硬限幅（下面的 clampf），上位机给飞了也不会毁机构；
 *   2) 前馈符号 / 前馈增益 / PID 三个系数一变就 PID_Reset —— 否则积分项
 *      会带着"上一个方向"的记忆继续推，瞬时长出一个大输出（也就等于
 *      把刚调好的东西又撞歪一次）；
 *   3) 本函数在主循环里被调用，跟 5ms 控制节拍是同一个上下文，
 *      不存在"改到一半被控制循环读走"的并发问题。
 * ========================================================================== */
void gimbal_link_on_param(uint8_t pid, int16_t value)
{
    float  v = (float)value / 100.0f;
    float  s = (v >= 0.0f) ? 1.0f : -1.0f;
    char   buf[72];
    char   n1[16];
    char   n2[16];

    switch (pid)
    {
    case GP_PARAM_YAW_FF_SIGN:
        g_yaw_ff_sign = s;
        break;

    case GP_PARAM_YAW_FF_GAIN:
        g_yaw_ff_gain = clampf(v, 0.0f, 200.0f);
        break;

    case GP_PARAM_PITCH_FF_SIGN:
        if (s != g_pitch_ff_sign)
        {
            g_pitch_ff_sign = s;
            PID_Reset(&g_pitch_pid);      /* 换方向必须清掉旧积分 */
        }
        break;

    case GP_PARAM_PITCH_FF_GAIN:
        /* 上限 60：这个量是"速率反馈"，过大等于自己给自己加正反馈，
         * 正常只会用到 5~20（理论抵消量 ≈ 9.55 rpm per rad/s） */
        g_pitch_ff_gain = clampf(v, 0.0f, 60.0f);
        break;

    case GP_PARAM_PITCH_KP:
        g_pitch_kp = clampf(v, 0.0f, 40.0f);
        g_pitch_pid.kp = g_pitch_kp;
        PID_Reset(&g_pitch_pid);
        break;

    case GP_PARAM_PITCH_KI:
        g_pitch_ki = clampf(v, 0.0f, 200.0f);
        g_pitch_pid.ki = g_pitch_ki;
        PID_Reset(&g_pitch_pid);
        break;

    case GP_PARAM_PITCH_KD:
        g_pitch_kd = clampf(v, 0.0f, 5.0f);
        g_pitch_pid.kd = g_pitch_kd;
        PID_Reset(&g_pitch_pid);
        break;

    case GP_PARAM_PITCH_ILIM:
        g_pitch_ilim = clampf(v, 0.0f, 5.0f);
        g_pitch_pid.integral_limit = g_pitch_ilim;
        break;

    case GP_PARAM_PITCH_OUT_RPM:
        g_pitch_out_rpm = clampf(v, 5.0f, 300.0f);
        g_pitch_pid.output_limit = g_pitch_out_rpm;
        break;

    case GP_PARAM_PITCH_PLAT_SIGN:
        /* 平台俯仰补偿：0 = 关，±1 = 开并定方向。
         * 切换时把估计值和补偿量一起归零，避免"换符号瞬间目标跳几十度"。 */
        g_pitch_plat_sign = (v == 0.0f) ? 0.0f : s;
        g_plat_comp       = 0.0f;
        g_plat_dtheta     = 0.0f;
        PID_Reset(&g_pitch_pid);
        break;

    case GP_PARAM_DUMP:
        /* 只汇报，不改动。两行，方便上位机直接打印 */
        fmt_f(n1, sizeof(n1), g_pitch_ff_sign, 2);
        fmt_f(n2, sizeof(n2), g_pitch_ff_gain, 1);
        snprintf(buf, sizeof(buf), "[PARAM] pitFF %s x%s", n1, n2);
        gimbal_link_log_force(buf);
        fmt_f(n1, sizeof(n1), g_pitch_kp, 2);
        fmt_f(n2, sizeof(n2), g_pitch_ki, 2);
        snprintf(buf, sizeof(buf), "[PARAM] pitKp %s Ki %s", n1, n2);
        gimbal_link_log_force(buf);
        fmt_f(n1, sizeof(n1), g_pitch_kd, 2);
        fmt_f(n2, sizeof(n2), g_pitch_ilim, 2);
        snprintf(buf, sizeof(buf), "[PARAM] pitKd %s iLim %s", n1, n2);
        gimbal_link_log_force(buf);
        fmt_f(n1, sizeof(n1), g_pitch_out_rpm, 0);
        fmt_f(n2, sizeof(n2), g_yaw_ff_gain, 1);
        snprintf(buf, sizeof(buf), "[PARAM] pitOut %s yawFFg %s", n1, n2);
        gimbal_link_log_force(buf);
        fmt_f(n1, sizeof(n1), g_pitch_plat_sign, 1);
        fmt_f(n2, sizeof(n2), g_plat_comp, 1);
        snprintf(buf, sizeof(buf), "[PARAM] platComp %s now %s", n1, n2);
        gimbal_link_log_force(buf);
        return;

    default:
        snprintf(buf, sizeof(buf), "[PARAM] unknown id=%u", (unsigned)pid);
        gimbal_link_log_force(buf);
        return;
    }

    /* 每条成功的修改都确认一条文本 —— 现场最怕"以为改了、其实没生效" */
    fmt_f(n1, sizeof(n1), v, 2);
    snprintf(buf, sizeof(buf), "[PARAM] id=%u <- %s OK", (unsigned)pid, n1);
    gimbal_link_log_force(buf);
}
