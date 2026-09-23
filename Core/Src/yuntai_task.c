/**
 * 云台双轴稳定控制
 *
 * 架构（简化自 speed_control 工程）：
 *   两个电机都跑【速度模式】，位置环全部在 MCU 里做，驱动器只当执行器。
 *
 *   Yaw   : IMU 角度闭环
 *             err = 目标偏航角 - IMU 偏航角  -> PID -> 速度指令
 *
 *   Pitch : 编码器位置环（IMU 只给目标角）
 *             目标电机角 = 上电电机角 - (IMU俯仰角 - 上电IMU俯仰角)
 *             err = 目标电机角 - 编码器角度 -> PID -> 速度指令
 *           再叠加陀螺角速度前馈。
 *
 * 为什么俯仰的反馈用编码器而不是 IMU：
 *   IMU 装在偏航轴上，看不到俯仰电机的转动；编码器能看到电机本体。
 *   IMU 只给目标角，闭环反馈用编码器 —— 标准级联结构。
 *   （把位置环交给驱动器去做的那套方案已实测行不通，不要再回头试。）
 *
 * ============================ 启动状态机 ============================
 * 上电到开始正常控制约 3.4s，依次经过：
 *
 *  0) 上电自由态  yuntai_init(): 只初始化 CAN，不发任何电机指令。
 *                 驱动器停在上电默认态(未使能) -> 此时电机可被手掰动。
 *  1) ST_BMI_INIT          ~0.2s 每 200ms 试一次 BMI088_init()
 *  2) ST_WAIT_MOTOR_BOOT    2.0s 纯延时，等驱动器上电自检完成
 *                                (= 原版 CALIBRATE_DELAY，无条件推进)
 *  3) ST_ENTER_CLOSED_LOOP 0.5s  发 0x00A2(进入闭环)到两条总线，每 100ms 重发
 *  4) ST_SET_SPEED_MODE    0.5s  发 0x0060=1(速度模式)，每 100ms 重发
 *  5) ST_LOCK              0.2s  不发指令；锁定 yaw_target / 俯仰基准，PID_Init
 *  6) ST_RUN               永久   每 2ms 读 IMU，每 5ms 发一次速度指令
 *
 *  ❗【启动路径上不允许有任何会阻塞的判据】。
 *    凡是"要满足某条件才能往下走"的逻辑，一旦条件因噪声/环境永远不满足，
 *    整个状态机就停在原地 —— 而表现是"云台一直是软的"，极难定位。
 *    启动阶段只允许纯延时推进；带条件的逻辑只能放在 ST_RUN 里做后台降级。
 *
 *  ❗0x0060(控制模式) 和 0x00A2(闭环使能) 只在 3/4 阶段发，进 ST_RUN 后一律不再碰。
 *    运行期反复写这两个寄存器会不断复位驱动器内部运动状态 —— 踩过：
 *    表现是水平电机完全不动。
 *
 *  ❗俯仰进入 ST_RUN 后还要再等第一帧编码器反馈才算真闭环：
 *    50ms 内收不到 -> PID_Reset，退化成纯角速度前馈(只响应转动、不响应慢漂)；
 *    首次收到 -> 锁 pitch_motor_locked，打印 "pitch encoder online"。
 *    该基准只锁一次、永不重建。
 *
 *  状态切换打印(缺哪条就说明卡在哪一步)：
 *    === YunTai Start === -> BMI088 OK -> startup delay done
 *    -> motor closed loop -> motor speed mode
 *    -> === STABILIZATION ACTIVE === -> pitch encoder online
 * ===================================================================
 */

#include "yuntai_task.h"
#include "can_bsp.h"
#include "jc4310.h"
#include "fdcan.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "user_lib.h"
#include "pid.h"
#include "debug_uart.h"
#include <math.h>
#include <stdio.h>

#define RAD2DEG        57.29578f
#define IMU_PERIOD     2    /* ms -> 500Hz */
#define CTRL_PERIOD    5    /* ms -> 200Hz（与原版一致，不要改） */

/* ---------- 陀螺零偏校准：已删除 ----------
 * 原版第一次提交的代码根本没有这一步，直接拿原始陀螺值用，照样能跑。
 * 我后加的"静止采样求平均"引入了一个会【卡死启动流程】的故障：
 *   它要求连续采满 600 个"没在动"的样本，任一样本超过阈值就把
 *   已采样本【整批清零重来】。而如果这颗 BMI088 的陀螺零偏本身就
 *   超过静置阈值(0.06 rad/s ≈ 3.4°/s)，就永远采不满，只能干等到 12s 超时。
 *   ⟹ 表现就是"迟迟进入不了状态"、云台一直是软的、能被手掰动。
 *
 * 代价权衡：不做零偏补偿，偏航角会因零偏残留而缓慢漂移。
 *   但这个漂移是可接受的，而"卡死启动"是不可接受的 —— 宁可漂，不能卡。
 * 若以后确实需要补零偏：只能在 ST_RUN 里【后台】慢慢估、慢慢用，
 *   绝不能让任何估计逻辑参与启动流程的放行条件。                     */

/* ---------- Yaw 轴 ----------
 * ❗kp/ki/out/ILIM 是原工程【实测调好】的，不要再按理论去"修正"。
 *   我曾按量纲换算改成 ki=1432 / kd=0.012 / ILIM=0.021 / 输出限幅=100，
 *   结果偏航直接不工作了。已恢复原值。
 *
 * ❗kd 从原版的 10 改成 5，原因在 MahonyAHRS.c：
 *   原版 `sampleFreq` 硬编码 1000，而 IMU 实际按 IMU_PERIOD=2ms(500Hz) 采，
 *   积分步长只有真实时间的 1/2 ⟹ 姿态角只报一半。
 *   修正后姿态报量变为原来的 2 倍，微分项也涨 2 倍，原 kd=10 相当于 20，
 *   会把高频噪声放大成振荡（历史上"剧烈左右摇摆"就是这个）。
 *   取 5 正好复原原版实测过的等效阻尼。 */
#define YAW_KP          200.0f   /* rpm/rad */
#define YAW_KI           30.0f   /* rpm/(rad*s) */
#define YAW_KD            5.0f   /* rpm/(rad/s)，见上方说明 */
#define YAW_OUT          50.0f   /* rpm */
#define YAW_ILIM         10.0f

/* ⚠⚠ 这是与原版相比唯一的【新增】项，也是偏航"看起来不工作"的根源。
 *
 * 原版偏航唯一的输出就是 `yaw_pid.out`，量级：
 *   P 项 kp=200rpm/rad → 1° 误差只有 3.5rpm
 *   I 项 ki=30 → 1° 误差下每秒只抬 0.52rpm（要 19s 才能到 10rpm）
 * 而直驱云台的静摩擦脱困速度在 10rpm 量级
 * ⟹ 手慢慢扭底盘时指令永远低于脱困速度，**电机根本不动**，看起来就是"不调节"。
 *
 * 俯仰之所以"能调"，是因为它有一个很大的角速度前馈 `gyro[1]*1500`
 * （1rad/s = 143rpm），手一转就远超脱困速度。
 * 参考工程 speed_control 的偏航也有同样的前馈：
 *   ff = -gyro_z(deg/s) * (1/6) * 7.0  = 理论抵消量的 1.167 倍
 * 本工程缺的就是这一项。折算成 rpm*100：
 *   1rad/s → 57.3deg/s → 57.3/6*7.0 = 66.85rpm → *100 = 6685
 *
 * ❗符号若反了，偏航会顺着转动方向越跑越快（助纣为虐）。
 *   首次上电若发现偏航单向飞转，把 YAW_FF_SIGN 改成 (+1.0f) 即可。
 *   想临时关掉前馈单独调 PID，把 YAW_FF_GAIN 置 0。 */
#define YAW_FF_GAIN   6685.0f
#define YAW_FF_SIGN  (-1.0f)

/* ---------- 开环自检开关 ----------
 * 置 1 时忽略 PID，直接给偏航（水平）电机一个固定转速指令。
 * 用来把问题一刀切开：
 *   电机转起来 -> CAN+速度模式这条通路是好的，问题在控制环/IMU
 *   电机不转   -> 问题在驱动/接线/ID/模式，跟 PID 参数无关
 * ⚠ 已被下面的 YAW_BRINGUP_TEST 取代（后者信息量更大），保留作为备选。 */
#define YAW_OPENLOOP_TEST   0
#define YAW_OPENLOOP_RPM    30.0f

/* ---------- 偏航电机独立自检（不需要电脑，看电机动作就行）----------
 * 置 1 时：整个状态机、IMU、PID、前馈、俯仰轴全部跳过，
 * 只对偏航总线 FDCAN1 做一件事 —— 轮流试各个驱动器 ID：
 *
 *     给 ID=1 发使能+速度模式+30rpm，持续 1.5s → 停 0.7s →
 *     给 ID=2 发同样的东西          → 停 0.7s →
 *     ... 一直到 YAW_BRINGUP_ID_MAX，然后循环
 *
 * 【怎么读结果】
 *   1) 在某一轮里电机转起来了 —— 记下是第几轮，那就是真正的 ID，
 *      改 YUNTAI_MOTOR_YAW_ID 即可解决问题。
 *   2) 自始至终电机一直是软的、不转 ——
 *      ⟹ FDCAN1 硬件侧问题，与控制代码无关：
 *        · 偏航驱动器有没有上电（看驱动器指示灯）
 *        · CAN1-H / CAN1-L 接线、是否插反
 *        · 总线上有没有 120Ω 终端电阻（两端各一个）
 *        · 偏航驱动器的波特率设置是否与俯仰那只一致
 *        · 前一次跑飞/振荡后驱动器是否锁在故障态 —— 断整机电源重上电
 * ⚠ 这一轮只发 FDCAN1，不会动俯仰轴，安全。                      */
#define YAW_BRINGUP_TEST     0
#define YAW_BRINGUP_ID_MAX   6
#define YAW_BRINGUP_RPM      30.0f
#define YAW_BRINGUP_ON_MS    1500   /* 每个 ID 通电时长 */
#define YAW_BRINGUP_OFF_MS    700   /* 每个 ID 之间的停顿 */

/* ---------- Pitch 轴 ----------
 * 参数换算（speed_control 用"每周期累加"形式，本工程 PID_Update 乘 dt）：
 *   kp_本 = kp_彼     ki_本 = ki_彼/dt     kd_本 = kd_彼*dt
 *   彼方取值 2.8 / 0.02 / 0.10 @500Hz  ->  2.8 / 10 / 0.0002            */
#define PITCH_KP          2.8f   /* rpm/deg */
#define PITCH_KI         10.0f   /* rpm/(deg*s) */
#define PITCH_KD       0.0002f
#define PITCH_OUT        50.0f   /* rpm */
#define PITCH_ILIM        0.6f   /* 最大积分贡献 = 10*0.6 = 6rpm */
#define PITCH_FF_GAIN  1500.0f   /* = 原版 gyro[1]*1500，实测值，勿改 */
#define PITCH_FF_SIGN  (-1.0f)
#define PITCH_LIMIT_DEG  80.0f   /* 相对上电位置的行程软限位 */

#define PITCH_FB_TIMEOUT_MS   50
#define LOCK_MS              200

/* ---------- 启动时序（严格对齐原版，不要随意压缩）----------
 * 原版：BMI 初始化 -> 空等 2000ms -> 发 0x00A2 进闭环 -> 等 500ms
 *       -> 发速度模式(0x0060=1) -> 等 500ms -> 锁定姿态 -> RUN
 *
 * ❗驱动器上电自检需要时间，指令发太早会被忽略；
 * ❗更关键的是【运行期绝对不能反复重发 0x0060/0x00A2】 —— 那会不断把
 *   驱动器内部的运动状态复位。旧代码每 500ms 重发一次，偏航指令本来
 *   就只有 3.5rpm（抬不过静摩擦），一复位就更起不来 —— 水平电机"不动"
 *   就是这么来的。现在控制模式只在启动期发，进 RUN 后一律不再碰。   */
#define STARTUP_DELAY_MS    2000   /* = 原版 CALIBRATE_DELAY */
#define CMD_STAGE_MS         500   /* = 原版 CLOSED_LOOP_DELAY / SET_MODE_DELAY */
#define CMD_RESEND_MS        100   /* 阶段内重发间隔：防丢帧，且不会打满 TxFifo */

#define ENABLE_RUNTIME_DEBUG   0
#define DEBUG_PERIOD        1000

#define INS_YAW_ADDRESS_OFFSET   0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET  2

/* ===================== 状态 ===================== */
typedef enum {
    ST_BMI_INIT = 0,
    ST_WAIT_MOTOR_BOOT,     /* 纯延时，等驱动器上电自检完成 */
    ST_ENTER_CLOSED_LOOP,
    ST_SET_SPEED_MODE,
    ST_LOCK,
    ST_RUN,
} State;

static State    s_state = ST_BMI_INIT;
static uint32_t s_state_ms;
static uint32_t s_imu_ms;
static uint32_t s_ctrl_ms;
static uint32_t s_debug_ms;

static float gyro[3], acc[3], temp;
static float imuQuat[4], imuAngle[3];

/* Yaw */
static PID_Controller yaw_pid;
static float          yaw_target;

/* Pitch */
static PID_Controller pitch_pid;
static JcFeedback_t   pitch_fb;
static uint32_t       pitch_fb_ms;
static uint8_t        pitch_base_ok;
static float          pitch_plat_locked;   /* 上电时的 IMU 俯仰角(rad) */
static float          pitch_motor_locked;  /* 上电时的电机角度(deg) */

/* ===================== 姿态解算 ===================== */
static void AHRS_init(float q[4])
{
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
}

static void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw   = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll  = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]), 2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

static void imu_read(void)
{
    /* 直接用原始值，不做零偏补偿。原因见文件上方的"陀螺零偏校准：已删除"。 */
    BMI088_read(gyro, acc, &temp);
}

static void ahrs_update(void)
{
    MahonyAHRSupdateIMU(imuQuat, gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
    GetAngle(imuQuat,
             imuAngle + INS_YAW_ADDRESS_OFFSET,
             imuAngle + INS_PITCH_ADDRESS_OFFSET,
             imuAngle + INS_ROLL_ADDRESS_OFFSET);
}

static float angle_diff_rad(float target, float current)
{
    return normalize_angle_rad(target - current);
}

/* ===================== 俯仰编码器反馈 =====================
 * 在 CAN 中断里直接解析并保存。轮询方式下"读到的不一定是对的帧"，
 * 判据一旦不成立位置环就会静默退化成开环，而且很难察觉。      */
void can_pitch_rx_hook(const uint8_t *data, uint8_t len)
{
    float pos_deg;

    if (jc_parse_position_reply(data, len, &pos_deg))
    {
        pitch_fb.pos_deg = pos_deg;
        pitch_fb.valid   = 1;
        pitch_fb_ms      = HAL_GetTick();
    }
    else
    {
        JcFeedback_t f;
        if (jc_parse_feedback(data, len, &f))
        {
            pitch_fb.pos_deg = f.pos_deg;
            pitch_fb.valid   = 1;
            pitch_fb_ms      = HAL_GetTick();
        }
    }
}

/* ===================== 启动期电机配置 =====================
 * 只在进入 RUN 之前发，运行期一律不再动控制模式寄存器。
 * FDCAN 关掉了自动重传，单帧丢了就永久丢了，所以每个阶段内
 * 按 CMD_RESEND_MS 重发几次 —— 同时最多只有 2 帧，不会打满 3 深的 TxFifo。 */
static void motor_enter_closed_loop(void)
{
    jc_enter_closed_loop(&hfdcan1, YUNTAI_MOTOR_YAW_ID);
    jc_enter_closed_loop(&hfdcan2, YUNTAI_MOTOR_PITCH_ID);
}

static void motor_set_speed_mode(void)
{
    jc_set_control_mode(&hfdcan1, YUNTAI_MOTOR_YAW_ID, JC_MODE_SPEED);
    jc_set_control_mode(&hfdcan2, YUNTAI_MOTOR_PITCH_ID, JC_MODE_SPEED);
}

/* ===================== 偏航电机独立自检 =====================
 * 见文件上方 YAW_BRINGUP_TEST 的说明。
 * 逐个 ID 试：发使能+速度模式(只在本 ID 的第一帧发一次) + 30rpm，
 * 每个 ID 持续 ON_MS 后停 OFF_MS，然后换下一个 ID。
 * 用电机动作本身把"真正的 ID 是几号"报出来。 */
static void yaw_bringup(void)
{
    static uint32_t t_ms    = 0;      /* 本阶段开始时刻 */
    static uint8_t  id      = 1;      /* 当前在试的 ID */
    static uint8_t  enabled = 0;      /* 本 ID 的使能指令发过没有 */
    uint32_t now = HAL_GetTick();

    if ((now - t_ms) < CTRL_PERIOD) return;
    t_ms = now;

    /* 阶段结束 -> 换下一个 ID */
    if ((now - s_state_ms) >= (YAW_BRINGUP_ON_MS + YAW_BRINGUP_OFF_MS))
    {
        s_state_ms = now;
        id++;
        if (id > YAW_BRINGUP_ID_MAX) id = 1;
        enabled = 0;
    }

    /* 先算一下是不是在这一轮的"通电"段里 */
    if ((now - s_state_ms) < YAW_BRINGUP_ON_MS)
    {
        if (!enabled)
        {
            /* ❗使能+控制模式只在每个 ID 的第一帧发，之后不再重复发，
             *   否则会不断复位驱动器内部运动状态、反而看不到动作 */
            jc_enter_closed_loop(&hfdcan1, id);
            jc_set_control_mode(&hfdcan1, id, JC_MODE_SPEED);
            enabled = 1;
        }
        jc_set_speed_rpm_x100(&hfdcan1, id, (int32_t)(YAW_BRINGUP_RPM * 100.0f));
    }
    else
    {
        /* 停顿段：把当前 ID 停住。
         * 这样上电后电机一定是"动 1.5s、停 0.7s"一阵一阵地转，
         * 而且循环总是从 ID=1 开始 ⟹ 上电后第几次动，对应的 ID 就是几。 */
        jc_set_speed_rpm_x100(&hfdcan1, id, 0);
    }
}

void yuntai_init(void)
{
    can_bsp_init();
    mahonySampleFreq = 1000.0f / (float)IMU_PERIOD;

    pitch_base_ok = 0;
    pitch_fb_ms   = 0;
    pitch_fb.pos_deg = 0.0f;

    debug_println("=== YunTai Start ===");
    s_state    = ST_BMI_INIT;
    s_state_ms = HAL_GetTick();
}

void yuntai_control_loop(void)
{
    uint32_t now = HAL_GetTick();

    /* 总线维护：bus-off 自动恢复。必须放在最前面，
     * 自检模式下也要跑，否则一旦 bus-off 连自检都发不出去。 */
    can_bsp_service();

#if YAW_BRINGUP_TEST
    /* 独立自检模式：整个状态机/IMU/PID/俯仰轴全部跳过 */
    yaw_bringup();
    return;
#endif

    switch (s_state)
    {
    /* ---- 1. BMI088 初始化 ---- */
    case ST_BMI_INIT:
        if ((now - s_state_ms) < 200) break;    /* 每 200ms 重试一次 */
        if (BMI088_init() == BMI088_NO_ERROR)
        {
            AHRS_init(imuQuat);
            debug_println("BMI088 OK");
            s_state    = ST_WAIT_MOTOR_BOOT;
            s_state_ms = now;
            s_imu_ms   = 0;
        }
        else
        {
            s_state_ms = now;
        }
        break;

    /* ---- 2. 等驱动器上电自检完成（纯延时，与原版 CALIBRATE_DELAY 相同）----
     * ❗这里【绝不能】再放"要满足某条件才放行"的逻辑。
     *   曾经在这里放陀螺零偏采样（静止求平均），它把启动流程卡死了：
     *     1) 它要求连续 600 个样本都满足"没在动"，任何一个样本超阈值就把
     *        已采样本整批清零重来。若这颗 BMI088 的陀螺零偏本身就超过
     *        静置阈值(0.06 rad/s)，那永远采不满，只能干等到 12s 超时；
     *     2) 更致命的是走不到这一步，后面 ST_ENTER_CLOSED_LOOP 就永远不执行,
     *        电机停在"未使能"状态 —— 表现就是云台一直是软的、能被手掰动。
     *   原版第一次提交的代码没有这一步，直接拿原始陀螺值用，照样能跑。
     *   现在改成和原版一样的纯延时：状态机【无条件推进】，不可能卡住。
     *
     *   这 2 秒是给驱动器上电自检用的，不能省（发太早会收不到指令）。 */
    case ST_WAIT_MOTOR_BOOT:
        if ((now - s_state_ms) < STARTUP_DELAY_MS) break;

        debug_println("startup delay done");
        s_state    = ST_ENTER_CLOSED_LOOP;
        s_state_ms = now;
        s_imu_ms   = now;
        break;

    /* ---- 3. 进入闭环 0x00A2（阶段内重发防丢帧）---- */
    case ST_ENTER_CLOSED_LOOP:
        if ((now - s_imu_ms) >= CMD_RESEND_MS)
        {
            s_imu_ms = now;
            motor_enter_closed_loop();
        }
        if ((now - s_state_ms) >= CMD_STAGE_MS)
        {
            debug_println("motor closed loop");
            s_state    = ST_SET_SPEED_MODE;
            s_state_ms = now;
            s_imu_ms   = now;
        }
        break;

    /* ---- 4. 切速度模式 0x0060=1（阶段内重发防丢帧）---- */
    case ST_SET_SPEED_MODE:
        if ((now - s_imu_ms) >= CMD_RESEND_MS)
        {
            s_imu_ms = now;
            motor_set_speed_mode();
        }
        if ((now - s_state_ms) >= CMD_STAGE_MS)
        {
            debug_println("motor speed mode");
            s_state    = ST_LOCK;
            s_state_ms = now;
            s_imu_ms   = 0;      /* ST_LOCK 要用 s_imu_ms 做 IMU 采样计时 */
        }
        break;

    /* ---- 5. 锁定姿态基准 ---- */
    case ST_LOCK:
        if ((now - s_imu_ms) >= IMU_PERIOD)
        {
            s_imu_ms = now;
            imu_read();
            ahrs_update();
        }

        if ((now - s_state_ms) > LOCK_MS)
        {
            PID_Init(&yaw_pid,   YAW_KP,   YAW_KI,   YAW_KD,   YAW_OUT);
            yaw_pid.integral_limit = YAW_ILIM;
            PID_Init(&pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_OUT);
            pitch_pid.integral_limit = PITCH_ILIM;

            pitch_plat_locked  = imuAngle[INS_PITCH_ADDRESS_OFFSET];
            pitch_motor_locked = 0.0f;
            pitch_base_ok      = 0;
            yaw_target         = imuAngle[INS_YAW_ADDRESS_OFFSET];

            s_ctrl_ms  = now;
            s_debug_ms = now;
            s_state    = ST_RUN;
            debug_println("=== STABILIZATION ACTIVE ===");
        }
        break;

    /* ---- 6. 双轴闭环 ---- */
    case ST_RUN:
        if ((now - s_imu_ms) >= IMU_PERIOD)
        {
            s_imu_ms = now;
            imu_read();
            ahrs_update();
        }

        if ((now - s_ctrl_ms) < CTRL_PERIOD) break;
        s_ctrl_ms = now;
        {
            float dt = (float)CTRL_PERIOD / 1000.0f;

            /* ---------- Yaw：角度闭环 + 角速度前馈（与原版同构 + 补上缺失的 FF）----------
             * 慢速修正靠 PID（把上电时的朝向保持住），
             * 快速响应靠 gyro[2] 前馈（否则指令抬不过静摩擦，看起来"不工作"）。 */
            {
                float yaw_err = angle_diff_rad(yaw_target, imuAngle[INS_YAW_ADDRESS_OFFSET]);
                float yaw_cmd;

                PID_Update(&yaw_pid, yaw_err, 0.0f, dt);

                yaw_cmd = yaw_pid.out
                        + YAW_FF_SIGN * gyro[2] * (YAW_FF_GAIN / 100.0f);

                if (yaw_cmd >  YAW_OUT) yaw_cmd =  YAW_OUT;
                if (yaw_cmd < -YAW_OUT) yaw_cmd = -YAW_OUT;

#if YAW_OPENLOOP_TEST
                /* 开环自检：忽略 PID，直接给水平电机固定转速 */
                (void)yaw_cmd;
                jc_set_speed_rpm_x100(&hfdcan1, YUNTAI_MOTOR_YAW_ID,
                                      (int32_t)(YAW_OPENLOOP_RPM * 100.0f));
#else
                jc_set_speed_rpm_x100(&hfdcan1, YUNTAI_MOTOR_YAW_ID,
                                      (int32_t)(yaw_cmd * 100.0f));
#endif
            }

            /* ---------- Pitch ---------- */
            {
                uint8_t fb_ok = ((now - pitch_fb_ms) < PITCH_FB_TIMEOUT_MS) ? 1 : 0;

                /* 首次拿到有效反馈时建立基准；只建一次，绝不重建 */
                if (fb_ok && !pitch_base_ok)
                {
                    pitch_plat_locked  = imuAngle[INS_PITCH_ADDRESS_OFFSET];
                    pitch_motor_locked = pitch_fb.pos_deg;
                    PID_Reset(&pitch_pid);
                    pitch_base_ok = 1;
                    debug_println("pitch encoder online");
                }

                if (pitch_base_ok && fb_ok)
                {
                    /* 目标电机角 = 上电电机角 - 平台俯仰变化量
                     * 变化量直接取 IMU 绝对俯仰角差值（重力修正，不会漂移） */
                    float dplat  = imuAngle[INS_PITCH_ADDRESS_OFFSET] - pitch_plat_locked;
                    float target = pitch_motor_locked - dplat * RAD2DEG;
                    float err, cmd;

                    if (target > (pitch_motor_locked + PITCH_LIMIT_DEG))
                        target = pitch_motor_locked + PITCH_LIMIT_DEG;
                    if (target < (pitch_motor_locked - PITCH_LIMIT_DEG))
                        target = pitch_motor_locked - PITCH_LIMIT_DEG;

                    err = target - pitch_fb.pos_deg;

                    PID_Update(&pitch_pid, err, 0.0f, dt);

                    cmd = pitch_pid.out
                        + PITCH_FF_SIGN * gyro[1] * (PITCH_FF_GAIN / 100.0f);

                    if (cmd >  PITCH_OUT) cmd =  PITCH_OUT;
                    if (cmd < -PITCH_OUT) cmd = -PITCH_OUT;

#if ENABLE_RUNTIME_DEBUG
                    if ((now - s_debug_ms) >= DEBUG_PERIOD)
                    {
                        char buf[128];
                        int  n;
                        s_debug_ms = now;
                        n = snprintf(buf, sizeof(buf),
                            "yaw=%d pIMU=%d pTgt=%d pEnc=%d pErr=%d cmd=%d gy=%d a=%d\r\n",
                            (int)(imuAngle[INS_YAW_ADDRESS_OFFSET] * 5730.0f),
                            (int)(imuAngle[INS_PITCH_ADDRESS_OFFSET] * 5730.0f),
                            (int)(target * 100.0f),
                            (int)(pitch_fb.pos_deg * 100.0f),
                            (int)(err * 100.0f),
                            (int)(cmd * 100.0f),
                            (int)(gyro[1] * 10000.0f),
                            (int)(sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]) * 100.0f));
                        if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 100);
                    }
#endif
                    jc_set_speed_rpm_x100(&hfdcan2, YUNTAI_MOTOR_PITCH_ID,
                                          (int32_t)(cmd * 100.0f));
                }
                else
                {
                    /* 反馈未就绪的降级：只给角速度前馈，保证不会完全失控 */
                    PID_Reset(&pitch_pid);
                    jc_set_speed_rpm_x100(&hfdcan2, YUNTAI_MOTOR_PITCH_ID,
                        (int32_t)(PITCH_FF_SIGN * gyro[1]
                                  * (PITCH_FF_GAIN / 100.0f) * 100.0f));
                }
            }
        }
        break;

    default:
        break;
    }
}
