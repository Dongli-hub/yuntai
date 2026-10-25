/**
  ******************************************************************************
  * @file    gimbal_proto.h
  * @brief   云台与机载计算机（地瓜派）之间的串口协议 —— 纯逻辑层，不依赖 HAL
  *
  * 这一层只做三件事：CRC16、帧的分解/组装、各个消息的字节编解码。
  * 之所以单独拆出来，是因为它不含任何硬件调用，可以在 PC 上直接编译运行，
  * 用电赛上位机程序（rdk_aim）产生的真实字节流做回归测试
  * （见 rdk_aim/tests/test_c_proto.py）。协议层的 bug 最难在板子上定位，
  * 所以这一层必须先在 PC 上跑绿。
  *
  * 帧格式（与 rdk_aim/eaim/protocol.py 严格一致）：
  *
  *   偏移  长度  字段        说明
  *    0     1    0xAA        帧头 1
  *    1     1    0x55        帧头 2
  *    2     1    msg_id      消息类型
  *    3     1    seq         序号 0~255
  *    4     1    len         payload 长度 0~64
  *    5    len   payload     负载（多字节量一律小端）
  *   5+len  2    crc16       小端；CRC16/Modbus，覆盖 msg_id..payload
  ******************************************************************************
  */

#ifndef __GIMBAL_PROTO_H__
#define __GIMBAL_PROTO_H__

#include <stdint.h>

/* ---------------- 帧结构参数 ---------------- */
#define GP_SOF1            0xAAu
#define GP_SOF2            0x55u
#define GP_HEAD_LEN        5u          /* AA 55 msg seq len */
#define GP_CRC_LEN         2u
#define GP_MAX_PAYLOAD     64u         /* 本工程最长的是 GIMBAL_STATE(21) 与 TEXT */

/* ---------------- 消息 ID（与上位机 MsgId 一一对应） ---------------- */
/* 2026-10-11 新架构：K230 只上报像素坐标，"像素->偏置"的计算挪到 H723
 * （见 yuntai_task.c 的"视觉外环"）。AIM payload 布局：
 *   [0..1] ut  [2..3] vt  [4..5] us  [6..7] vs  （u16 小端，像素）
 *   [8]    flags  [9] quality
 *   [10..11] kx100 = 像素/度 × 100（i16 小端）—— 可选，见下。
 * ut/vt = 靶心像素，us/vs = 激光光斑像素。
 * 0x16：新 K230 视觉直接发送靶心减激光的像素差（各 int16 像素），
 *       后跟 flags(u8) + quality(u8)，H723 收到后直接进入 AIM。
 * 头部 seq = 视觉帧序号：只有 K230 产生新检测结果时才 +1。
 * 同一视觉帧可以重复发送（喂看门狗），重复包的 seq 不变。
 *
 * ⚠ 为什么要带 K（px/°）：K230 的处理分辨率是可切换的（640x480/800x600），
 *   px/° 随分辨率同比变化（7.8 → 9.75）。外环的 e_deg = e_px / K，K 用错
 *   25% 就等于把外环增益悄悄改了 25%（表现是"比之前更容易来回摆"）。
 *   所以让 K230 每帧带着 K 走，换分辨率不用重烧 H723。
 *   兼容：只有一个 10 字节 payload 的老上位机照样能跑 —— 这时用默认 7.8。
 *         只认 10 字节的老固件遇到 12 字节帧也没事（多出来两个字节被忽略）。 */
#define GP_MSG_AIM            0x10u    /* 下：视觉像素坐标上报（30Hz 保持，核心指令） */
#define GP_AIM_PAYLOAD_LEN    10u      /* AIM payload 长度（老格式：无 K） */
#define GP_AIM_PAYLOAD_LEN_K  12u      /* AIM payload 长度（新格式：带 kx100） */
#define GP_MSG_AIM_ERROR_PX   0x16u    /* 下：直接像素差，err_u/err_v + flags + quality */
#define GP_AIM_ERROR_PAYLOAD_LEN 6u
#define GP_MSG_MODE           0x11u    /* 下：工作模式 */
#define GP_MSG_HEARTBEAT      0x12u    /* 下：心跳 */
#define GP_MSG_SET_ZERO       0x13u    /* 下：把当前朝向记为偏置零点 */
#define GP_MSG_UNWIND         0x14u    /* 下：解绕速度 */
#define GP_MSG_PARAM          0x15u    /* 下：运行时就地调参（不用重新烧写） */
#define GP_MSG_GIMBAL_STATE   0x90u    /* 上：状态遥测 */
#define GP_MSG_ACK            0x91u    /* 上：指令应答 */
#define GP_MSG_VERSION        0x92u    /* 上：版本字符串 */
#define GP_MSG_TEXT           0x93u    /* 上：调试文本（替代原来的串口 printf） */

/* ---------------- 运行时调参（GP_MSG_PARAM） ----------------
 * payload：param_id(u8) + value(i16 小端)，value 统一 = 真实值 × 100
 *          例：KP 改成 12.5  -> 0x05, 00 4E(小端 1250)；前馈符号取反 -> -100
 *
 * 为什么要有这条消息：
 *   俯仰振荡这类问题只能"改一个数、掰一下、看结果"，如果每试一个组合都要
 *   改代码 + 重新烧写，一轮好几分钟，一天也调不出所以然。有了它，现场几秒
 *   就能试完一个组合。
 *
 * 为什么参数只存在 RAM、重新上电就恢复默认：
 *   故意的。避免"上一次调歪的值被记住"，导致下次上电出现莫名其妙的故障；
 *   调好后要把数值写回代码里的默认值，再烧一次固化。
 */
#define GP_PARAM_DUMP            0x00u  /* 下发它 = 让 H723 回一行当前参数 */
#define GP_PARAM_YAW_FF_SIGN     0x01u  /* ±1，前馈符号 */
#define GP_PARAM_YAW_FF_GAIN     0x02u  /* rpm/(rad/s) */
#define GP_PARAM_PITCH_FF_SIGN   0x03u  /* ±1，前馈符号（俯仰振荡第一嫌疑人） */
#define GP_PARAM_PITCH_FF_GAIN   0x04u  /* rpm/(rad/s)，0 = 关掉前馈 */
#define GP_PARAM_PITCH_KP        0x05u  /* rpm/deg */
#define GP_PARAM_PITCH_KI        0x06u  /* rpm/(deg*s) */
#define GP_PARAM_PITCH_KD        0x07u  /* rpm/(deg/s) */
#define GP_PARAM_PITCH_ILIM      0x08u  /* 积分限幅 */
#define GP_PARAM_PITCH_OUT_RPM   0x09u  /* 输出限幅 */
#define GP_PARAM_PITCH_PLAT_SIGN 0x0Au  /* 平台俯仰补偿：0=关，+1/-1=开并定方向 */
/* 0x0B：偏航静摩擦补偿转速（rpm）。小指令推不动轴时，补一个"最小有效转速"；
 *       0 = 关掉补偿。现场用 tune_pitch.py 的 PARAM 在线加减，找到"刚好推得动
 *       又不来回抖"的值。 */
#define GP_PARAM_YAW_STICTION_RPM 0x0Bu
/* 0x0C：偏航积分限幅。ki*该值 = 积分单独能给出的最大 rpm。
 *       必须小于"能推动轴的最小转速"（约 10rpm），否则轴会被积分推着慢慢转。 */
#define GP_PARAM_YAW_ILIM        0x0Cu
/* 0x0D：视觉外环的 K（像素/度）。默认 7.8（= 640x480 档标定值）。
 *       正常不用手动发 —— K230 每帧的 AIM 里带着 K（见 GP_AIM_PAYLOAD_LEN_K）。
 *       留这个入口是为了排查："是不是 K 用错了" 时能在现场直接覆盖它试一把。 */
#define GP_PARAM_VIS_K           0x0Du
/* 0x0E：视觉外环偏航 KP（1/s）。每个视觉帧最多修正 65% 的残差，
 *       再受单帧 8° 限幅；默认 6.0。0x0F 为旧 KD，已不再使用。 */
#define GP_PARAM_VIS_KP_YAW      0x0Eu
#define GP_PARAM_VIS_KD_YAW      0x0Fu
/* 0x10：偏航输出限幅（rpm）。它同时管两件事：①前馈能在多快的底盘转速下还
 * 保持线性（限幅/前馈增益 = 上限转速）；②跑飞保护的上限。默认 80（≈480°/s）。*/
#define GP_PARAM_YAW_OUT_RPM     0x10u
/* 0x11：偏航"平台速率阻尼"（rpm per rad/s）。**这是压顿挫/抖动最直接的旋钮**：
 *   它按"平台实测转速 - 视觉要求转速"出力（抗扰 + 稳速）。参考代码的角度环
 *   就是速率项为主（KD/KP ≈ 22:1），我们默认 70（≈7 倍理论抵消量 9.55）。
 *   现场从 40 一档档往上加，看"顿挫"最小、又不起高频抖。*/
#define GP_PARAM_YAW_RATE_DAMP   0x11u
/* 0x12：偏航"底盘转速前馈"（rpm per rad/s）。理论值 -9.55（见 yuntai_task.c
 *   里 YAW_BASE_FF_RPM_PER_RADS 的推导）。**符号取决于编码器/陀螺方向约定**，
 *   现场先各试一次：-955 与 +955，哪个"平移台子时平台更稳"就用哪个；
 *   0 = 关（默认）。确认后写回默认值。*/
#define GP_PARAM_YAW_BASE_FF     0x12u
/* 0x13：电机"类速度模式"的控制模式值（写驱动器寄存器 0x0060）。
 *   手册：0x01 = 速度模式（明写）；0x05 = 低速大扭矩模式。**
 *   ⚠ 2026-10-12：默认已回到 0x01** —— 上一版默认 0x05 时现场出现"转底盘不顶、
 *   手晃有反抗、猛地转到别处定住"，而手册给 0x05 单列了寄存器 0x0027"设置低速"，
 *   很可能它跟的不是我们一直在写的 0x0021。要用 0x05 必须先看电机响应测试。*/
#define GP_PARAM_SPEED_MODE      0x13u
/* 0x14：偏航**力矩模式**开关（0=关/速度类模式，1=开）。
 *   开=固件自己跑速率环（反馈 IMU 陀螺，输出电流->力矩，寄存器 0x0020），
 *   即参考代码的结构；增益见 0x15/0x16/0x17。⚠ 台架上先小电流试。*/
#define GP_PARAM_YAW_TORQUE      0x14u
/* 0x15：力矩模式速率环 P（mA/rpm ×100）。参考代码 1.3e-2 A/rpm = 13 mA/rpm */
#define GP_PARAM_RATE_KP         0x15u
/* 0x16：力矩模式速率环 I（mA/(rpm·s) ×100）。参考 3.9e-4 A = 0.39 mA */
#define GP_PARAM_RATE_KI         0x16u
/* 0x17：力矩模式电流限幅（A ×100）。参考 GIMBAL_MAX_CURRENT = 1.65 A */
#define GP_PARAM_RATE_ILIM       0x17u
/* 0x18/0x19/0x1A：**偏航角度环** KP/KI/KD（rpm/rad、rpm/(rad·s)、rpm/(rad/s)）。
 *   参考代码的角度环是"软位置 + 强速率"（kp=5 对 kd=110），我们原来是
 *   KP=200 的硬位置 → 误差攒大了会"猛甩"。默认已改 60/20/10，现场一档档试。*/
#define GP_PARAM_YAW_KP          0x18u
#define GP_PARAM_YAW_KI          0x19u
#define GP_PARAM_YAW_KD          0x1Au
/* 0x1B：**电机响应测试**（台架用，K230 只收日志）。
 *   value = 100：速度脚本（约 10s）——
 *      1s 停 → 1.5s +20rpm → 1.5s −20rpm → 1.5s +8 → 1.5s −8 → 1.5s +3 → 1.5s −3
 *      H723 用 [MOTO] V 行打印"每条指令 vs 驱动器自报转速 vs 位置"。
 *      目的：一次把"驱动器到底跟不跟指令、低转速有没有死区、反馈对不对"问清楚。
 *   value = 200：力矩脚本（约 7s，自动切力矩模式、结束自动切回）——
 *      1s 停 → 1.2s +500mA → 1.2s −500mA → 1.2s +1500mA → 1.2s −1500mA → 停
 *      H723 用 [MOTO] T 行打印"注入电流 vs 平台陀螺(°/s) vs 位置"。
 *      目的：定死"力矩符号对不对"+"多大电流推得动平台"（0.5A≈0.02Nm、
 *      1.5A≈0.06Nm，按力矩常数 0.04Nm/A 估）。*/
#define GP_PARAM_MOTOR_TEST      0x1Bu
/* 0x1C：**控制/指令周期**（ms × 100）。默认 200 = 2ms = **500Hz**（bf3bf46
 *   "不脱靶追随"那一版跑的就是 500Hz，是唯一被现场验证过的节拍）。
 *       (0x1C, 100) = 1ms/1kHz（比 500Hz 快一倍；但驱动器每条指令都要回一帧，
 *                      1kHz 时现场连续出现"卡角度档位、猛地转过去"，所以默认
 *                      改回 500Hz，这个档位留着做 A/B 对比）
 *       (0x1C, 200) = 2ms/500Hz（默认）
 *       (0x1C, 500) = 5ms/200Hz（更保守） */
#define GP_PARAM_CTRL_PERIOD     0x1Cu
/* 0x1D：力矩模式"**脱困补偿**"电流（A × 100）。
 *   偏航带滑环、静摩擦大（43/44 号日志：±0.08Nm 脉冲推不动），线性环路要产生
 *   0.2Nm 就得攒十几度误差。这一项在"误差>死区 且 平台几乎没动"时，把输出至少
 *   垫到 ±该电流去脱困，一动起来立刻撤掉、交回线性环路精修。
 *   ⚠ 2026-10-14 十三版：**默认 250 = 2.5A**（原 100 = 1.0A）。依据 38 号日志：
 *   "那条远离靶面的边"是**卡滞→崩开**（电机停在滑环摩擦最大的角度区，`ym=1007.5°`，
 *   补偿 1.0A 顶不开 → 偏置卷到 −26° → 一帧甩 7.2°、5~8Hz 振铃半秒）。
 *   2.5A 只给"最坏角度区"留余量；好角度区里它涨到 ~0.5A 轴就动了、随即以 30A/s 收回，
 *   不会过推。还卡就 300/400（上限受 0x17 限幅），反而过冲就往 150 收。0 = 关。*/
#define GP_PARAM_TQ_STICTION     0x1Du
/* 0x1E：**力矩指令下发周期**（ms × 100）。
 *   ⚠ 2026-10-13：用户上位机实测"0.02Nm 就能让偏航轴动" —— 说明轴很轻，
 *   之前"推不动"是**力矩没落到电机上**：驱动器参数里"力矩爬升率 5.00Nm/s"，
 *   而我们每 2ms 重写一次 0x0020，爬升斜坡被反复重置，力矩永远爬不上去
 *   （2ms 只允许涨 0.01Nm）。上位机是单条指令，所以能动。
 *   默认 2000 = 20ms（50Hz）；嫌迟钝往 1000/500 调；0 = 关掉节流（旧行为）。
 *   调试时可以 (0x1E, 5000) 拉长到 50ms 对比，看是不是这个原因。*/
#define GP_PARAM_TQ_WR_MS        0x1Eu
/* 0x1F：俯仰视觉外环 KP（1/s），默认 6.0。0x20 为旧 KD，已不再使用。 */
#define GP_PARAM_VIS_KP_PITCH    0x1Fu
#define GP_PARAM_VIS_KD_PITCH    0x20u
/* 偏航力矩环"积分项最大贡献"（mA）。2026-10-13 十版新增：
 * 参考代码角度环的 sum_error 限幅 ±1.8、ki=0.1 —— 折算成电流就是 0.18A。
 * 我们把它做成在线旋钮，现场在"参考档 0.18A"和"我们这轴的摩擦档 0.5~0.6A"
 * 之间一键切换：value = mA（1800 = 1.8A）。
 *   18  = 0.18A（参考原值，默认）
 *   60  = 0.6A （轴"慢慢漂/推不动"就往这加；我们静摩擦实测 ~0.5A）*/
#define GP_PARAM_TQ_I_LIM        0x21u
/* 0x22 / 0x23：旧"速度记忆"参数，保留编号，当前固件忽略。 */
#define GP_PARAM_VIS_KII_YAW     0x22u
#define GP_PARAM_VIS_KII_PITCH   0x23u
/* 0x24：旧"偏置速率限幅"，保留编号，当前固件忽略。 */
#define GP_PARAM_VIS_RATE_LIM    0x24u
/* 0x25：旧"方位变化率前馈"增益，保留编号，当前固件忽略。 */
#define GP_PARAM_VIS_FF_GAIN     0x25u
/* 0x26：旧"方位变化率前馈"符号，保留编号，当前固件忽略。 */
#define GP_PARAM_VIS_FF_SIGN     0x26u
/* 0x27：偏航偏置绝对限幅（°，编码 = 真实值×100）。默认 190，
 *       可调范围 40~350。 */
#define GP_PARAM_VIS_YAW_LIM     0x27u
#define GP_PARAM_NUM             0x28u

/* ---------------- 工作模式（GP_MSG_MODE） ---------------- */
#define GP_MODE_IDLE       0u          /* 不使能、不控制（电机自由） */
#define GP_MODE_STAB       1u          /* 只做稳定、忽略偏置、关激光（安全模式） */
#define GP_MODE_AIM        2u          /* 稳定 + 应用瞄准偏置（正常工作模式） */
#define GP_MODE_UNWIND     3u          /* 解绕：相对偏航角转回 0，激光必须关闭 */
#define GP_MODE_ESTOP      4u          /* 急停：两轴指令给 0 */

/* ---------------- AIM 的 flags 位 ---------------- */
#define GP_AIM_LASER_ON   0x01u
#define GP_AIM_VALID      0x02u        /* 上位机本帧视觉有效 */
#define GP_AIM_BOOST      0x04u        /* 拐角加强中 */
#define GP_AIM_DRAWING    0x08u        /* 正在画圆 */
#define GP_AIM_LOCKED     0x10u

/* ---------------- GIMBAL_STATE 的 flags 位 ---------------- */
#define GP_ST_CLOSED_LOOP 0x01u
#define GP_ST_SPEED_MODE  0x02u
#define GP_ST_ENCODER_OK  0x04u        /* 俯仰编码器反馈正常（位置环在跑） */
#define GP_ST_LASER_ON    0x08u
#define GP_ST_READY       0x10u        /* 启动流程走完、可以接受偏置了 */

/* 关于 GP_ST_READY：
 * 上位机判断"云台起来了没有"**必须用这一位**，不要用 state 数值去比大小。
 * 因为 state 是固件内部枚举，改一次状态机就可能变（这个坑踩过一次：
 * 状态机去掉一个 INIT 状态后 RUNNING 从 6 变成 5，上位机就一直等不到）。
 */

/* GIMBAL_STATE 的 payload 长度：B B h h h h h h h B I = 21 字节 */
#define GP_GIMBAL_STATE_LEN  21u

/* --------------------------------------------------------------------------
 * 流式解析器
 *
 * 用状态机逐字节推进，不需要整帧缓冲，天然抗粘包/断包/干扰：
 * 任何一个字节对不上就回到"找帧头"，绝不卡死。
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  state;
    uint8_t  msg_id;
    uint8_t  seq;
    uint8_t  len;
    uint8_t  idx;
    uint8_t  payload[GP_MAX_PAYLOAD];
    uint16_t crc_calc;
    uint16_t crc_rx;
    /* 统计量：现场排障时非常有用（crc_err 一直涨 = 波特率不对/线太长/没共地） */
    uint32_t frames;
    uint32_t crc_err;
    uint32_t bad_len;
    uint32_t resync;
} gp_parser_t;

/* ---------------- 基础工具 ---------------- */
uint16_t gp_crc16(const uint8_t *data, uint16_t len);
uint16_t gp_crc16_update(uint16_t crc, uint8_t byte);

int16_t  gp_get_i16(const uint8_t *p);
uint16_t gp_get_u16(const uint8_t *p);
uint32_t gp_get_u32(const uint8_t *p);
void     gp_put_i16(uint8_t *p, int16_t v);
void     gp_put_u16(uint8_t *p, uint16_t v);
void     gp_put_u32(uint8_t *p, uint32_t v);

/** 浮点角度 -> int16(度*100)，自动限幅，与上位机 scale_deg() 一致 */
int16_t  gp_deg_to_x100(float deg);

/* ---------------- 解析 ---------------- */
void    gp_parser_init(gp_parser_t *p);

/**
 * @brief  喂一个字节
 * @retval 1 = 刚刚收齐一整帧（结果在 p->msg_id/seq/len/payload 里）
 */
uint8_t gp_parser_feed(gp_parser_t *p, uint8_t byte);

/* ---------------- 组帧 ---------------- */
/**
 * @brief  组装一整帧（帧头 + 头 + payload + CRC）
 * @retval 写入 out 的字节数；0 表示失败（payload 太长或 out 不够大）
 */
uint16_t gp_build(uint8_t msg_id, uint8_t seq, const uint8_t *payload,
                  uint8_t len, uint8_t *out, uint16_t out_cap);

/** 组装 GIMBAL_STATE 的 payload（21 字节），返回 payload 长度 */
uint8_t gp_make_gimbal_state(
    uint8_t *out,
    uint8_t  state, uint8_t fault,
    float yaw_deg, float pitch_deg, float roll_deg,
    float yaw_motor_deg, float pitch_motor_deg,
    float gyro_y_dps, float gyro_z_dps,
    uint8_t flags, uint32_t uptime_ms);

/** 组装 ACK 的 payload（2 字节） */
uint8_t gp_make_ack(uint8_t *out, uint8_t ack_msg_id, uint8_t code);

#endif /* __GIMBAL_PROTO_H__ */
