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
#define GP_MSG_AIM            0x10u    /* 下：瞄准偏置（100Hz，核心指令） */
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
#define GP_PARAM_NUM             0x0Bu

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
