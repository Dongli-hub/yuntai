#include "jc4310.h"

/**
 * @brief:     电机校准
 * @param:     hfdcan: FDCAN句柄
 * @param:     motor_id: 电机ID (1~127)
 */
void jc_calibrate_motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    if (motor_id == 0 || motor_id > 127) return;
    uint8_t can_data[8] = {0x2B, 0x00, 0xA1, 0x00, 0x00, 0x01, 0x00, 0x00};
    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

/**
 * @brief:     设置电机速度（单位：rpm*100，例：100.00 rpm -> 10000）
 * @param:     hfdcan: FDCAN句柄
 * @param:     motor_id: 电机ID
 * @param:     speed_rpm: 速度值（x100）
 */
void jc_set_speed_rpm_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t speed_rpm)
{
    if (motor_id == 0 || motor_id > 127) return;

    int32_t speed_scaled = speed_rpm;
    uint8_t can_data[8];

    can_data[0] = 0x23;           // 命令字：写32位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x0021）
    can_data[2] = 0x21;           // 寄存器低字节（速度指令）
    can_data[3] = 0x00;

    can_data[4] = (uint8_t)((speed_scaled >> 24) & 0xFF);
    can_data[5] = (uint8_t)((speed_scaled >> 16) & 0xFF);
    can_data[6] = (uint8_t)((speed_scaled >>  8) & 0xFF);
    can_data[7] = (uint8_t)( speed_scaled        & 0xFF);

    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

/**
 * @brief:     设置扭矩（力矩模式）。单位：Nm × 100（例 0.2Nm -> 20）
 * 依据《JC系列CAN通信说明》5.8：命令字 0x2B，寄存器 0x0020
 */
void jc_set_torque_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                        int16_t nm_x100)
{
    if (motor_id == 0 || motor_id > 127) return;

    uint8_t can_data[8];
    can_data[0] = 0x2B;           // 命令字：写 16 位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x0020）
    can_data[2] = 0x20;           // 寄存器低字节（扭矩指令）
    can_data[3] = 0x00;
    can_data[4] = (uint8_t)(((uint16_t)nm_x100 >> 8) & 0xFF);
    can_data[5] = (uint8_t)((uint16_t)nm_x100 & 0xFF);
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

/**
 * @brief:     进入闭环控制
 * @param:     hfdcan: FDCAN句柄
 * @param:     motor_id: 电机ID
 */
void jc_enter_closed_loop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    if (motor_id == 0 || motor_id > 127) return;

    uint8_t can_data[8];
    can_data[0] = 0x2B;           // 命令字：写32位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x00A2）
    can_data[2] = 0xA2;           // 寄存器低字节
    can_data[3] = 0x00;

    can_data[4] = 0x00;
    can_data[5] = 0x01;           // 开启闭环
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

/**
 * @brief:     设置控制模式
 * @param:     hfdcan: FDCAN句柄
 * @param:     motor_id: 电机ID
 * @param:     mode: 控制模式 (0x01=速度, 0x02=位置梯形, 0x04=位置直通)
 */
void jc_set_control_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t mode)
{
    if (motor_id == 0 || motor_id > 127) return;

    uint8_t can_data[8];
    can_data[0] = 0x2B;           // 命令字：写32位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x0060）
    can_data[2] = 0x60;           // 寄存器低字节（控制模式）
    can_data[3] = 0x00;

    can_data[4] = 0x00;
    can_data[5] = mode;           // 控制模式
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

/**
 * @brief:     设置绝对位置（单位：0.01deg，例：360deg -> 36000）
 * @param:     hfdcan: FDCAN句柄
 * @param:     motor_id: 电机ID
 * @param:     position_x100: 位置值（x100）
 */
void jc_set_abs_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t position_x100)
{
    if (motor_id == 0 || motor_id > 127) return;

    uint8_t can_data[8];
    can_data[0] = 0x23;           // 命令字：写32位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x0023）
    can_data[2] = 0x23;           // 寄存器低字节（绝对位置）
    can_data[3] = 0x00;

    can_data[4] = (uint8_t)((position_x100 >> 24) & 0xFF);
    can_data[5] = (uint8_t)((position_x100 >> 16) & 0xFF);
    can_data[6] = (uint8_t)((position_x100 >>  8) & 0xFF);
    can_data[7] = (uint8_t)( position_x100        & 0xFF);

    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

/**
 * @brief:     设置相对位置（单位：0.01deg）
 * @param:     hfdcan: FDCAN句柄
 * @param:     motor_id: 电机ID
 * @param:     delta_position_x100: 位置增量（x100）
 */
void jc_set_rel_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t delta_position_x100)
{
    if (motor_id == 0 || motor_id > 127) return;

    uint8_t can_data[8];
    can_data[0] = 0x23;           // 命令字：写32位数据
    can_data[1] = 0x00;           // 寄存器高字节（0x0025）
    can_data[2] = 0x25;           // 寄存器低字节（相对位置）
    can_data[3] = 0x00;

    can_data[4] = (uint8_t)((delta_position_x100 >> 24) & 0xFF);
    can_data[5] = (uint8_t)((delta_position_x100 >> 16) & 0xFF);
    can_data[6] = (uint8_t)((delta_position_x100 >>  8) & 0xFF);
    can_data[7] = (uint8_t)( delta_position_x100        & 0xFF);

    uint32_t can_id = 0x600 + motor_id;
    fdcanx_send_data(hfdcan, can_id, can_data, 8);
}

/**
 * @brief:     设置绝对角度（兼容旧接口，等同于设置绝对位置）
 */
void jc_set_abs_angle_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t angle_x100)
{
    jc_set_abs_position_x100(hfdcan, motor_id, angle_x100);
}

/* ==========================================================================
 * 反馈解析
 * ========================================================================== */

void jc_read_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t can_data[8];
    if (motor_id == 0 || motor_id > 127) return;

    can_data[0] = 0x43;           /* 命令字：读参数 */
    can_data[1] = 0x00;           /* 寄存器 0x0008 = 实时位置 */
    can_data[2] = 0x08;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    fdcanx_send_data(hfdcan, 0x600 + motor_id, can_data, 8);
}

uint8_t jc_parse_position_reply(const uint8_t *data, uint8_t len, float *pos_deg)
{
    int32_t raw;
    if (data == 0 || pos_deg == 0 || len < 8u) return 0;
    /* ⚠ 2026-10-12：这里原来只查 data[0]==0x43，于是**任何** 0x43 回复（比如
     * 读错误码 0x0C、温度 0x0B……）都会被当成"位置"灌进反馈里 —— 位置会突然
     * 跳一个垃圾值。必须连寄存器号一起校验。*/
    if (data[0] != 0x43) return 0;
    if (data[2] != 0x08) return 0;      /* 寄存器 0x0008 = 实时位置 */
    raw = (int32_t)(((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                    ((uint32_t)data[6] << 8)  |  (uint32_t)data[7]);
    *pos_deg = (float)raw / 100.0f;
    return 1;
}

/* ---- 读驱动器错误码（寄存器 0x000C，命令字 0x43）----
 * 依据《JC系列CAN通信说明》5.7：回复 "43 00 0C 00 xx xx xx xx"，
 * data[4..7] = 32 位故障码（示例里 0x40 = 编码器 SPI 通信故障）。
 * 为什么要读它：现场出现过"指令给着、电机就是不动、姿态卡在几个角度" ——
 * 如果这时驱动器其实已经报了故障/保护，错误码会直接告诉我们原因。*/
void jc_read_error(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t can_data[8];
    if (motor_id == 0 || motor_id > 127) return;

    can_data[0] = 0x43;
    can_data[1] = 0x00;
    can_data[2] = 0x0C;           /* 寄存器 0x000C = 错误信息 */
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    fdcanx_send_data(hfdcan, 0x600 + motor_id, can_data, 8);
}

uint8_t jc_parse_error_reply(const uint8_t *data, uint8_t len, uint32_t *err_code)
{
    if (data == 0 || err_code == 0 || len < 8u) return 0;
    if (data[0] != 0x43) return 0;
    if (data[2] != 0x0C) return 0;
    *err_code = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                ((uint32_t)data[6] << 8)  |  (uint32_t)data[7];
    return 1;
}

/* ---- 读控制模式（寄存器 0x0060，命令字 0x43）----
 * 为什么必须读它：驱动器**切换后的控制模式不保存，重启就回到"位置梯形轨迹"**。
 * 一旦它在运行中自己重启（比如被大摆动/掉电搞的），我们发速度指令它根本不执行，
 * 反而"锁在当前位置"——现场表现就是"转底盘顶不住、手一推一跳一跳、指令给了
 * 电机自报转速却是 0"。所以每 2 秒回读一次，发现不对立刻重写。*/
void jc_read_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t can_data[8];
    if (motor_id == 0 || motor_id > 127) return;

    /* ⚠ 命令字必须是 0x4B（读 16 位）而不是 0x43（读 32 位）——
     *   手册里 0x4B 用于电压/电流/温度这类 16 位量，0x43 用于速度/位置/故障码
     *   这类 32 位量（见 5.1~5.7）。用错命令字拿到的"模式"是假值。*/
    can_data[0] = 0x4B;
    can_data[1] = 0x00;
    can_data[2] = 0x60;           /* 寄存器 0x0060 = 控制模式 */
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    fdcanx_send_data(hfdcan, 0x600 + motor_id, can_data, 8);
}

uint8_t jc_parse_mode_reply(const uint8_t *data, uint8_t len, uint8_t *mode)
{
    if (data == 0 || mode == 0 || len < 8u) return 0;
    if (data[0] != 0x4B) return 0;
    if (data[2] != 0x60) return 0;
    /* 与写入格式对称：data[4] 高字节、data[5] 低字节 */
    *mode = data[5];
    return 1;
}

/* ---- 读"闭环使能"状态（寄存器 0x00A2，命令字 0x43）----
 * 0xA2 是"进入闭环"写的那一位（写 1 = 进闭环并锁定当前位置）。
 * 为什么要读它：**模式（0x0060）和"到底有没有在闭环驱动"是两码事**。
 * 现场 40/41 号日志里，两个驱动器都报 mode=1、err=0，但偏航自报转速恒 0、
 * 电流只有 0.05~0.9A、平台被底盘原样带走、手推一跳一跳（正是 7 对极电机的
 * 齿槽位置，25.7°=360/14 一档）—— 这几乎就是"电机根本没被驱动"的样子。*/
void jc_read_loop_state(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t can_data[8];
    if (motor_id == 0 || motor_id > 127) return;

    can_data[0] = 0x4B;           /* 16 位读（同 jc_read_mode 说明）*/
    can_data[1] = 0x00;
    can_data[2] = 0xA2;           /* 寄存器 0x00A2 = 闭环使能 */
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    fdcanx_send_data(hfdcan, 0x600 + motor_id, can_data, 8);
}

uint8_t jc_parse_loop_state_reply(const uint8_t *data, uint8_t len, uint8_t *on)
{
    if (data == 0 || on == 0 || len < 8u) return 0;
    if (data[0] != 0x4B) return 0;
    if (data[2] != 0xA2) return 0;
    *on = data[5];
    return 1;
}

uint8_t jc_parse_speed_reply(const uint8_t *data, uint8_t len, float *pos_deg,
                             int16_t *speed_raw, int16_t *current_raw)
{
    int32_t raw24;
    if (data == 0 || len < 8u) return 0;
    if (data[0] != 0x2A) return 0;

    /* 有符号 24 位：先左对齐到 32 位再算术右移 8 位，符号位才能正确延伸 */
    raw24  = (int32_t)(((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                       ((uint32_t)data[3] << 8));
    raw24 >>= 8;
    if (pos_deg != 0)
    {
        *pos_deg = (float)raw24 / 100.0f;
    }
    if (speed_raw != 0)
    {
        *speed_raw = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    }
    if (current_raw != 0)
    {
        *current_raw = (int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7]);
    }
    return 1;
}
