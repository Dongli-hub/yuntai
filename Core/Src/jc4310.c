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
    if (data[0] != 0x43) return 0;
    raw = (int32_t)(((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                    ((uint32_t)data[6] << 8)  |  (uint32_t)data[7]);
    *pos_deg = (float)raw / 100.0f;
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
