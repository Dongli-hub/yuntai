#include "jc4310.h"

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
 * @brief:     主动读取电机轴角度
 * @param:     hfdcan: FDCAN句柄
 * @param:     motor_id: 电机ID
 * @details:   发送读指令：43 00 08 00 00 00 00 00 （读寄存器0x0008，2个寄存器）
 *             驱动器回复 ID=0x580+ID，数据 43 00 08 00 <4字节大端, ×100>
 * @retval:    无
 */
void jc_read_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    if (motor_id == 0 || motor_id > 127) return;

    uint8_t can_data[8];
    can_data[0] = JC_CMD_READ_2REG;          // 读2个寄存器(4字节)
    can_data[1] = (uint8_t)(JC_REG_REAL_POS >> 8);   // 寄存器高字节 0x00
    can_data[2] = (uint8_t)(JC_REG_REAL_POS & 0xFF); // 寄存器低字节 0x08
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;

    fdcanx_send_data(hfdcan, 0x600 + motor_id, can_data, 8);
}

/**
 * @brief:     解析驱动器对写指令的回复帧
 * @param:     data: 收到的数据域
 * @param:     len:  数据长度
 * @param:     fb:   解析结果
 * @details:   帧格式：2A | 位置H 位置M 位置L | 速度H 速度L | 电流H 电流L
 *             位置占3字节有符号数，×100，正负233圈内
 * @retval:    1=解析成功
 */
uint8_t jc_parse_feedback(const uint8_t *data, uint8_t len, JcFeedback_t *fb)
{
    int32_t pos;

    if (data == NULL || fb == NULL) return 0;
    if (len < 8 || data[0] != JC_REPLY_WRITE_CMD) return 0;

    /* 24位大端有符号数，并做符号扩展 */
    pos = ((int32_t)data[1] << 16) | ((int32_t)data[2] << 8) | (int32_t)data[3];
    if (pos & 0x00800000) pos |= (int32_t)0xFF000000;

    fb->pos_deg      = (float)pos / 100.0f;
    fb->speed_rpm    = (int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5]);
    fb->current_x100 = (int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7]);
    fb->valid        = 1;
    return 1;
}

/**
 * @brief:     解析“主动读位置”指令的回复
 * @details:   帧格式：43 | 00 | 08 | 00 | <4字节大端有符号, ×100>
 * @retval:    1=解析成功
 */
uint8_t jc_parse_position_reply(const uint8_t *data, uint8_t len, float *pos_deg)
{
    int32_t pos;

    if (data == NULL || pos_deg == NULL) return 0;
    if (len < 8) return 0;
    if (data[0] != JC_CMD_READ_2REG) return 0;
    if (data[2] != (uint8_t)(JC_REG_REAL_POS & 0xFF)) return 0;   /* 寄存器低字节应为 0x08 */

    pos = ((int32_t)data[4] << 24) | ((int32_t)data[5] << 16)
        | ((int32_t)data[6] << 8)  |  (int32_t)data[7];

    *pos_deg = (float)pos / 100.0f;
    return 1;
}
