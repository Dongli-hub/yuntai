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
