#ifndef _JC4310__H__
#define _JC4310__H__
#include "main.h"
#include "can_bsp.h"

// 控制模式定义（寄存器 0x0060）
#define JC_MODE_TORQUE      0x00    // 力矩模式
#define JC_MODE_SPEED       0x01    // 速度模式
#define JC_MODE_POS_TRAP    0x02    // 位置梯形轨迹
#define JC_MODE_POS_FILTER  0x03    // 位置滤波模式
#define JC_MODE_POS_DIRECT  0x04    // 位置直通模式
#define JC_MODE_LOW_SPEED   0x05    // 低速大扭模式

// 命令字（见《JC系列CAN通信说明》二、指令格式）
#define JC_CMD_READ_2REG    0x43    // 读2个寄存器(4字节)
#define JC_CMD_READ_1REG    0x4B    // 读1个寄存器(2字节)
#define JC_CMD_WRITE_4BYTE  0x23    // 写2个寄存器(4字节)
#define JC_CMD_WRITE_2BYTE  0x2B    // 写1个寄存器(2字节)
#define JC_REPLY_WRITE_CMD  0x2A    // 写指令的回复命令字

// 寄存器地址
#define JC_REG_REAL_SPEED   0x0006  // 实时速度(32bit, ×100)
#define JC_REG_REAL_POS     0x0008  // 实时位置(32bit, ×100)
#define JC_REG_CTRL_MODE    0x0060  // 控制模式
#define JC_REG_SET_SPEED    0x0021  // 设定速度(32bit, ×100)
#define JC_REG_SET_ABS_POS  0x0023  // 设定绝对位置(32bit, ×100)
#define JC_REG_SET_REL_POS  0x0025  // 设定相对位置(32bit, ×100)
#define JC_REG_IDLE         0x00A0  // 空闲状态
#define JC_REG_CALIBRATE    0x00A1  // 校准电机
#define JC_REG_CLOSED_LOOP  0x00A2  // 进入闭环
#define JC_REG_SET_ORIGIN   0x00A6  // 设置原点

/* 驱动器回复(命令字 0x2A)携带的反馈信息
 * 帧格式: 2A | 位置H 位置M 位置L | 速度H 速度L | 电流H 电流L
 *   位置: 有符号24位大端, ×100, 范围 ±83886.08° (±233圈)
 *   速度: 有符号16位大端, 单位 rpm
 *   电流: 有符号16位大端, ×100 (A)                                        */
typedef struct
{
	float   pos_deg;       // 电机轴绝对角度(deg)
	int16_t speed_rpm;     // 实时速度(rpm)
	int16_t current_x100;  // 相电流 ×100 (A)
	uint8_t valid;         // 1=解析成功
} JcFeedback_t;

void jc_set_speed_rpm_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t speed_rpm);
void jc_enter_closed_loop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void jc_set_control_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t mode);

/* 主动读取电机轴角度（发 0x43 读寄存器 0x0008，回复命令字为 0x43） */
void jc_read_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/* 解析驱动器回复帧；返回 1 表示解析成功 */
uint8_t jc_parse_feedback(const uint8_t *data, uint8_t len, JcFeedback_t *fb);

/* 解析主动读位置指令的回复(命令字 0x43, 寄存器 0x0008, 有符号32位大端, ×100)
 * 返回 1 表示解析成功，角度写入 *pos_deg */
uint8_t jc_parse_position_reply(const uint8_t *data, uint8_t len, float *pos_deg);

#endif // !_JC4310__H__
