#ifndef _JC4310__H__
#define _JC4310__H__
#include "main.h"
#include "can_bsp.h"

// 控制模式定义
#define JC_MODE_SPEED       0x01    // 速度模式
#define JC_MODE_POS_TRAP    0x02    // 位置梯形模式
#define JC_MODE_POS_DIRECT  0x04    // 位置直通模式

void jc_calibrate_motor(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void jc_set_speed_rpm_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t speed_rpm);
void jc_enter_closed_loop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void jc_set_control_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t mode);
void jc_set_abs_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t position_x100);
void jc_set_rel_position_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t delta_position_x100);
void jc_set_abs_angle_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t angle_x100);

/* ---- 反馈解析（俯仰位置环的反馈源） ---- */

/** 主动读实时位置：命令字 0x43，寄存器 0x0008 */
void jc_read_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/**
 * @brief  解析"主动读位置"的回复（命令字 0x43）
 *         data[0]=0x43，data[4..7] = 有符号 32 位大端，÷100 = 度
 */
uint8_t jc_parse_position_reply(const uint8_t *data, uint8_t len, float *pos_deg);

/**
 * @brief  解析"速度指令回复"（命令字 0x2A）里自带的位置
 *         data[1..3] = 有符号 24 位大端，÷100 = 度
 *         data[4..5] = 速度原始值，data[6..7] = 电流原始值（单位未标定，仅诊断）
 *
 * 优先用它：驱动器对每条速度指令都会回这一帧，
 * 位置反馈是"免费"的，不额外占 CAN 带宽。
 */
uint8_t jc_parse_speed_reply(const uint8_t *data, uint8_t len, float *pos_deg,
                             int16_t *speed_raw, int16_t *current_raw);

#endif // !_JC4310__H__
