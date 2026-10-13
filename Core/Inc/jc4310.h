#ifndef _JC4310__H__
#define _JC4310__H__
#include "main.h"
#include "can_bsp.h"

// 控制模式定义
/* ---- 控制模式（写寄存器 0x0060）----
 * 依据《JC4010和JC4310快速入门》第六节"控制设置"：
 *   控制模式：力矩、速度、位置梯形轨迹、位置平滑滤波、位置直通模式、低速大扭模式
 * 手册给的 CAN 例子："10、切换为速度模式：01 06 00 60 00 01" -> 速度 = 0x01 ✓
 * 其余按上位机下拉顺序推：0x00 力矩 / 0x02 位置梯形 / 0x03 位置平滑 /
 * 0x04 位置直通 / 0x05 低速大扭。⚠ 只有 0x01 是手册明写的，别的都是推断 ——
 * 所以固件里做成了**可在线切换的参数 0x13**，现场两个值各试一次即可。*/
#define JC_MODE_TORQUE      0x00    // 力矩模式（推断）
#define JC_MODE_SPEED       0x01    // 速度模式（手册明写 ✓）
#define JC_MODE_POS_TRAP    0x02    // 位置梯形轨迹（推断）
#define JC_MODE_POS_SMOOTH  0x03    // 位置平滑（推断）
#define JC_MODE_POS_DIRECT  0x04    // 位置直通（推断）
/* ⚠⚠ 低速大扭模式（手册原文）：
 *   "在速度模式下，如果设置速度过低，电机没劲转不动。低速大扭模式可以使电机在
 *    超低速时仍保持最大扭矩，速度低至 0.01rpm"
 *   —— 云台的精细修正要的正是 0.1~2rpm 的超低速，而普通速度模式在那里**没劲**，
 *   于是"粘住 -> 误差攒够 -> 猛跳一下"= 现场看到的"一顿一顿"。
 *   所以默认用它（0x05），并可在线切回 0x01 对比。*/
#define JC_MODE_LOWSPEED    0x05    // 低速大扭矩模式（推断）
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

/** 主动读错误码：命令字 0x43，寄存器 0x000C（回复 data[4..7] = 32 位故障码） */
void jc_read_error(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/** 解析 0x43/0x0C 的错误码回复；返回 1 = 是本帧 */
uint8_t jc_parse_error_reply(const uint8_t *data, uint8_t len, uint32_t *err_code);

/** 主动读控制模式：命令字 0x43，寄存器 0x0060（回复 data[5] = 模式值 0~5） */
void jc_read_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/** 解析 0x43/0x60 的模式回复；返回 1 = 是本帧 */
uint8_t jc_parse_mode_reply(const uint8_t *data, uint8_t len, uint8_t *mode);

/** 主动读"闭环使能"状态：命令字 0x43，寄存器 0x00A2（回复 data[5] = 0/1） */
void jc_read_loop_state(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

/** 解析 0x43/0xA2 的闭环状态回复；返回 1 = 是本帧 */
uint8_t jc_parse_loop_state_reply(const uint8_t *data, uint8_t len, uint8_t *on);

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

/** 设置扭矩（力矩模式用）。参数 = Nm × 100（i16，大端）。
 *  依据《JC系列CAN通信说明》5.8：寄存器 0x0020，命令字 0x2B，
 *  例："设置扭矩 0.2Nm -> 2B 00 20 00 00 14 00 00" ✓ */
void jc_set_torque_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                        int16_t nm_x100);

#endif // !_JC4310__H__
