#ifndef _JC4310__H__
#define _JC4310__H__

#include "main.h"
#include "can_bsp.h"

void jc_enter_closed_loop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void jc_set_torque_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
void jc_set_torque_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                        int16_t nm_x100);

void jc_read_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);
uint8_t jc_parse_position_reply(const uint8_t *data, uint8_t len,
                                float *pos_deg);
uint8_t jc_parse_speed_reply(const uint8_t *data, uint8_t len,
                             float *pos_deg, int16_t *speed_raw,
                             int16_t *current_raw);

#endif
