#include "jc4310.h"

void jc_set_torque_x100(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                        int16_t nm_x100)
{
    uint8_t data[8];

    if ((motor_id == 0u) || (motor_id > 127u))
    {
        return;
    }
    data[0] = 0x2B;
    data[1] = 0x00;
    data[2] = 0x20;
    data[3] = 0x00;
    data[4] = (uint8_t)(((uint16_t)nm_x100 >> 8) & 0xFFu);
    data[5] = (uint8_t)((uint16_t)nm_x100 & 0xFFu);
    data[6] = 0x00;
    data[7] = 0x00;
    fdcanx_send_data(hfdcan, 0x600u + motor_id, data, 8u);
}

void jc_enter_closed_loop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t data[8];

    if ((motor_id == 0u) || (motor_id > 127u))
    {
        return;
    }
    data[0] = 0x2B;
    data[1] = 0x00;
    data[2] = 0xA2;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x01;
    data[6] = 0x00;
    data[7] = 0x00;
    fdcanx_send_data(hfdcan, 0x600u + motor_id, data, 8u);
}

void jc_set_torque_mode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t data[8];

    if ((motor_id == 0u) || (motor_id > 127u))
    {
        return;
    }
    data[0] = 0x2B;
    data[1] = 0x00;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    fdcanx_send_data(hfdcan, 0x600u + motor_id, data, 8u);
}

void jc_read_position(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t data[8];

    if ((motor_id == 0u) || (motor_id > 127u))
    {
        return;
    }
    data[0] = 0x43;
    data[1] = 0x00;
    data[2] = 0x08;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    fdcanx_send_data(hfdcan, 0x600u + motor_id, data, 8u);
}

uint8_t jc_parse_position_reply(const uint8_t *data, uint8_t len,
                                float *pos_deg)
{
    int32_t raw;

    if ((data == NULL) || (pos_deg == NULL) || (len < 8u))
    {
        return 0u;
    }
    if ((data[0] != 0x43u) || (data[2] != 0x08u))
    {
        return 0u;
    }
    raw = (int32_t)(((uint32_t)data[4] << 24) |
                    ((uint32_t)data[5] << 16) |
                    ((uint32_t)data[6] << 8) |
                    (uint32_t)data[7]);
    *pos_deg = (float)raw / 100.0f;
    return 1u;
}

uint8_t jc_parse_speed_reply(const uint8_t *data, uint8_t len,
                             float *pos_deg, int16_t *speed_raw,
                             int16_t *current_raw)
{
    int32_t raw24;

    if ((data == NULL) || (len < 8u) || (data[0] != 0x2Au))
    {
        return 0u;
    }
    raw24 = (int32_t)(((uint32_t)data[1] << 24) |
                      ((uint32_t)data[2] << 16) |
                      ((uint32_t)data[3] << 8));
    raw24 >>= 8;
    if (pos_deg != NULL)
    {
        *pos_deg = (float)raw24 / 100.0f;
    }
    if (speed_raw != NULL)
    {
        *speed_raw = (int16_t)(((uint16_t)data[4] << 8) |
                               (uint16_t)data[5]);
    }
    if (current_raw != NULL)
    {
        *current_raw = (int16_t)(((uint16_t)data[6] << 8) |
                                 (uint16_t)data[7]);
    }
    return 1u;
}
