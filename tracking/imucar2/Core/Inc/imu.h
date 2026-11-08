#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"
#include <stdint.h>

/* ==========================================================================
 *  IMU 驱动 (MPU6050 模块, 软件 I2C)
 * --------------------------------------------------------------------------
 *  硬件:  模块插在板上 H3 座(1=5V, 2=GND, 3=PB14, 4=PB15, 8=PB9)
 *         PB14 = SCL, PB15 = SDA, 器件地址 0x68 (AD0 接地) 或 0x69
 *         本工程只读三轴陀螺仪的 Z 轴, 用来做"转过 90 度"的角度闭环。
 *
 *  配置:  PWR_MGMT_1 = 0x01  唤醒 + 时钟源选 X 轴陀螺 PLL
 *         SMPLRT_DIV = 4    1kHz/(1+4) = 200Hz 输出(和 5ms 控制周期一致)
 *         CONFIG     = 3    数字低通 44Hz
 *         GYRO_CONFIG= 0x10 FS_SEL = 2 -> ±1000dps, 32.8 LSB/(°/s)
 *         WHO_AM_I(0x75) 复位值 0x68
 *
 *  时序:  IMU_Update() 每 5ms 调用一次, 读一次角速度并积分航向角。
 * ========================================================================== */

/*
 * 航向角符号: 车体逆时针(左转)为正。
 * 模块正装(芯片朝上)时就是 +1; 如果实测左转时 yaw 在减小, 改成 (-1.0f)。
 */
#define IMU_GYRO_Z_SIGN        (+1.0f)

/* 初始化: 找器件 + 软复位 + 配置量程, 返回 1 成功 / 0 失败 */
uint8_t IMU_Init(void);

/*
 * 陀螺仪零偏校准: 采样 samples 次求平均作为零偏。
 * 调用时小车必须完全静止! 建议 500~1000 次。
 */
void IMU_CalibrateGyro(uint16_t samples);

/* 5ms 周期调用(放在 TIM2 中断里) */
void IMU_Update(void);

/* 航向角(度): 正 = 逆时针/左转 */
float IMU_GetYawDeg(void);

/* 当前 Z 轴角速度(度/秒), 已扣零偏、已滤波 */
float IMU_GetGyroZDps(void);

/* 把航向角清零(准备转弯时调用) */
void IMU_ResetYaw(void);

/*
 * 车静止时调用: 用当前读数缓慢把陀螺仪零偏修正到零。
 * 用来消掉温漂(实测停车时 GZ 会稳定偏 -2.8°/s 左右, 转 1.4 秒就要吃掉约 4°)。
 * 只读数小于 5°/s 时才修正, 所以车被推动时不会把零偏带跑。
 */
void IMU_TrimBias(void);

/* 是否在线(最近的 I2C 读写是否正常) */
uint8_t IMU_IsOnline(void);

/* WHO_AM_I(0x75) 的读数, 初始化失败时为 0 */
uint8_t IMU_GetWhoAmI(void);

/* WHO_AM_I 是否等于 MPU6050 的复位值 0x68 */
uint8_t IMU_IsExpected(void);

/* 实际用到的 7 位从机地址(0x68/0x69), 0 = 两个地址都没找到器件 */
uint8_t IMU_GetAddress(void);

/*
 * 总线体检: 把 SCL/SDA 都释放成高电平再读回来。
 * 返回 bit0 = SCL 能拉高, bit1 = SDA 能拉高(正常应该是 0x03)。
 * 读不到高电平说明线被拉死/短路/没上拉。
 */
uint8_t IMU_BusCheck(void);

/* 扫描 I2C 总线 0x08~0x77, 把有应答的 7 位地址写进 list, 返回个数 */
uint8_t IMU_ProbeBus(uint8_t *list, uint8_t max);

#endif /* __IMU_H__ */
