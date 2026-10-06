#ifndef MahonyAHRS_h
#define MahonyAHRS_h

extern volatile float twoKp;
extern volatile float twoKi;

/* AHRS 采样频率(Hz)：必须等于【实际调用 MahonyAHRSupdateIMU 的频率】。
 * 它决定了四元数积分的步长 halfT = 0.5/sampleFreq：
 *   设成实际频率的 2 倍 -> 姿态积分量只有真实的一半 -> 角度"走一半"
 *   （实测后果：上位机给 6° 偏置，电机要转 11° 才让角度估计到位，
 *    等效把 yaw 环路增益放大 2 倍，视觉闭环很难稳）
 * 由 yuntai_init 按 1000/IMU_PERIOD_MS 自动设定，别手写常数。 */
extern volatile float mahonySampleFreq;

void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

#endif
