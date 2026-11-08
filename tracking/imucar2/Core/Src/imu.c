#include "imu.h"
#include <math.h>

/* ==================== MPU6050 寄存器 ==================== */
#define MPU_REG_SMPLRT_DIV      0x19u
#define MPU_REG_CONFIG          0x1Au
#define MPU_REG_GYRO_CONFIG     0x1Bu
#define MPU_REG_INT_ENABLE      0x38u
#define MPU_REG_GYRO_ZOUT_H     0x47u
#define MPU_REG_GYRO_ZOUT_L     0x48u
#define MPU_REG_PWR_MGMT_1      0x6Bu
#define MPU_REG_WHO_AM_I        0x75u

#define MPU_PWR_MGMT_1_RESET    0x80u
#define MPU_PWR_MGMT_1_WAKE     0x01u
#define MPU_SMPLRT_DIV_VAL      0x04u
#define MPU_CONFIG_VAL          0x03u
#define MPU_GYRO_CONFIG_VAL     0x10u
#define MPU_GYRO_LSB_PER_DPS    32.8f
#define MPU_WHO_AM_I_EXPECTED   0x68u

/*
 * 从机地址是 b110100X, X 由模块上的 AD0 决定:
 *   0x68 = AD0 接 GND (GY-521 默认)
 *   0x69 = AD0 接 VCC
 * 初始化时两个都探一遍, 免得因为模块跳线不同而一直找不到芯片。
 */
#define MPU_ADDR_7BIT_A         0x68u
#define MPU_ADDR_7BIT_B         0x69u

static uint8_t mpu_addr;
#define MPU_ADDR_W              (uint8_t)(mpu_addr << 1)
#define MPU_ADDR_R              (uint8_t)((mpu_addr << 1) | 1u)

#define IMU_DT_SEC              0.005f
#define IMU_GYRO_LPF_ALPHA      0.50f
#define IMU_ERR_LIMIT           5u

/* ==================== 软件 I2C 底层 (PB14/PB15) ==================== */

#define SCL_PIN                 IMU_I2C_SCL_Pin
#define SDA_PIN                 IMU_I2C_SDA_Pin

#define SCL_H()                 (GPIOB->BSRR = (uint32_t)SCL_PIN)
#define SCL_L()                 (GPIOB->BRR  = (uint32_t)SCL_PIN)
#define SDA_H()                 (GPIOB->BSRR = (uint32_t)SDA_PIN)
#define SDA_L()                 (GPIOB->BRR  = (uint32_t)SDA_PIN)
#define SDA_LEVEL()             ((GPIOB->IDR & (uint32_t)SDA_PIN) != 0u)
#define SCL_LEVEL()             ((GPIOB->IDR & (uint32_t)SCL_PIN) != 0u)

/* 约 200~300kHz 的软件 I2C 时钟(主频 72MHz) */
static inline void i2c_delay(void)
{
    for (volatile uint32_t i = 0u; i < 18u; i++) {
        __NOP();
    }
}

/* 拉高 SCL 并等它真的变高; 带超时, 免得总线被拉死时卡住 5ms 中断 */
static uint8_t i2c_scl_high(void)
{
    uint16_t t = 0u;

    SCL_H();
    while ((SCL_LEVEL() == 0u) && (t < 500u)) {
        t++;
    }
    if (t >= 500u) {
        return 0u;
    }
    return 1u;
}

static void i2c_start(void)
{
    SDA_H();
    (void)i2c_scl_high();
    i2c_delay();
    SDA_L();
    i2c_delay();
    SCL_L();
    i2c_delay();
}

static void i2c_stop(void)
{
    SDA_L();
    i2c_delay();
    (void)i2c_scl_high();
    i2c_delay();
    SDA_H();
    i2c_delay();
}

/* 发送一个字节, 返回 1 = 收到 ACK */
static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0u; i < 8u; i++) {
        if ((b & 0x80u) != 0u) {
            SDA_H();
        } else {
            SDA_L();
        }
        b = (uint8_t)(b << 1);
        i2c_delay();
        if (i2c_scl_high() == 0u) {
            return 0u;
        }
        i2c_delay();
        SCL_L();
        i2c_delay();
    }

    SDA_H();
    i2c_delay();
    if (i2c_scl_high() == 0u) {
        return 0u;
    }
    i2c_delay();
    ack = (SDA_LEVEL() == 0u) ? 1u : 0u;
    SCL_L();
    i2c_delay();
    return ack;
}

/* 读一个字节, ack = 1 时回 ACK, 否则回 NACK */
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t v = 0u;

    SDA_H();
    for (i = 0u; i < 8u; i++) {
        v = (uint8_t)(v << 1);
        i2c_delay();
        if (i2c_scl_high() == 0u) {
            return 0u;
        }
        i2c_delay();
        if (SDA_LEVEL() != 0u) {
            v |= 1u;
        }
        SCL_L();
        i2c_delay();
    }

    if (ack != 0u) {
        SDA_L();
    } else {
        SDA_H();
    }
    i2c_delay();
    (void)i2c_scl_high();
    i2c_delay();
    SCL_L();
    i2c_delay();
    SDA_H();
    return v;
}

/* 总线恢复: 如果从机把 SDA 拉死, 打 9 个时钟把它顶出来 */
static void i2c_bus_recover(void)
{
    uint8_t i;

    SDA_H();
    for (i = 0u; i < 9u; i++) {
        SCL_L();
        i2c_delay();
        SCL_H();
        i2c_delay();
    }
    i2c_stop();
}

static uint8_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t ok = 0u;

    i2c_start();
    if (i2c_write_byte(MPU_ADDR_W) != 0u) {
        if (i2c_write_byte(reg) != 0u) {
            if (i2c_write_byte(val) != 0u) {
                ok = 1u;
            }
        }
    }
    i2c_stop();
    return ok;
}

static uint8_t mpu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t ok = 0u;

    if ((buf == 0) || (len == 0u)) {
        return 0u;
    }

    i2c_start();
    if (i2c_write_byte(MPU_ADDR_W) != 0u) {
        if (i2c_write_byte(reg) != 0u) {
            i2c_start();
            if (i2c_write_byte(MPU_ADDR_R) != 0u) {
                for (i = 0u; i < len; i++) {
                    buf[i] = i2c_read_byte((uint8_t)((i + 1u) < len));
                }
                ok = 1u;
            }
        }
    }
    i2c_stop();
    return ok;
}

/* 发一个地址看有没有 ACK(不改全局地址, 扫描不影响后面的初始化) */
static uint8_t i2c_ack_addr(uint8_t addr7)
{
    uint8_t ack;

    i2c_start();
    ack = i2c_write_byte((uint8_t)(addr7 << 1));
    i2c_stop();
    return ack;
}

/* ==================== 对外接口 ==================== */

static uint8_t  who_am_i;
static uint8_t  online;
static uint8_t  err_cnt;
static uint16_t err_total;

static float    gyro_bias;
static float    gyro_lp;
static float    yaw_deg;
static float    gyro_raw_dps;       /* 未扣零偏的角速度(供 IMU_TrimBias 用) */

uint8_t IMU_Init(void)
{
    uint8_t tmp = 0u;
    uint8_t cand[2];
    uint8_t i;

    online = 0u;
    who_am_i = 0u;
    mpu_addr = 0u;
    err_cnt = 0u;
    err_total = 0u;
    gyro_bias = 0.0f;
    gyro_lp = 0.0f;
    yaw_deg = 0.0f;

    SCL_H();
    SDA_H();
    HAL_Delay(100);     /* MPU6050 上电后需要一点稳定时间(手册 30ms 起) */

    /* 1) 找器件: 先试 0x68(AD0 接地), 不行再试 0x69 */
    cand[0] = MPU_ADDR_7BIT_A;
    cand[1] = MPU_ADDR_7BIT_B;
    for (i = 0u; i < 2u; i++) {
        mpu_addr = cand[i];
        if (mpu_read_regs(MPU_REG_WHO_AM_I, &tmp, 1u) != 0u) {
            if ((tmp != 0x00u) && (tmp != 0xFFu)) {
                who_am_i = tmp;
                break;
            }
        }
        mpu_addr = 0u;
    }
    if (mpu_addr == 0u) {
        i2c_bus_recover();
        return 0u;
    }

    /* 2) 器件复位, 手册要求复位后等 100ms */
    (void)mpu_write_reg(MPU_REG_PWR_MGMT_1, MPU_PWR_MGMT_1_RESET);
    HAL_Delay(100);

    /* 3) 退出休眠并选时钟源 */
    if (mpu_write_reg(MPU_REG_PWR_MGMT_1, MPU_PWR_MGMT_1_WAKE) == 0u) {
        i2c_bus_recover();
        return 0u;
    }
    HAL_Delay(10);

    /* 4) 采样率 / 低通 / 量程 / 关中断 */
    if (mpu_write_reg(MPU_REG_SMPLRT_DIV, MPU_SMPLRT_DIV_VAL) == 0u) {
        i2c_bus_recover();
        return 0u;
    }
    if (mpu_write_reg(MPU_REG_CONFIG, MPU_CONFIG_VAL) == 0u) {
        i2c_bus_recover();
        return 0u;
    }
    if (mpu_write_reg(MPU_REG_GYRO_CONFIG, MPU_GYRO_CONFIG_VAL) == 0u) {
        i2c_bus_recover();
        return 0u;
    }
    (void)mpu_write_reg(MPU_REG_INT_ENABLE, 0x00u);

    HAL_Delay(50);

    online = 1u;
    return 1u;
}

void IMU_CalibrateGyro(uint16_t samples)
{
    uint8_t  buf[2];
    int32_t  sum = 0;
    uint16_t n = 0u;
    uint16_t i;

    if (samples == 0u) {
        return;
    }

    for (i = 0u; i < samples; i++) {
        if (mpu_read_regs(MPU_REG_GYRO_ZOUT_H, buf, 2u) != 0u) {
            sum += (int32_t)(int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
            n++;
        }
    }

    if (n != 0u) {
        gyro_bias = ((float)sum / (float)n) / MPU_GYRO_LSB_PER_DPS;
    }
    gyro_lp = 0.0f;
    yaw_deg = 0.0f;
}

void IMU_Update(void)
{
    uint8_t buf[2];
    int16_t raw;
    float   gz;

    if (mpu_read_regs(MPU_REG_GYRO_ZOUT_H, buf, 2u) == 0u) {
        if (err_cnt < 250u) {
            err_cnt++;
        }
        if (err_total < 0xFFFFu) {
            err_total++;
        }
        if (err_cnt >= IMU_ERR_LIMIT) {
            online = 0u;
            if (err_cnt == IMU_ERR_LIMIT) {
                i2c_bus_recover();
            }
        }
        return;
    }

    err_cnt = 0u;
    online = 1u;

    raw = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    gyro_raw_dps = (float)raw / MPU_GYRO_LSB_PER_DPS;
    gz  = gyro_raw_dps - gyro_bias;
    gz *= IMU_GYRO_Z_SIGN;

    gyro_lp += (gz - gyro_lp) * IMU_GYRO_LPF_ALPHA;
    yaw_deg += gyro_lp * IMU_DT_SEC;
}

float IMU_GetYawDeg(void)
{
    return yaw_deg;
}

void IMU_TrimBias(void)
{
    /*
     * 把零偏慢慢拉向"当前读数", 也就是让扣完零偏的角速度趋于 0。
     * 系数 0.0005(每 5ms) -> 时间常数约 10 秒, 足够慢, 不会把真实的缓慢转动当零偏吃掉。
     * 读数大于 5°/s 就完全不修正 —— 那是车在被推动或者真的在转。
     */
    if (fabsf(gyro_raw_dps) < 5.0f) {
        gyro_bias += (gyro_raw_dps - gyro_bias) * 0.0005f;
    }
}

float IMU_GetGyroZDps(void)
{
    return gyro_lp;
}

void IMU_ResetYaw(void)
{
    yaw_deg = 0.0f;
}

uint8_t IMU_IsOnline(void)
{
    return online;
}

uint8_t IMU_GetWhoAmI(void)
{
    return who_am_i;
}

uint16_t IMU_GetErrorCount(void)
{
    return err_total;
}

uint8_t IMU_IsExpected(void)
{
    return (who_am_i == MPU_WHO_AM_I_EXPECTED) ? 1u : 0u;
}

uint8_t IMU_GetAddress(void)
{
    return mpu_addr;
}

uint8_t IMU_BusCheck(void)
{
    uint8_t r = 0u;
    volatile uint32_t i;

    SDA_H();
    SCL_H();
    for (i = 0u; i < 2000u; i++) {
        __NOP();
    }
    if (SCL_LEVEL() != 0u) {
        r |= 0x01u;
    }
    if (SDA_LEVEL() != 0u) {
        r |= 0x02u;
    }
    return r;
}

uint8_t IMU_ProbeBus(uint8_t *list, uint8_t max)
{
    uint8_t addr;
    uint8_t n = 0u;

    for (addr = 0x08u; addr <= 0x77u; addr++) {
        if (i2c_ack_addr(addr) != 0u) {
            if ((list != 0) && (n < max)) {
                list[n] = addr;
            }
            n++;
        }
    }
    return n;
}
