#include "BMI088Middleware.h"
#include "main.h"
#include "spi.h"

#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

void BMI088_GPIO_init(void)
{
}

void BMI088_com_init(void)
{
}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 480;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0xFFu;
    /* One byte at the configured SPI clock takes a few microseconds.
     * Never allow a transient SPI fault to hold the 2 ms control loop for
     * 1 s: with the 0.5 s IWDG that would reset the H723 and release pitch. */
    if (HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data,
                                1u, 2u) != HAL_OK)
    {
        return 0xFFu;
    }
    return rx_data;
}
