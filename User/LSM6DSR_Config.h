#ifndef __LSM6DSR_CONFIG_H__
#define __LSM6DSR_CONFIG_H__

#include "stm32f10x.h"
#include <stdint.h>

// 选择通信方式：硬件SPI或软件SPI
// 使用硬件SPI时注释掉下面这行，使用软件SPI时保留
// #define USE_SOFTWARE_SPI

// 引脚定义：PB12(CS), PB13(SCK), PB14(MISO), PB15(MOSI)
#define LSM6DSR_CS_PORT    GPIOB
#define LSM6DSR_CS_PIN     GPIO_Pin_12
#define LSM6DSR_SCK_PORT   GPIOB
#define LSM6DSR_SCK_PIN    GPIO_Pin_13
#define LSM6DSR_MISO_PORT  GPIOB
#define LSM6DSR_MISO_PIN   GPIO_Pin_14
#define LSM6DSR_MOSI_PORT  GPIOB
#define LSM6DSR_MOSI_PIN   GPIO_Pin_15

// LSM6DSR寄存器地址
#define LSM6DSR_WHO_AM_I       0x0F
#define LSM6DSR_DEVICE_ID      0x6B

#define LSM6DSR_FUNC_CFG       0x01
#define LSM6DSR_INT1_CTRL      0x0D
#define LSM6DSR_INT2_CTRL      0x0E

#define LSM6DSR_CTRL1_XL       0x10  // 加速度计控制寄存器1
#define LSM6DSR_CTRL2_G        0x11  // 陀螺仪控制寄存器2
#define LSM6DSR_CTRL3_C        0x12
#define LSM6DSR_CTRL4_C        0x13
#define LSM6DSR_CTRL5_C        0x14
#define LSM6DSR_CTRL6_C        0x15
#define LSM6DSR_CTRL7_G        0x16
#define LSM6DSR_CTRL8_XL       0x17
#define LSM6DSR_CTRL9_XL       0x18
#define LSM6DSR_CTRL10_C       0x19

#define LSM6DSR_STATUS_REG     0x1E

#define LSM6DSR_OUT_TEMP_L     0x20
#define LSM6DSR_OUT_TEMP_H     0x21

#define LSM6DSR_OUTX_L_GYRO    0x22
#define LSM6DSR_OUTX_H_GYRO    0x23
#define LSM6DSR_OUTY_L_GYRO    0x24
#define LSM6DSR_OUTY_H_GYRO    0x25
#define LSM6DSR_OUTZ_L_GYRO    0x26
#define LSM6DSR_OUTZ_H_GYRO    0x27

#define LSM6DSR_OUTX_L_ACC     0x28
#define LSM6DSR_OUTX_H_ACC     0x29
#define LSM6DSR_OUTY_L_ACC     0x2A
#define LSM6DSR_OUTY_H_ACC     0x2B
#define LSM6DSR_OUTZ_L_ACC     0x2C
#define LSM6DSR_OUTZ_H_ACC     0x2D

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	float ax_g;     // 加速度 X轴 (g)
	float ay_g;     // 加速度 Y轴 (g)
	float az_g;     // 加速度 Z轴 (g)
	float gx_rads;  // 角速度 X轴 (弧度/秒)
	float gy_rads;  // 角速度 Y轴 (弧度/秒)
	float gz_rads;  // 角速度 Z轴 (弧度/秒)
}LSM6DSR_DATA_T;
extern LSM6DSR_DATA_T LSE6DSR_data;
// 函数声明
uint8_t LSM6DSR_Init(void);
uint8_t LSM6DSR_ReadID(void);
void LSM6DSR_ReadData(LSM6DSR_DATA_T *physics);
void LSM6DSR_ConvertToPhysics(LSM6DSR_DATA_T *physics);
uint8_t LSM6DSR_ReadReg(uint8_t reg);
void LSM6DSR_WriteReg(uint8_t reg, uint8_t value);
void LSM6DSR_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);

#endif

