#ifndef __MOTOR_CTR_H
#define __MOTOR_CTR_H

#include "stm32f10x.h"

// 电机相关宏定义
#define MOTOR_FREQUENCY_17KHZ    17000   // PWM频率17KHz
#define MOTOR_DUTY_MAX           10000   // 最大占空比值
#define MOTOR_DUTY_MIN           0       // 最小占空比值

// 电机编号定义
#define MOTOR_R                  1       // 电机1
#define MOTOR_L                  2       // 电机2

// 电机方向定义
#define MOTOR_DIR_FORWARD        0       // 正转
#define MOTOR_DIR_BACKWARD       1       // 反转

// 函数声明
void Motor_Init(void);                           // 电机初始化函数
void Motor_Enable(void);                         // 使能电机
void Motor_Disable(void);                        // 禁用电机
void Motor_SetSpeed(uint8_t motor_id, uint16_t duty);  // 设置电机速度 (0-10000)
void Motor_SetDirection(uint8_t motor_id, uint8_t direction);  // 设置电机方向
void Motor_Stop(uint8_t motor_id);               // 停止指定电机
void Motor_StopAll(void);                        // 停止所有电机
void Motor_SetFrequency(uint32_t freq);          // 设置PWM频率
uint16_t Motor_GetDuty(uint8_t motor_id);        // 获取当前占空比
uint8_t Motor_GetDirection(uint8_t motor_id);    // 获取当前方向

#endif 