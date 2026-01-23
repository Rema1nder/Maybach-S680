#ifndef __M3PWM_H
#define __M3PWM_H

#include "stm32f10x.h"

// PWM相关宏定义
#define PWM_FREQUENCY_17KHZ    37000   // PWM频率17KHz
#define PWM_DUTY_CYCLE_50      50      // 默认占空比50%

// 函数声明
void M3PWM_Init(void);                    // PWM初始化函数
void M3PWM_SetDutyCycle(uint16_t duty);    // 设置PWM占空比 (0-1000)
void M3PWM_Start(void);                   // 启动PWM输出
void M3PWM_Stop(void);                    // 停止PWM输出
void M3PWM_SetFrequency(uint32_t freq);   // 设置PWM频率

#endif 
