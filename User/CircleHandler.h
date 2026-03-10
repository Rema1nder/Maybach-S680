#ifndef __CIRCLE_HANDLER_H__
#define __CIRCLE_HANDLER_H__

#include "stm32f10x.h"
#include <stdint.h>

/**
 * @file    CircleHandler.h
 * @brief   圆环元素检测与处理模块
 * @details 使用状态机 + 路口检测 + 陀螺仪角度积分实现圆环的驶入、绕行、驶出
 *
 *          状态转移:
 *          NORMAL → 检测到路口 → ENTERING
 *          ENTERING → 偏向引导驶入圆弧 → IN_CIRCLE
 *          IN_CIRCLE → 陀螺仪累计转过约 330° → EXITING
 *          EXITING → 检测到路口并偏向驶出 → NORMAL
 */

/* 圆环方向 */
#define CIRCLE_DIR_LEFT   0   /* 圆环在左侧 (逆时针绕行) */
#define CIRCLE_DIR_RIGHT  1   /* 圆环在右侧 (顺时针绕行) */

/* 圆环状态机状态 */
typedef enum {
	CIRCLE_STATE_NORMAL = 0,    /* 正常巡线 */
	CIRCLE_STATE_ENTERING,      /* 正在驶入圆环 (路口偏向引导) */
	CIRCLE_STATE_IN_CIRCLE,     /* 在圆环内绕行 */
	CIRCLE_STATE_EXITING        /* 正在驶出圆环 */
} CircleState_t;

/**
 * @brief  初始化圆环处理模块
 */
void Circle_Init(void);

/**
 * @brief  重置圆环状态机为 NORMAL
 */
void Circle_Reset(void);

/**
 * @brief  设置圆环方向
 * @param  dir  CIRCLE_DIR_LEFT 或 CIRCLE_DIR_RIGHT
 */
void Circle_SetDirection(uint8_t dir);

/**
 * @brief  获取当前圆环方向
 * @return CIRCLE_DIR_LEFT 或 CIRCLE_DIR_RIGHT
 */
uint8_t Circle_GetDirection(void);

/**
 * @brief  获取当前圆环状态
 * @return 当前状态
 */
CircleState_t Circle_GetState(void);

/**
 * @brief  圆环状态机更新 (在 SysTick 中断中调用, 2ms 周期)
 * @param  adc_values  传感器 ADC 值数组
 * @note   此函数负责更新状态机, 以及设置 position_get 的值
 *         当处于 ENTERING/EXITING 状态时, 使用半侧追线替代全幅追线
 */
void Circle_Update(volatile uint16_t *adc_values);

/**
 * @brief  查询当前是否处于圆环处理模式 (非 NORMAL)
 * @return 1=正在处理圆环, 0=正常巡线
 */
uint8_t Circle_IsActive(void);

/**
 * @brief  获取圆环内累积转过的角度 (度)
 * @return 累积角度绝对值
 */
float Circle_GetAccumulatedAngle(void);

#endif /* __CIRCLE_HANDLER_H__ */
