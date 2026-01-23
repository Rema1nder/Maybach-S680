#ifndef __KEY_SCAN_H
#define __KEY_SCAN_H

#include "stm32f10x.h"

// 按键定义
typedef enum {
    KEY_NONE = 0,   // 无按键
    KEY_K1,         // K1 (PA5)
    KEY_K2,         // K2 (PA4)
    KEY_K3,         // K3 (PC14)
    KEY_K4          // K4 (PC13)
} Key_ID_t;

// 按键事件
typedef struct {
    Key_ID_t key_id;        // 按键ID (KEY_K1, KEY_K2, KEY_K3, KEY_K4)
} Key_Event_t;

/**
 * @brief 初始化按键扫描
 * @note 配置GPIO为上拉输入模式
 */
void Key_Scan_Init(void);

/**
 * @brief 按键扫描函数（非阻塞）
 * @note 需要周期性调用，建议在1ms定时器中断或主循环中调用
 */
void Key_Scan_Update(void);

/**
 * @brief 获取按键当前状态
 * @param key_id: 按键ID (KEY_K1, KEY_K2, KEY_K3, KEY_K4)
 * @return 0=未按下, 1=按下
 */
uint8_t Key_GetState(Key_ID_t key_id);

/**
 * @brief 获取按键事件（按下松开为一次触发）
 * @return 按键事件指针，如果没有事件则返回NULL
 * @note 事件被读取后会自动清除，每个事件只能读取一次
 */
Key_Event_t* Key_GetEvent(void);

/**
 * @brief 检查是否有按键事件
 * @return 1=有事件, 0=无事件
 */
uint8_t Key_HasEvent(void);

/**
 * @brief 清除所有按键事件
 */
void Key_ClearEvent(void);

#endif /* __KEY_SCAN_H */

