#ifndef __BT_COMM_H
#define __BT_COMM_H

#include "stm32f10x.h"
#include <stdint.h>

/*=== 字符串命令协议定义 ===*/
// 命令格式: "CMD:参数\n" 或 "CMD\n"
// 例如: "FWD:50\n" = 前进50%速度, "STOP\n" = 停止

#define BT_CMD_BUFFER_SIZE  64          // 命令缓冲区大小

/*=== 字符串命令定义 ===*/
// 全局命令（任何时候都生效）：
// "ESTOP"    - 急停（立即停止所有电机和负压）
// "VON"      - 负压开
// "VOFF"     - 负压关
// "VS:xx"    - 负压转速 (0-100)
// "SPD:xx"   - 基础速度设置 (0-100)
// "?STATUS"  - 查询状态

// 键控模式命令（需要按下K2进入键控模式）：
// "FWD"      - 前进（使用基础速度）
// "FWD:xx"   - 前进指定速度
// "BWD"      - 后退
// "BWD:xx"   - 后退指定速度
// "TL"       - 左转
// "TL:xx"    - 左转指定速度
// "TR"       - 右转
// "TR:xx"    - 右转指定速度
// "STOP"     - 停止移动

/*=== 数据包结构体（保留用于兼容） ===*/
typedef struct
{
    uint8_t cmd;                        // 命令类型（1字节）
    uint8_t data;                       // 数据（1字节）
} BT_Packet_t;

/*=== 运动模式定义 ===*/
typedef enum
{
    MOTION_IDLE = 0,                    // 空闲
    MOTION_FORWARD,                     // 前进
    MOTION_BACKWARD,                    // 后退
    MOTION_TURN_LEFT,                   // 左转
    MOTION_TURN_RIGHT,                  // 右转
} Motion_Mode_t;

/*=== 全局状态结构 ===*/
typedef struct
{
    Motion_Mode_t motion_mode;          // 当前运动模式
    uint8_t motion_speed;               // 运动速度 (0-100)
    uint8_t base_speed;                 // 基础速度 (0-100)，用于键控时的默认速度
    uint8_t vacuum_enabled;             // 吸盘是否启用
    uint8_t vacuum_speed;               // 吸盘转速 (0-100)
    uint8_t key_control_mode;           // 键控模式 (0=OFF, 1=ON)
    uint8_t emergency_stop;             // 急停标志 (0=正常, 1=急停)
} BT_State_t;

/*=== 函数声明 ===*/

/**
 * @brief 蓝牙通信初始化
 */
void BT_Init(void);

/**
 * @brief 处理接收到的数据
 * @note 应该在主循环中调用
 */
void BT_Process(void);

/**
 * @brief 发送数据包
 * @param packet: 数据包指针
 */
void BT_SendPacket(const BT_Packet_t *packet);

/**
 * @brief 获取当前运动状态
 */
Motion_Mode_t BT_GetMotionMode(void);

/**
 * @brief 获取当前运动速度
 */
uint8_t BT_GetMotionSpeed(void);

/**
 * @brief 获取完整的蓝牙状态
 */
const BT_State_t* BT_GetState(void);

/**
 * @brief 直接设置吸盘转速（用于硬件K1同步状态）
 */
void BT_SetVacuumSpeedDirect(uint8_t speed_percent);

/**
 * @brief 获取当前设置的负压风扇转速百分比 (0-100)
 */
uint8_t BT_GetVacuumSpeed(void);

/**
 * @brief 处理键控模式切换（硬件K2直接调用）
 */
void BT_HandleKeyControlModeChange(uint8_t enabled);

/**
 * @brief 执行急停（全局生效）
 */
void BT_EmergencyStop(void);

/**
 * @brief 清除急停状态
 */
void BT_ClearEmergencyStop(void);

/**
 * @brief 检查是否处于急停状态
 */
uint8_t BT_IsEmergencyStopped(void);

/**
 * @brief 开启/关闭负压电机（全局生效）
 */
void BT_VacuumControl(uint8_t enable);

/**
 * @brief 设置负压电机转速（全局生效）
 */
void BT_VacuumSetSpeed(uint8_t speed);

/**
 * @brief 陀螺仪流状态机心跳（在 SysTick 中以 2ms 周期调用）
 * @note  仅更新标志位，不执行 UART 发送
 */
void BT_GyroStream_Tick(void);

#endif