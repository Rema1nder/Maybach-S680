/**
 * @file    main.c
 * @brief   巡线小车主程序
 * @details 实现以下功能:
 *          - K1: 启动/停止巡线模式
 *          - K2: 启动/停止蓝牙遥控模式
 *          - K3: 打印传感器状态 (调试用)
 *          - K4: 急停/解除急停
 *          
 *          控制架构:
 *          - 巡线模式: 位置环 + 速度环双环 PID 控制
 *          - 蓝牙模式: 仅速度环控制, 接收蓝牙指令
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "M3PWM.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "ADC_get.h"
#include "BTComm.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "RGB_Led.h"
#include "LSM6DSR_Config.h"
#include "pose.h"
#include "Key_Scan.h"
#include "system_stm32f10x.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"

/* ========================================================================== */
/*                              外部变量引用                                   */
/* ========================================================================== */
extern float add_angle;
extern float add_angle_num;
extern int16_t position_get;
extern int16_t speed_left;
extern int16_t speed_right;
extern BlackPointResult_t result_BlackPoint;
extern volatile uint32_t g_systick_ms;
extern volatile uint32_t g_lose_time;            /* 丢线计数器 */
extern volatile uint32_t g_main_loop_watchdog;   /* 主循环看门狗 */

/* ========================================================================== */
/*                              全局变量定义                                   */
/* ========================================================================== */
uint8_t star_car = 0;                        /* 巡线模式启动标志: 0=停止, 1=运行 */
volatile uint8_t g_bt_key_control_mode = 0;  /* 蓝牙遥控模式标志 */
float BDI_V = 0;                             /* 电池电压 (V) */

/* ========================================================================== */
/*                              系统初始化                                     */
/* ========================================================================== */

/**
 * @brief  配置 SysTick 定时器
 * @note   2ms 中断周期, 用于 PID 控制和传感器采样
 */
static void SysTick_Init(void)
{
	SysTick_Config(SystemCoreClock / 500);
}

/* ========================================================================== */
/*                           蓝牙遥控辅助函数                                  */
/* ========================================================================== */

/**
 * @brief  将速度百分比转换为目标速度值
 * @param  speed_percent  速度百分比 (0-100)
 * @return 目标速度 (编码器单位)
 */
static float BT_SpeedPercentToTargetSpeed(uint8_t speed_percent)
{
	if(speed_percent > 100) speed_percent = 100;
	return (230.0f * (float)speed_percent) / 100.0f;
}

/**
 * @brief  将速度百分比转换为 PWM 占空比上限
 * @param  speed_percent  速度百分比 (0-100)
 * @return PWM 占空比
 */
static uint16_t BT_SpeedPercentToDuty(uint8_t speed_percent)
{
	if(speed_percent > 100) speed_percent = 100;
	return (uint16_t)((MOTOR_DUTY_MAX * (uint32_t)speed_percent) / 100U);
}

/**
 * @brief  应用蓝牙运动指令
 * @param  state  蓝牙状态结构体指针
 * @note   包含软启动斜坡控制, 防止电机突然加速
 */
static void BT_ApplyMotion(const BT_State_t *state)
{
	static uint16_t ramp_duty = 0;
	static uint32_t last_ms = 0;
	static Motion_Mode_t last_mode = MOTION_IDLE;

	if(state == NULL) return;

	/* 急停状态: 立即停止所有电机 */
	if(state->emergency_stop)
	{
		ramp_duty = 0;
		last_mode = MOTION_IDLE;
		Motor_StopAll();
		Motor_Disable();
		PID_SetBluetoothTarget(0.0f, 0.0f, 0);
		SpeedPID_ResetState();
		return;
	}

	/* 空闲状态: 停止电机 */
	if(state->motion_mode == MOTION_IDLE)
	{
		ramp_duty = 0;
		last_mode = MOTION_IDLE;
		Motor_StopAll();
		PID_SetBluetoothTarget(0.0f, 0.0f, 0);
		SpeedPID_ResetState();
		return;
	}

	/* 模式切换: 重置斜坡 */
	if(state->motion_mode != last_mode)
	{
		ramp_duty = 0;
		last_mode = state->motion_mode;
		SpeedPID_ResetState();
	}

	/* 计算目标速度和占空比上限 */
	float target_speed = BT_SpeedPercentToTargetSpeed(state->motion_speed);
	uint16_t duty_cap = BT_SpeedPercentToDuty(state->motion_speed);

	/* 软启动斜坡控制 */
	uint32_t now_ms = g_systick_ms;
	uint32_t elapsed_ms = now_ms - last_ms;
	if(elapsed_ms > 100) elapsed_ms = 2;  /* 防止异常大的时间间隔 */
	last_ms = now_ms;

	uint32_t max_delta = elapsed_ms * 50U;
	if(max_delta == 0U) max_delta = 50U;

	if(ramp_duty < duty_cap)
	{
		uint16_t delta = (duty_cap - ramp_duty) > max_delta ? max_delta : (duty_cap - ramp_duty);
		ramp_duty += delta;
	}
	else if(ramp_duty > duty_cap)
	{
		uint16_t delta = (ramp_duty - duty_cap) > max_delta ? max_delta : (ramp_duty - duty_cap);
		ramp_duty -= delta;
	}

	Motor_Enable();

	/* 根据运动模式设置左右轮目标速度 */
	float left_target = 0.0f;
	float right_target = 0.0f;

	switch(state->motion_mode)
	{
		case MOTION_FORWARD:
			left_target = target_speed;
			right_target = target_speed;
			break;
		case MOTION_BACKWARD:
			left_target = -target_speed;
			right_target = -target_speed;
			break;
		case MOTION_TURN_LEFT:
			left_target = -target_speed * 0.5f;
			right_target = target_speed * 0.5f;
			break;
		case MOTION_TURN_RIGHT:
			left_target = target_speed * 0.5f;
			right_target = -target_speed * 0.5f;
			break;
		case MOTION_IDLE:
		default:
			Motor_StopAll();
			return;
	}

	/* 传递目标值给 PID 模块, 实际输出在 SysTick 中断中执行 */
	PID_SetBluetoothTarget(left_target, right_target, ramp_duty);
}

/* ========================================================================== */
/*                              按键事件处理                                   */
/* ========================================================================== */

/**
 * @brief  处理按键事件
 * @param  event  按键事件指针
 */
static void HandleKeyEvent(Key_Event_t *event)
{
	if(event == NULL) return;

	switch(event->key_id)
	{
		case KEY_NONE:
			break;

		/* K1: 巡线模式启停 */
		case KEY_K1:
			if(BT_IsEmergencyStopped() || g_bt_key_control_mode) break;
			
			if(star_car)
			{
				/* 停止巡线 */
				star_car = 0;
				Motor_StopAll();
				Motor_Disable();
				M3PWM_SetDutyCycle(0);  /* 停止负压风扇 */
				PID_PositionLoop_Enable(0);
				SpeedPID_ResetState();
				RGB_SetColor(RGB_COLOR_OFF);
			}
			else
			{
				/* 启动巡线 */
				RGB_SetColor(RGB_COLOR_G);
				star_car = 1;
				g_lose_time = 0;  /* 清零丢线计数 */
				SpeedPID_ResetState();
				PositionPID_ResetState();
				Motor_Enable();
				M3PWM_SetDutyCycle(200);  /* 启动负压风扇 */
				M3PWM_Start();
				PID_PositionLoop_Enable(1);
			}
			break;

		/* K2: 蓝牙遥控模式启停 */
		case KEY_K2:
			if(BT_IsEmergencyStopped() || star_car) break;
			
			if(g_bt_key_control_mode)
			{
				/* 退出蓝牙模式 */
				g_bt_key_control_mode = 0;
				BT_HandleKeyControlModeChange(0);
				Motor_StopAll();
				Motor_Disable();
				SpeedPID_ResetState();
				RGB_SetColor(RGB_COLOR_OFF);
			}
			else
			{
				/* 进入蓝牙模式 */
				g_bt_key_control_mode = 1;
				star_car = 0;
				PID_PositionLoop_Enable(0);
				BT_HandleKeyControlModeChange(1);
				SpeedPID_ResetState();
				Motor_Enable();
				RGB_SetColor(RGB_COLOR_B);
			}
			break;

		/* K3: 打印传感器状态 */
		case KEY_K3:
			RGB_SetColor(RGB_COLOR_OFF);
			MuxADC_SampleAll();
			BlackPoint_Finder_Init();
			printf("State: ");
			for(int i = 0; i < 16; i++)
			{
				uint8_t is_black = BlackPoint_Finder_IsBlackPoint(i, g_mux_adc_values[i]);
				printf("%d ", is_black);
			}
			printf("\r\n");
			break;

		/* K4: 急停/解除急停 */
		case KEY_K4:
			if(BT_IsEmergencyStopped())
			{
				/* 解除急停 */
				BT_ClearEmergencyStop();
				SpeedPID_ResetState();
				RGB_SetColor(RGB_COLOR_OFF);
			}
			else
			{
				/* 触发急停 */
				RGB_SetColor(RGB_COLOR_R);
				star_car = 0;
				g_bt_key_control_mode = 0;
				BT_HandleKeyControlModeChange(0);
				SpeedPID_ResetState();
				BT_EmergencyStop();
			}
			break;
	}
}

/* ========================================================================== */
/*                              主函数                                         */
/* ========================================================================== */

int main(void)
{
	/* ====== 外设初始化 ====== */
	RGB_Init();
	LSM6DSR_Init();
	M3PWM_Init();
	SysTick_Init();
	Motor_Init();
	M3PWM_Start();
	ABEncoder_Init();
	BlackPoint_Finder_Init();
	MuxADC_Init();
	Key_Scan_Init();
	Uart2_Init(115200);
	BT_Init();
	PID_Init();
	
	/* ====== 初始状态: 等待按键启动 ====== */
	PID_PositionLoop_Enable(0);

	/* ====== 主循环 ====== */
	while(1)
	{
		/* 喂狗: 主循环正在运行 */
		g_main_loop_watchdog = g_systick_ms;
		
		/* 按键扫描与蓝牙处理 */
		Key_Scan_Update();
		BT_Process();
		
		/* 读取电池电压 */
		BDI_V = (float)g_mux_adc_values[16] * 0.00426508726f;
		
		/* 处理按键事件 */
		Key_Event_t *event = Key_GetEvent();
		if(event != NULL)
		{
			HandleKeyEvent(event);
		}

		/* 蓝牙遥控模式: 应用运动指令 */
		if(g_bt_key_control_mode)
		{
			const BT_State_t *state = BT_GetState();
			BT_ApplyMotion(state);
		}
	}
}
