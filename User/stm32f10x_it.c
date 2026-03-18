/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "ABEncoder.h"
#include "ADC_get.h"
#include "LSM6DSR_Config.h"
#include "pose.h"
#include "Motor_ctr.h"
#include "PID_Controller.h"
#include "RGB_Led.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "M3PWM.h"
#include "Odometer.h"
#include "CircleHandler.h"
#include "BTComm.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
float add_angle = 0;
float add_angle_num = 0;
int16_t position_get = 0.0f;
BlackPointResult_t result_BlackPoint;
volatile uint32_t g_systick_ms = 0;
volatile uint32_t g_lose_time = 0; // 定义全局丢线计数器，以便 main.c 清零
extern uint16_t uart_rev_tiem;
extern uint8_t star_car;
extern volatile uint8_t g_bt_key_control_mode;

// 偏航角（Z轴积分），单位：度，供状态机使用
// 使用方法：extern volatile float g_yaw_angle; 然后直接读取，即为角度�?
// 清零方法：g_yaw_angle = 0;
volatile float g_yaw_angle = 0.0f;

// 看门狗变量：检测主循环是否正常运行
volatile uint32_t g_main_loop_watchdog = 0;
#define MAIN_LOOP_TIMEOUT_MS  500  // 主循环超�?00ms没响应则认为卡死

void SysTick_Handler(void)
{
	float dt = 0.002f;  // 修正：SysTick 实际�?2ms 周期
  g_systick_ms += 2;
	LSM6DSR_ReadData(&LSE6DSR_data);
	LSM6DSR_ConvertToPhysics(&LSE6DSR_data);
	
	// 串口接收超时看门狗（已屏蔽，避免影响本地模式�?
	uart_rev_tiem ++;
	/* 
	if(uart_rev_tiem > 250)
	{
		star_car = 0; 
		uart_rev_tiem = 250;
		Motor_Disable();
	}
	*/
//	// 更新旧的角度计算（用于兼容性，使用转换后的角速度�?
	add_angle += LSE6DSR_data.gz_rads * dt;  // 使用弧度/秒，正确积分
	add_angle_num += 1.0f;
	
	// Yaw角积分由驱动层统一管理（死区滤�?梯形积分），同步给外�?extern 引用
	LSM6DSR_UpdateYaw(dt);
	g_yaw_angle = LSM6DSR_GetYaw();
	
	// 保留旧的姿态解算调用（可选，用于对比�?
//	prepare_data();
//	imuupdate(&gyr_rad, &acc_g, &att_angle);
	
  ABEncoder_UpdateSpeed();
  Odometer_Update();  /* 更新里程计数据，必须在编码器更新之后 */
  MuxADC_SampleAll();
  if(!g_bt_key_control_mode && PID_PositionLoop_IsEnabled())
  {
    volatile uint16_t *adc_src = BlackPoint_Finder_IsSimulateMode() 
                                 ? BlackPoint_Finder_GetSimulateADC() 
                                 : g_mux_adc_values;
    
    /* 圆环状态机更新 (在正常搜索之�? */
    Circle_Update(adc_src);
    
    /* 圆环 ENTERING/EXITING 阶段已在 Circle_Update 中设置了 position_get,
     * 跳过正常的全幅搜�? NORMAL/IN_CIRCLE 阶段走正常流�?*/
    if(Circle_GetState() == CIRCLE_STATE_NORMAL || 
       Circle_GetState() == CIRCLE_STATE_IN_CIRCLE)
    {
      BlackPoint_Finder_Search(adc_src, &result_BlackPoint);
      
      /* 无论是否找到，都�?precise_position 更新 position_get�?
       * 未找到时 Search 返回 last_precise_position (初始�?7.5f = 75)�?
       * 防止 position_get 停留在全局初始�?0 导致 PID 产生 kp*(0-7.5) 的冲击�?*/
      position_get = result_BlackPoint.precise_position * 10.0f;
      
      if(result_BlackPoint.found)
      {
        if(g_lose_time > 0)
        {
          g_lose_time--;
        }
      }
      else
      {
        g_lose_time ++;
        if(g_lose_time > 250) 
        {
          g_lose_time = 50;
          Motor_Disable();
          Motor_StopAll();
          SpeedPID_ResetState();
        }
      }
    }
  }
  // 必须�?PID_Control_Update 移出 if 条件
  // 因为自动整定需要在没有 PositionLoopEnabled 的情况下运行
  // 且需要在中断中获得精确的 2ms 周期
  PID_Control_Update();
  BT_GyroStream_Tick();  /* 陀螺仪数据流心跳，2ms 周期 */
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

