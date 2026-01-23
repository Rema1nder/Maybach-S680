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

// 看门狗变量：检测主循环是否正常运行
volatile uint32_t g_main_loop_watchdog = 0;
#define MAIN_LOOP_TIMEOUT_MS  500  // 主循环超过500ms没响应则认为卡死

void SysTick_Handler(void)
{
	float dt = 0.002f;  // 修正：SysTick 实际是 2ms 周期
  g_systick_ms += 2;
	LSM6DSR_ReadData(&LSE6DSR_data);
	LSM6DSR_ConvertToPhysics(&LSE6DSR_data);
	
	// 串口接收超时看门狗（已屏蔽，避免影响本地模式）
	uart_rev_tiem ++;
	/* 
	if(uart_rev_tiem > 250)
	{
		star_car = 0; 
		uart_rev_tiem = 250;
		Motor_Disable();
	}
	*/
//	// 更新旧的角度计算（用于兼容性，使用转换后的角速度）
	add_angle += LSE6DSR_data.gy_rads * dt;  // 使用弧度/秒，正确积分
	add_angle_num += 1.0f;
	
	// 保留旧的姿态解算调用（可选，用于对比）
//	prepare_data();
//	imuupdate(&gyr_rad, &acc_g, &att_angle);
	
  ABEncoder_UpdateSpeed();
  MuxADC_SampleAll();
  if(!g_bt_key_control_mode && PID_PositionLoop_IsEnabled())
  {
    // 检查是否为模拟模式
    if(BlackPoint_Finder_IsSimulateMode())
    {
      // 使用模拟ADC值进行黑点搜索
      BlackPoint_Finder_Search(BlackPoint_Finder_GetSimulateADC(), &result_BlackPoint);
    }
    else
    {
      // 正常模式：使用真实ADC值
      BlackPoint_Finder_Search(g_mux_adc_values, &result_BlackPoint);
    }
    
    if(result_BlackPoint.found)
    {
      position_get = result_BlackPoint.precise_position * 10.0f;
      if(g_lose_time > 0)
      {
        g_lose_time--;
      }
    }
    else
    {
      g_lose_time ++;
      // 这里的 lose_time 计数频率 = SysTick频率(500Hz)
      // 50 * 2ms = 100ms 超时停机 (原先是500 -> 1s)
      if(g_lose_time > 250) 
      {
        g_lose_time = 50;
        Motor_Disable();
        Motor_StopAll(); // 确保 PWM 也置 0
        SpeedPID_ResetState(); // 确保 PID 复位
        // 关键修改：丢线后不退出 star_car 模式，不关风扇，不关灯
        // 保持 star_car = 1，风扇继续转，等待用户手动按 K1 退出
        // star_car = 0; 
        // RGB_SetColor(RGB_COLOR_OFF); 
      }
    }
  }
  // 必须将 PID_Control_Update 移出 if 条件
  // 因为自动整定需要在没有 PositionLoopEnabled 的情况下运行
  // 且需要在中断中获得精确的 2ms 周期
  PID_Control_Update();
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
