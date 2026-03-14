/**
 * @file    CircleHandler.c
 * @brief   圆环元素检测与处理模块实现
 * @details 7段式流程（右环→三横线→里程段→左环→停车），
 *          基于传感器分布特征检测 + 陀螺仪角度积分
 *
 *          流程:
 *          1. 发车+直线（1s后允许右环入口检测）
 *          2. 右环段
 *          3. 直线及缓曲线（横穿横线计数3次后进入第4段）
 *          4. 急折线段（进入时里程清零，里程>6000mm进入第5段）
 *          5. 直线及换曲线段（检测左环入口特征后进入第6段）
 *          6. 左环段
 *          7. 连续直角弯+停车（仅第7段启用并行停车检测）
 */

#include "CircleHandler.h"
#include "BlackPoint_Finder.h"
#include "LSM6DSR_Config.h"
#include "ADC_get.h"
#include "PID_Controller.h"
#include "Motor_ctr.h"
#include "M3PWM.h"
#include "RGB_Led.h"
#include "Odometer.h"
#include "Uart_Config.h"
#include <math.h>
#include <stdio.h>

/* ========================================================================== */
/*                              外部变量引用                                   */
/* ========================================================================== */
extern int16_t position_get;
extern volatile uint32_t g_systick_ms;
extern volatile uint32_t g_lose_time;
extern uint8_t star_car;

/* ========================================================================== */
/*                              可调参数                                       */
/* ========================================================================== */

/* 入环检测 */
#define ENTRY_CONFIRM_CNT        12      /* 连续帧确认 */

/* 入环偏向引导 */
#define ENTERING_DURATION_MS     800     /* 半侧追线持续时间 (ms) */

/* 出环触发条件（当前临时改为仅光电判定，Yaw门槛保留待恢复） */
/* #define EXIT_TRIGGER_YAW_DEG           250.0f */
#define EXIT_TRIGGER_EDGE_BLACK_MIN       3       /* 目标侧边缘黑点最小数量 */

/* 右环直行触发（从ENTERING开始累计里程） */
#define EXIT_TRIGGER_RIGHT_DISTANCE_MM     3200.0f /* 右环触发直行最小总里程 */
#define EXIT_TRIGGER_RIGHT_DISTANCE_MAX_MM 3600.0f /* 右环触发直行最大总里程（超出强制触发） */
#define EXIT_TRIGGER_RIGHT_LEFTBLACK_MIN   3       /* 右环触发时左侧四路黑点下限 */
#define EXIT_TRIGGER_RIGHT_LEFTBLACK_MAX   4       /* 右环触发时左侧四路黑点上限 */
#define EXIT_TRIGGER_CONFIRM_CNT           3       /* 触发直行需连续满足帧数 */

/* 出环偏向引导 */
#define EXIT_DURATION_MS         1000    /* 半侧追线持续时间 (ms) */

/* 出环直行策略 */
#define EXIT_STRAIGHT_FINISH_BLACK_MIN     2   /* 退出直行时16路总黑点下限（防止单黑误触发） */
#define EXIT_STRAIGHT_FINISH_BLACK_MAX     3   /* 退出直行时16路总黑点上限 */
#define EXIT_STRAIGHT_FINISH_DISTANCE_MM   25.0f /* 退出直行最小里程(2.5cm) */
#define EXIT_STRAIGHT_TIMEOUT_MS           800 /* 直行超时兜底 */
#define EXIT_FORCE_POS_RIGHT_X10           75  /* 右环出环直行强制位置 */
#define EXIT_FORCE_POS_LEFT_X10            74  /* 左环出环直行强制位置 */

/* 陀螺仪积分 */
#define CIRCLE_DT                0.002f
#define CIRCLE_GYRO_DEADZONE     0.003f
#define CIRCLE_RAD_TO_DEG        57.2957795f

/* 流程时间参数 */
#define RIGHT_ENABLE_DELAY_MS    1000    /* 发车后 1s 开始检测右环 */
#define PARKING_ENABLE_DELAY_MS  2000    /* 左环出环后 2s 开始检测停车 */
#define PARKING_STOP_DURATION_MS 3000    /* 停车持续 3s */
#define CROSSLINE_TARGET_COUNT   3       /* 需要检测到的横线数量 */
#define SEG3_CROSSLINE_TARGET_COUNT 3    /* 第3段切换到第4段需要横穿次数 */

/* ========================================================================== */
/*                              序列阶段枚举                                   */
/* ========================================================================== */
typedef enum {
	PHASE_SEG1_START_STRAIGHT = 0,   /* 第1段：发车+直线 */
	PHASE_SEG2_RIGHT_CIRCLE,         /* 第2段：右环 */
	PHASE_SEG3_STRAIGHT_CURVE,       /* 第3段：直线+缓曲线 */
	PHASE_SEG4_SHARP_LINE,           /* 第4段：急折线 */
	PHASE_SEG5_STRAIGHT_SWITCH,      /* 第5段：直线+换曲线 */
	PHASE_SEG6_LEFT_CIRCLE,          /* 第6段：左环 */
	PHASE_SEG7_RIGHT_ANGLE_PARK      /* 第7段：连续直角弯+停车 */
} CirclePhase_t;

/* ========================================================================== */
/*                              内部状态变量                                   */
/* ========================================================================== */
static CircleState_t s_state = CIRCLE_STATE_NORMAL;

/* 序列阶段 */
static CirclePhase_t s_phase = PHASE_SEG1_START_STRAIGHT;
static uint8_t s_current_dir = CIRCLE_DIR_RIGHT;

/* 入环检测消抖 */
static uint8_t  s_entry_confirm_cnt = 0;

/* 角度积分 */
static float s_accumulated_angle = 0.0f;

/* 入环/出环计时 */
static uint32_t s_enter_start_ms = 0;
static uint32_t s_exit_start_ms = 0;
static uint32_t s_exit_straight_start_ms = 0;

/* 时序计时 */
static uint32_t s_car_start_ms = 0;       /* star_car 启动时间 */
static uint32_t s_seg7_start_ms = 0;      /* 第7段起始时间（停车检测门控） */

/* 第3段横线计数切段 */
static uint8_t s_seg3_crossline_count = 0;
static uint8_t s_seg3_on_crossline = 0;

/* 上一次 star_car 状态 (边沿检测) */
static uint8_t s_was_running = 0;

/* 停车检测 */
static uint8_t s_parking_detection_enabled = 0;
static uint8_t s_crossline_count = 0;
static uint8_t s_on_crossline = 0;        /* 当前正在横线上 (消抖) */
static uint8_t s_parking_stopped = 0;     /* 已触发停车 */
static uint32_t s_parking_stop_ms = 0;    /* 停车开始时间 */

/* 调试打印限流 */
static uint32_t s_last_print_ms = 0;

/* 出环直行控制 */
static uint8_t s_exit_straight_active = 0;
static uint8_t s_exit_trigger_confirm_cnt = 0;
static uint8_t s_exit_finish_confirm_cnt = 0;

/**
 * @brief  打印16路光电黑白状态（BLK:0101...）
 */
static void Circle_LogBlackFlags(const char *tag, volatile uint16_t *adc_values)
{
	uint8_t black_flags[SENSOR_COUNT];
	char bits[SENSOR_COUNT + 1];
	uint8_t i;

	if(adc_values == NULL)
	{
		printf("[CIRCLE] %s BLK:NULL\r\n", tag);
		return;
	}

	BlackPoint_Finder_GetBlackFlags_Dynamic(adc_values, black_flags);
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		bits[i] = black_flags[i] ? '1' : '0';
	}
	bits[SENSOR_COUNT] = '\0';
	printf("[CIRCLE] %s BLK:%s\r\n", tag, bits);
}

/**
 * @brief  发送分段结束事件到蓝牙串口
 * @param  seg_idx 段号(1~7)
 */
static void Circle_SendSegmentEndEvent(uint8_t seg_idx)
{
	char msg[24];
	sprintf(msg, "EVT:SEG_END:%d\r\n", seg_idx);
	Uart2_SendString(msg);
}

/* ========================================================================== */
/*                              公有函数实现                                   */
/* ========================================================================== */

void Circle_Init(void)
{
	s_state = CIRCLE_STATE_NORMAL;
	s_phase = PHASE_SEG1_START_STRAIGHT;
	s_current_dir = CIRCLE_DIR_RIGHT;
	s_entry_confirm_cnt = 0;
	s_accumulated_angle = 0.0f;
	s_enter_start_ms = 0;
	s_exit_start_ms = 0;
	s_exit_straight_start_ms = 0;
	s_car_start_ms = 0;
	s_seg7_start_ms = 0;
	s_seg3_crossline_count = 0;
	s_seg3_on_crossline = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_parking_stop_ms = 0;
	s_last_print_ms = 0;
	s_exit_straight_active = 0;
	s_exit_trigger_confirm_cnt = 0;
	s_exit_finish_confirm_cnt = 0;
	printf("[CIRCLE] Init: RIGHT then LEFT\r\n");
}

void Circle_Reset(void)
{
	if(s_state != CIRCLE_STATE_NORMAL)
	{
		printf("[CIRCLE] Reset (was %d)\r\n", s_state);
	}
	s_state = CIRCLE_STATE_NORMAL;
	s_phase = PHASE_SEG1_START_STRAIGHT;
	s_current_dir = CIRCLE_DIR_RIGHT;
	s_entry_confirm_cnt = 0;
	s_accumulated_angle = 0.0f;
	s_enter_start_ms = 0;
	s_exit_start_ms = 0;
	s_exit_straight_start_ms = 0;
	s_car_start_ms = 0;
	s_seg7_start_ms = 0;
	s_seg3_crossline_count = 0;
	s_seg3_on_crossline = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_stopped = 0;
	s_last_print_ms = 0;
	s_exit_straight_active = 0;
	s_exit_trigger_confirm_cnt = 0;
	s_exit_finish_confirm_cnt = 0;
}

void Circle_SetDirection(uint8_t dir)
{
	(void)dir; /* 方向由阶段自动决定 */
}

uint8_t Circle_GetDirection(void)
{
	return s_current_dir;
}

CircleState_t Circle_GetState(void)
{
	return s_state;
}

uint8_t Circle_IsActive(void)
{
	return (s_state != CIRCLE_STATE_NORMAL) ? 1 : 0;
}

float Circle_GetAccumulatedAngle(void)
{
	return fabsf(s_accumulated_angle);
}

/**
 * @brief  检测左环入口特征
 * @param  adc_values  传感器 ADC 值数组
 * @return 1=符合左环入口特征, 0=不符合
 */
static uint8_t Circle_DetectLeftEntry(volatile uint16_t *adc_values)
{
	uint8_t left_count = 0;
	uint8_t right_count = 0;
	uint8_t center_count = 0;
	uint8_t i;
	
	/* 左侧5路 */
	for(i = 0; i <= 4; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			left_count++;
	}
	
	/* 右侧4路 */
	for(i = 12; i <= 15; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			right_count++;
	}
	
	/* 中间区域 */
	for(i = 6; i <= 9; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			center_count++;
	}
	
	return (left_count >= 3 && right_count == 0 && center_count >= 1) ? 1 : 0;
}

/**
 * @brief  检测右环入口特征 (左环的镜像)
 * @param  adc_values  传感器 ADC 值数组
 * @return 1=符合右环入口特征, 0=不符合
 */
static uint8_t Circle_DetectRightEntry(volatile uint16_t *adc_values)
{
	uint8_t right_count = 0;
	uint8_t left_count = 0;
	uint8_t center_count = 0;
	uint8_t i;
	
	/* 右侧5路 (11~15) */
	for(i = 11; i <= 15; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			right_count++;
	}
	
	/* 左侧4路 (0~3) */
	for(i = 0; i <= 3; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			left_count++;
	}
	
	/* 中间区域 (6~9) */
	for(i = 6; i <= 9; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
			center_count++;
	}
	
	return (right_count >= 3 && left_count == 0 && center_count >= 1) ? 1 : 0;
}

/**
 * @brief  检测横线 (十字路口停车区域)
 * @param  adc_values  传感器 ADC 值数组
 * @return 1=检测到横线, 0=未检测到
 * @details 利用传感器圆弧形状特征:
 *          对称外侧传感器对同时检测到黑线 → 横线
 *          检查 (0,15), (1,14), (2,13) 三对, 至少 2 对同时为黑
 */
static uint8_t Circle_DetectCrossLine(volatile uint16_t *adc_values)
{
	uint8_t pair_count = 0;
	
	if(BlackPoint_Finder_IsBlackPoint(0, adc_values[0]) &&
	   BlackPoint_Finder_IsBlackPoint(15, adc_values[15]))
		pair_count++;
	
	if(BlackPoint_Finder_IsBlackPoint(1, adc_values[1]) &&
	   BlackPoint_Finder_IsBlackPoint(14, adc_values[14]))
		pair_count++;
	
	if(BlackPoint_Finder_IsBlackPoint(2, adc_values[2]) &&
	   BlackPoint_Finder_IsBlackPoint(13, adc_values[13]))
		pair_count++;
	
	return (pair_count >= 2) ? 1 : 0;
}

/**
 * @brief  统计出环判定用黑点数（仅边缘四路）
 * @note   左侧仅统计 0~3，右侧仅统计 12~15
 */
static void Circle_CountBlack(volatile uint16_t *adc_values, uint8_t *left_black, uint8_t *right_black, uint8_t *total_black)
{
	uint8_t i;
	uint8_t l = 0;
	uint8_t r = 0;
	uint8_t t = 0;

	if(adc_values == NULL)
	{
		if(left_black) *left_black = 0;
		if(right_black) *right_black = 0;
		if(total_black) *total_black = 0;
		return;
	}

	for(i = 0; i < 16; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
		{
			if(i <= 3)
			{
				l++;
				t++;
			}
			else if(i >= 12)
			{
				r++;
				t++;
			}
		}
	}

	if(left_black) *left_black = l;
	if(right_black) *right_black = r;
	if(total_black) *total_black = t;
}

/**
 * @brief  统计16路总黑点数量
 */
static uint8_t Circle_CountBlackAll16(volatile uint16_t *adc_values)
{
	uint8_t i;
	uint8_t total = 0;

	if(adc_values == NULL) return 0;

	for(i = 0; i < 16; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
		{
			total++;
		}
	}

	return total;
}

/**
 * @brief  完成当前方向的出环流程并恢复 NORMAL
 */
static void Circle_CompleteExit(void)
{
	s_state = CIRCLE_STATE_NORMAL;
	s_accumulated_angle = 0.0f;
	s_exit_straight_active = 0;
	s_exit_trigger_confirm_cnt = 0;
	s_exit_finish_confirm_cnt = 0;
	s_exit_straight_start_ms = 0;
	PositionPID_ResetState();
	printf("[CIRCLE] EXIT COMPLETE (%s)\r\n",
	       (s_current_dir == CIRCLE_DIR_RIGHT) ? "RIGHT" : "LEFT");

	if(s_current_dir == CIRCLE_DIR_RIGHT)
	{
		Circle_SendSegmentEndEvent(2);
		s_phase = PHASE_SEG3_STRAIGHT_CURVE;
		s_seg3_crossline_count = 0;
		s_seg3_on_crossline = 0;
		printf("[CIRCLE] << RIGHT DONE, to SEG3\r\n");
	}
	else
	{
		Circle_SendSegmentEndEvent(6);
		s_phase = PHASE_SEG7_RIGHT_ANGLE_PARK;
		s_seg7_start_ms = g_systick_ms;
		s_parking_detection_enabled = 0;
		s_crossline_count = 0;
		s_on_crossline = 0;
		printf("[CIRCLE] << LEFT DONE, to SEG7\r\n");
	}
}

/**
 * @brief  圆环状态机主更新函数
 */
void Circle_Update(volatile uint16_t *adc_values)
{
	BlackPointResult_t half_result;
	float gz_filtered;
	
	if(adc_values == NULL) return;
	
	/* ======== 停车状态处理 (最高优先级) ======== */
	if(s_parking_stopped)
	{
		if((g_systick_ms - s_parking_stop_ms) >= PARKING_STOP_DURATION_MS)
		{
			Circle_SendSegmentEndEvent(7);
			/* 3s 停车完成, 退出巡线模式 */
			s_parking_stopped = 0;
			star_car = 0;
			WheelLock_Disable();
			Motor_StopAll();
			Motor_Disable();
			M3PWM_SetDutyCycle(0);  /* 关闭负压风扇 */
			RGB_SetColor(RGB_COLOR_OFF);
			PID_PositionLoop_Enable(0);
			SpeedPID_ResetState();
			printf("[PARKING] 3s done, exit line-following\r\n");
		}
		return;
	}
	
	/* ======== 急停/退出保护 ======== */
	if(!star_car)
	{
		if(s_state != CIRCLE_STATE_NORMAL)
		{
			s_state = CIRCLE_STATE_NORMAL;
			s_entry_confirm_cnt = 0;
			s_accumulated_angle = 0.0f;
			s_exit_straight_active = 0;
			s_exit_trigger_confirm_cnt = 0;
			s_exit_finish_confirm_cnt = 0;
			s_exit_straight_start_ms = 0;
			PositionPID_ResetState();
		}
		s_was_running = 0;
		return;
	}
	
	/* ======== 启动边沿检测: star_car 0→1 ======== */
	if(!s_was_running)
	{
		s_was_running = 1;
		s_car_start_ms = g_systick_ms;
		s_phase = PHASE_SEG1_START_STRAIGHT;
		s_seg7_start_ms = 0;
		s_seg3_crossline_count = 0;
		s_seg3_on_crossline = 0;
		s_parking_detection_enabled = 0;
		s_crossline_count = 0;
		s_on_crossline = 0;
		printf("[CIRCLE] Car started, enter SEG1\r\n");
	}
	
	/* ======== 停车区域检测 (仅第7段并行启用) ======== */
	if(s_phase == PHASE_SEG7_RIGHT_ANGLE_PARK &&
	   s_parking_detection_enabled &&
	   s_state == CIRCLE_STATE_NORMAL)
	{
		uint8_t cross = Circle_DetectCrossLine(adc_values);
		
		if(cross)
		{
			if(!s_on_crossline)
			{
				/* 上升沿: 进入横线 */
				s_on_crossline = 1;
				s_crossline_count++;
				printf("[PARKING] Crossline %d/%d\r\n", s_crossline_count, CROSSLINE_TARGET_COUNT);
				
				if(s_crossline_count >= CROSSLINE_TARGET_COUNT)
				{
					/* 达到 3 根横线, 立即抱死停车 */
					s_parking_stopped = 1;
					s_parking_stop_ms = g_systick_ms;
					s_parking_detection_enabled = 0;
					WheelLock_Enable();
					printf("[PARKING] 3 lines reached, STOP!\r\n");
					return;
				}
			}
		}
		else
		{
			s_on_crossline = 0;
		}
	}
	
	/* ======== 停车检测启用定时 (第7段进入后) ======== */
	if(s_phase == PHASE_SEG7_RIGHT_ANGLE_PARK && !s_parking_detection_enabled)
	{
		if((g_systick_ms - s_seg7_start_ms) >= PARKING_ENABLE_DELAY_MS)
		{
			s_parking_detection_enabled = 1;
			s_crossline_count = 0;
			s_on_crossline = 0;
			printf("[PARKING] Detection enabled\r\n");
		}
	}
	
	/* ======== 陀螺仪死区滤波 ======== */
	gz_filtered = LSE6DSR_data.gz_rads;
	if(gz_filtered > -CIRCLE_GYRO_DEADZONE && gz_filtered < CIRCLE_GYRO_DEADZONE)
	{
		gz_filtered = 0.0f;
	}
	
	/* ======== 状态机 ======== */
	switch(s_state)
	{
		/* ================================================================ */
		case CIRCLE_STATE_NORMAL:
		{
			/* 根据当前阶段决定行为 */
			switch(s_phase)
			{
					case PHASE_SEG1_START_STRAIGHT:
						/* 第1段：发车+直线，1s后允许检测右环入口 */
						if((g_systick_ms - s_car_start_ms) < RIGHT_ENABLE_DELAY_MS)
						{
							s_entry_confirm_cnt = 0;
							break;
						}

						/* 结束条件：右环入口特征成立 */
					if(Circle_DetectRightEntry(adc_values))
					{
						s_entry_confirm_cnt++;
						if(s_entry_confirm_cnt >= ENTRY_CONFIRM_CNT)
						{
							Circle_SendSegmentEndEvent(1);
							s_state = CIRCLE_STATE_ENTERING;
							s_current_dir = CIRCLE_DIR_RIGHT;
								s_phase = PHASE_SEG2_RIGHT_CIRCLE;
							s_enter_start_ms = g_systick_ms;
							s_accumulated_angle = 0.0f;
							s_entry_confirm_cnt = 0;
							Odometer_Reset(); /* 右环：从入环瞬间开始累计里程 */
							LSM6DSR_ClearYaw();
								printf("[CIRCLE] SEG1->SEG2, ENTERING (RIGHT)\r\n");
							Circle_LogBlackFlags("ENTERING(R)", adc_values);
						}
					}
					else
					{
						s_entry_confirm_cnt = 0;
					}
					break;

					case PHASE_SEG2_RIGHT_CIRCLE:
						/* 第2段：右环（由圆环内状态机驱动），切段在 Circle_CompleteExit 中完成 */
						break;

					case PHASE_SEG3_STRAIGHT_CURVE:
						/* 第3段：横穿横线计数3次后进入第4段 */
						if(Circle_DetectCrossLine(adc_values))
						{
							if(!s_seg3_on_crossline)
							{
								s_seg3_on_crossline = 1;
								s_seg3_crossline_count++;
								printf("[SEG3] Crossline %d/%d\r\n",
								       s_seg3_crossline_count,
								       SEG3_CROSSLINE_TARGET_COUNT);
								if(s_seg3_crossline_count >= SEG3_CROSSLINE_TARGET_COUNT)
								{
									Circle_SendSegmentEndEvent(3);
									s_phase = PHASE_SEG4_SHARP_LINE;
									Odometer_Reset();
									printf("[SEG3->SEG4] Odometer reset\r\n");
								}
							}
						}
						else
						{
							s_seg3_on_crossline = 0;
						}
						break;

					case PHASE_SEG4_SHARP_LINE:
						/* 第4段：里程 > 6000mm 后进入第5段 */
						if(Odometer_GetLocation() > 6000.0f)
						{
							Circle_SendSegmentEndEvent(4);
							s_phase = PHASE_SEG5_STRAIGHT_SWITCH;
							printf("[SEG4->SEG5] dist=%.1f\r\n", Odometer_GetLocation());
						}
						break;

					case PHASE_SEG5_STRAIGHT_SWITCH:
						/* 第5段结束条件：检测到左环入口特征 */
					if(Circle_DetectLeftEntry(adc_values))
					{
						s_entry_confirm_cnt++;
						if(s_entry_confirm_cnt >= ENTRY_CONFIRM_CNT)
						{
							Circle_SendSegmentEndEvent(5);
							s_state = CIRCLE_STATE_ENTERING;
							s_current_dir = CIRCLE_DIR_LEFT;
								s_phase = PHASE_SEG6_LEFT_CIRCLE;
							s_enter_start_ms = g_systick_ms;
							s_accumulated_angle = 0.0f;
							s_entry_confirm_cnt = 0;
							LSM6DSR_ClearYaw();
								printf("[SEG5->SEG6] ENTERING (LEFT)\r\n");
							Circle_LogBlackFlags("ENTERING(L)", adc_values);
						}
					}
					else
					{
						s_entry_confirm_cnt = 0;
					}
					break;

					case PHASE_SEG6_LEFT_CIRCLE:
						/* 第6段：左环（由圆环内状态机驱动），切段在 Circle_CompleteExit 中完成 */
						break;

					case PHASE_SEG7_RIGHT_ANGLE_PARK:
						/* 第7段：连续直角弯+停车，停车检测并行逻辑已在上方执行 */
					break;
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_ENTERING:
		{
			/* 入环引导: 右环→右半侧追线(side=1), 左环→左半侧追线(side=0) */
			uint8_t enter_side = (s_current_dir == CIRCLE_DIR_RIGHT) ? 1 : 0;
			BlackPoint_Finder_SearchHalf(adc_values, enter_side, &half_result);
			if(half_result.found)
			{
				position_get = (int16_t)(half_result.precise_position * 10.0f);
				g_lose_time = 0;
			}
			else
			{
				/* 半侧丢线, 全幅兜底 */
				BlackPointResult_t full_result;
				BlackPoint_Finder_Search(adc_values, &full_result);
				if(full_result.found)
				{
					position_get = (int16_t)(full_result.precise_position * 10.0f);
					g_lose_time = 0;
				}
			}
			
			/* 积分角度 */
			s_accumulated_angle += gz_filtered * CIRCLE_DT * CIRCLE_RAD_TO_DEG;
			
			/* 入环引导时间到 → IN_CIRCLE */
			if((g_systick_ms - s_enter_start_ms) >= ENTERING_DURATION_MS)
			{
				s_state = CIRCLE_STATE_IN_CIRCLE;
				printf("[CIRCLE] >> IN_CIRCLE\r\n");
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_IN_CIRCLE:
		{
			uint8_t left_black = 0;
			uint8_t right_black = 0;
			uint8_t total_black = 0;
			uint8_t target_side_black = 0;
			float in_circle_distance_mm = 0.0f;
			uint8_t right_window_trigger = 0;
			uint8_t right_force_trigger = 0;

			/* 正常全幅巡线 + 角度积分 */
			s_accumulated_angle += gz_filtered * CIRCLE_DT * CIRCLE_RAD_TO_DEG;
			Circle_CountBlack(adc_values, &left_black, &right_black, &total_black);
			target_side_black = (s_current_dir == CIRCLE_DIR_RIGHT) ? left_black : right_black;
			in_circle_distance_mm = Odometer_GetLocation();

			right_window_trigger = (s_current_dir == CIRCLE_DIR_RIGHT &&
			                        in_circle_distance_mm > EXIT_TRIGGER_RIGHT_DISTANCE_MM &&
			                        in_circle_distance_mm < EXIT_TRIGGER_RIGHT_DISTANCE_MAX_MM &&
			                        left_black >= EXIT_TRIGGER_RIGHT_LEFTBLACK_MIN &&
			                        left_black <= EXIT_TRIGGER_RIGHT_LEFTBLACK_MAX) ? 1 : 0;

			right_force_trigger = (s_current_dir == CIRCLE_DIR_RIGHT &&
			                       in_circle_distance_mm >= EXIT_TRIGGER_RIGHT_DISTANCE_MAX_MM) ? 1 : 0;

			/* 右环：3200~3600窗口内按光电判定；>=3600强制触发。左环保持原逻辑 */
			if(right_window_trigger ||
			   right_force_trigger ||
			   (s_current_dir == CIRCLE_DIR_LEFT &&
			    target_side_black >= EXIT_TRIGGER_EDGE_BLACK_MIN))
			{
				if(right_force_trigger)
				{
					s_exit_trigger_confirm_cnt = EXIT_TRIGGER_CONFIRM_CNT;
				}
				else
				{
					s_exit_trigger_confirm_cnt++;
				}
				if(s_exit_trigger_confirm_cnt >= EXIT_TRIGGER_CONFIRM_CNT)
				{
					s_state = CIRCLE_STATE_EXITING;
					s_exit_start_ms = g_systick_ms;
					s_exit_straight_active = 1;
					s_exit_finish_confirm_cnt = 0;
					s_exit_straight_start_ms = g_systick_ms;
					position_get = (s_current_dir == CIRCLE_DIR_RIGHT) ? EXIT_FORCE_POS_RIGHT_X10 : EXIT_FORCE_POS_LEFT_X10;
					g_lose_time = 0;
					PositionPID_ResetState();
					if(s_current_dir == CIRCLE_DIR_LEFT)
					{
						/* 左环维持现有逻辑：触发直行时清零里程用于后续退出判定 */
						Odometer_Reset();
					}
					printf("[CIRCLE] >> EXITING STRAIGHT dir=%s side=%d dist=%.1f\r\n",
					       (s_current_dir == CIRCLE_DIR_RIGHT) ? "R" : "L",
					       target_side_black,
					       in_circle_distance_mm);
					Circle_LogBlackFlags("EXIT_TRIG", adc_values);
				}
			}
			else
			{
				s_exit_trigger_confirm_cnt = 0;
			}
			break;
		}
		
		/* ================================================================ */
		case CIRCLE_STATE_EXITING:
		{
			uint8_t left_black = 0;
			uint8_t right_black = 0;
			uint8_t total_black = 0;
			uint8_t target_side_black = 0;

			/* 角度继续积分 (调试用) */
			s_accumulated_angle += gz_filtered * CIRCLE_DT * CIRCLE_RAD_TO_DEG;
			Circle_CountBlack(adc_values, &left_black, &right_black, &total_black);
			target_side_black = (s_current_dir == CIRCLE_DIR_RIGHT) ? left_black : right_black;
			
			/* 直行阶段未触发前，仍用半侧追线引导 */
			if(!s_exit_straight_active)
			{
				/* 出环引导: 右环→左半侧追线(side=0), 左环→右半侧追线(side=1) */
				{
					uint8_t exit_side = (s_current_dir == CIRCLE_DIR_RIGHT) ? 0 : 1;
					BlackPoint_Finder_SearchHalf(adc_values, exit_side, &half_result);
				}
				if(half_result.found)
				{
					position_get = (int16_t)(half_result.precise_position * 10.0f);
					g_lose_time = 0;
				}
				else
				{
					/* 半侧丢线, 全幅兜底 */
					BlackPointResult_t full_result;
					BlackPoint_Finder_Search(adc_values, &full_result);
					if(full_result.found)
					{
						position_get = (int16_t)(full_result.precise_position * 10.0f);
						g_lose_time = 0;
					}
				}

				/* 兜底触发：EXITING但尚未直行时，改为同样光电单判定切入直行 */
				if(target_side_black >= EXIT_TRIGGER_EDGE_BLACK_MIN)
				{
					s_exit_straight_active = 1;
					s_exit_straight_start_ms = g_systick_ms;
					s_exit_finish_confirm_cnt = 0;
					position_get = (s_current_dir == CIRCLE_DIR_RIGHT) ? EXIT_FORCE_POS_RIGHT_X10 : EXIT_FORCE_POS_LEFT_X10;
					g_lose_time = 0;
					PositionPID_ResetState();
					Odometer_Reset();
					printf("[CIRCLE] EXIT STRAIGHT ON dir=%s side=%d\r\n",
					       (s_current_dir == CIRCLE_DIR_RIGHT) ? "R" : "L",
					       target_side_black);
					Circle_LogBlackFlags("EXIT_TRIG", adc_values);
				}
			}
			else
			{
				uint8_t total_black_all;
				float exit_distance_mm;

				/* 直行阶段：忽略光电干涉，强制中心位置 */
				position_get = (s_current_dir == CIRCLE_DIR_RIGHT) ? EXIT_FORCE_POS_RIGHT_X10 : EXIT_FORCE_POS_LEFT_X10;
				g_lose_time = 0;
				total_black_all = Circle_CountBlackAll16(adc_values);
				exit_distance_mm = Odometer_GetLocation();

				/* 退出双判定：里程>5cm 且16路总黑在2~3 */
				if(exit_distance_mm > EXIT_STRAIGHT_FINISH_DISTANCE_MM &&
				   total_black_all >= EXIT_STRAIGHT_FINISH_BLACK_MIN &&
				   total_black_all <= EXIT_STRAIGHT_FINISH_BLACK_MAX)
				{
					printf("[CIRCLE] EXIT STRAIGHT OFF dist=%.1f total16=%d\r\n",
					       exit_distance_mm,
					       total_black_all);
					Circle_LogBlackFlags("EXIT_DONE", adc_values);
					Circle_CompleteExit();
				}

				/* 超时兜底 */
				if(s_state == CIRCLE_STATE_EXITING &&
				   (g_systick_ms - s_exit_straight_start_ms) >= EXIT_STRAIGHT_TIMEOUT_MS)
				{
					printf("[CIRCLE] EXIT STRAIGHT TIMEOUT\r\n");
					Circle_CompleteExit();
				}
			}
			break;
		}
		
		default:
			s_state = CIRCLE_STATE_NORMAL;
			break;
	}
}
