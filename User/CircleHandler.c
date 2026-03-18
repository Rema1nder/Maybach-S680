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
#define EXIT_TRIGGER_RIGHT_DISTANCE_MAX_MM 3900.0f /* 右环触发直行最大总里程（超出强制触发） */
#define EXIT_TRIGGER_RIGHT_LEFTBLACK_MIN   3       /* 右环触发时左侧四路黑点下限 */
#define EXIT_TRIGGER_RIGHT_LEFTBLACK_MAX   4       /* 右环触发时左侧四路黑点上限 */
#define EXIT_TRIGGER_LEFT_DISTANCE_MM      4000.0f /* 左环触发直行最小总里程 */
#define EXIT_TRIGGER_LEFT_DISTANCE_MAX_MM  5000.0f /* 左环触发直行最大总里程 */
#define EXIT_TRIGGER_CONFIRM_CNT           3       /* 触发直行需连续满足帧数 */

/* 出环偏向引导 */
#define EXIT_DURATION_MS         1000    /* 半侧追线持续时间 (ms) */

/* 出环直行策略 */
#define EXIT_STRAIGHT_FINISH_BLACK_MIN     2   /* 退出直行时16路总黑点下限（防止单黑误触发） */
#define EXIT_STRAIGHT_FINISH_BLACK_MAX     3   /* 退出直行时16路总黑点上限 */
#define EXIT_STRAIGHT_FINISH_DISTANCE_RIGHT_MM   50.0f  /* 右环退出直行最小里程 */
#define EXIT_STRAIGHT_FINISH_DISTANCE_LEFT_MM    200.0f /* 左环退出直行最小里程 */
#define EXIT_STRAIGHT_FINISH_CONFIRM_CNT         3      /* 退出直行需连续满足帧数 */
#define EXIT_STRAIGHT_TIMEOUT_RIGHT_MS           2000    /* 右环直行超时兜底 */
#define EXIT_STRAIGHT_TIMEOUT_LEFT_MS            2000   /* 左环直行超时兜底 */
#define EXIT_FORCE_POS_RIGHT_X10           135  /* 右环出环直行强制位置 */
#define EXIT_FORCE_POS_LEFT_X10            45  /* 左环出环直行强制位置 */

/* 陀螺仪积分 */
#define CIRCLE_DT                0.002f
#define CIRCLE_GYRO_DEADZONE     0.003f
#define CIRCLE_RAD_TO_DEG        57.2957795f
#define CIRCLE_ODO_VALID_MAX_MM  200000.0f
#define CIRCLE_GYRO_VALID_MAX_DPS 1000.0f
#define TURN90_DBG_PERIOD_MS     200U

/* 流程时间参数 */
#define RIGHT_ENABLE_DELAY_MS    1000    /* 发车后 1s 开始检测右环 */
#define PARKING_ENABLE_DELAY_MS  2000    /* 左环出环后 2s 开始检测停车 */
#define PARKING_STOP_DURATION_MS 3000    /* 停车持续 3s */
#define CROSSLINE_TARGET_COUNT   4       /* 需要检测到的横线数量 */
#define PARKING_CROSSLINE_CONFIRM_CNT 3  /* 停车横线连续判定帧数 */
#define SEG3_CROSSLINE_TARGET_COUNT 3    /* 第3段切换到第4段需要横穿次数 */
#define SEG3_CROSSLINE_ENABLE_DISTANCE_MM 10000.0f /* 第3段累计里程达到后才开始横线计数 */
#define SEG3_CROSSLINE_CONFIRM_CNT 4      /* 第3段横线连续判定帧数 */
#define SEG3_CROSSLINE_MIN_GAP_MM  500.0f /* 第3段相邻横线最小间隔里程 */
#define SEG3_MASK_WHITE_ADC_VALUE         4095U
#define SEG3_CROSSLINE_ENABLE_ACCEL_TARGET_SPEED 150.0f
#define SEG3_CROSSLINE_ENABLE_ACCEL_DISTANCE_MM  1000.0f

/* 速度时序参数 */
#define SEG1_BASE_SPEED                    150.0f
#define SEG2_DECEL_TRIGGER_DISTANCE_MM     2500.0f
#define SEG2_DECEL_DISTANCE_MM             800.0f
#define SEG2_DECEL_TARGET_SPEED            110.0f
#define SEG3_TURN_DECEL_TARGET_SPEED       90.0f
#define SEG3_TURN_DECEL_DISTANCE_MM        1000.0f
#define SEG3_TURN_ACCEL_TARGET_SPEED       110.0f
#define SEG3_TURN_ACCEL_DISTANCE_MM        1000.0f
#define SEG4_HOLD_DISTANCE_MM              3000.0f
#define SEG4_DECEL_TARGET_SPEED            110.0f
#define SEG4_DECEL_DISTANCE_MM             1000.0f
#define SEG6_BASE_SPEED                    120.0f
#define SEG6_DECEL_TRIGGER_DISTANCE_MM     2000.0f
#define SEG6_DECEL_DISTANCE_MM             1000.0f
#define SEG6_DECEL_TARGET_SPEED            90.0f
#define SEG7_DECEL_TARGET_SPEED            85.0f
#define SEG7_DECEL_DISTANCE_MM             1000.0f

/* 第7段直角弯控制参数 */
#define RIGHT_ANGLE_DONE_YAW_DEG          75.0f
#define RIGHT_ANGLE_CENTER_CONFIRM_CNT     2

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
static uint8_t s_seg3_crossline_enabled = 0;
static uint8_t s_seg3_crossline_confirm_cnt = 0;
static float s_seg3_start_odo_mm = 0.0f;
static float s_seg3_last_crossline_odo_mm = 0.0f;
static uint8_t s_seg3_turns_done = 0;
static uint8_t s_seg3_turn_sequence_active = 0;
static uint8_t s_seg3_turn_decel_done = 0;
static uint8_t s_seg3_accel_started = 0;
static uint8_t s_seg3_right_angle_detect_active = 0;
static uint8_t s_seg3_right_angle_done = 0;

/* 右环内减速门控 */
static uint8_t s_seg2_decel_started = 0;
static uint8_t s_seg2_decel_done = 0;

/* 第4段速度时序 */
static uint8_t s_seg4_decel_started = 0;
static uint8_t s_seg4_decel_done = 0;

/* 第6段(左环)速度时序 */
static uint8_t s_seg6_decel_started = 0;
static uint8_t s_seg6_decel_done = 0;

/* 直角检测内部状态 */
static uint8_t s_right_angle_detect_started = 0;
static uint8_t s_right_angle_yaw_reached = 0;
static uint8_t s_right_angle_center_confirm_cnt = 0;

/* 上一次 star_car 状态 (边沿检测) */
static uint8_t s_was_running = 0;

/* 停车检测 */
static uint8_t s_parking_detection_enabled = 0;
static uint8_t s_crossline_count = 0;
static uint8_t s_on_crossline = 0;        /* 当前正在横线上 (消抖) */
static uint8_t s_parking_crossline_confirm_cnt = 0;
static uint8_t s_parking_stopped = 0;     /* 已触发停车 */
static uint32_t s_parking_stop_ms = 0;    /* 停车开始时间 */

/* 调试打印限流 */
static uint32_t s_last_print_ms = 0;

/* 出环直行控制 */
static uint8_t s_exit_straight_active = 0;
static uint8_t s_exit_trigger_confirm_cnt = 0;
static uint8_t s_exit_finish_confirm_cnt = 0;

/* 基础速度线性调节（按里程） */
static uint8_t s_base_speed_ramp_active = 0;
static float s_base_speed_ramp_start_base = 0.0f;
static float s_base_speed_ramp_target_base = 0.0f;
static float s_base_speed_ramp_distance_mm = 0.0f;
static float s_base_speed_ramp_start_odo_mm = 0.0f;
static float s_base_speed_ramp_speed_range = 0.0f;

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
	snprintf(msg, sizeof(msg), "EVT:SEG_END:%d\r\n", seg_idx);
	Uart2_SendString(msg);
}

/**
 * @brief  启动按里程线性调节基础速度
 * @param  target_base_speed   目标基础速度
 * @param  ramp_distance_mm    加/减速距离(mm)
 * @note   调用时读取当前基础速度，在指定里程内线性调到目标值
 */
static void Circle_StartBaseSpeedRampByDistance(float target_base_speed, float ramp_distance_mm)
{
	float cur_base = 0.0f;
	float cur_range = 0.0f;

	PID_GetLineSpeedParams(&cur_base, &cur_range);

	if(ramp_distance_mm <= 0.0f)
	{
		PID_SetLineSpeedParams(target_base_speed, cur_range);
		s_base_speed_ramp_active = 0;
		return;
	}

	s_base_speed_ramp_active = 1;
	s_base_speed_ramp_start_base = cur_base;
	s_base_speed_ramp_target_base = target_base_speed;
	s_base_speed_ramp_distance_mm = ramp_distance_mm;
	s_base_speed_ramp_start_odo_mm = Odometer_GetLocation();
	s_base_speed_ramp_speed_range = cur_range;

	printf("[SPD_RAMP] start %.1f->%.1f in %.1fmm\r\n",
	       s_base_speed_ramp_start_base,
	       s_base_speed_ramp_target_base,
	       s_base_speed_ramp_distance_mm);
}

/**
 * @brief  更新按里程线性调速过程
 */
static void Circle_UpdateBaseSpeedRampByDistance(void)
{
	float traveled_mm;
	float progress;
	float new_base;

	if(!s_base_speed_ramp_active) return;

	traveled_mm = Odometer_GetLocation() - s_base_speed_ramp_start_odo_mm;
	if(traveled_mm < 0.0f) traveled_mm = 0.0f;

	progress = traveled_mm / s_base_speed_ramp_distance_mm;
	if(progress >= 1.0f)
	{
		PID_SetLineSpeedParams(s_base_speed_ramp_target_base, s_base_speed_ramp_speed_range);
		s_base_speed_ramp_active = 0;
		{
			char evt[36];
			snprintf(evt, sizeof(evt), "EVT:SPD_RAMP_DONE:%d\r\n", (int)(s_base_speed_ramp_target_base + 0.5f));
			Uart2_SendString(evt);
		}
		printf("[SPD_RAMP] done base=%.1f\r\n", s_base_speed_ramp_target_base);
		return;
	}

	new_base = s_base_speed_ramp_start_base +
		(s_base_speed_ramp_target_base - s_base_speed_ramp_start_base) * progress;
	PID_SetLineSpeedParams(new_base, s_base_speed_ramp_speed_range);
}

/**
 * @brief  直角完成检测（调用触发后先清Yaw）
 * @param  adc_values  传感器ADC数组
 * @return 1=直角完成, 0=未完成
 */
static uint8_t Circle_DetectRightAngleDone(volatile uint16_t *adc_values)
{
	uint8_t center_black;
	float yaw;
	float yaw_abs;
	float gz_dps;
	static uint32_t s_turn90_dbg_last_ms = 0;

	if(adc_values == NULL) return 0;

	if(!s_right_angle_detect_started)
	{
		s_right_angle_detect_started = 1;
		s_right_angle_yaw_reached = 0;
		s_right_angle_center_confirm_cnt = 0;
		s_turn90_dbg_last_ms = g_systick_ms;
		Uart2_SendString("EVT:TURN90_DETECT_START\r\n");
		LSM6DSR_ClearYaw();
		printf("[TURN90] detect start, yaw cleared\r\n");
	}

	center_black = (BlackPoint_Finder_IsBlackPoint(7, adc_values[7]) ||
	                BlackPoint_Finder_IsBlackPoint(8, adc_values[8])) ? 1 : 0;
	yaw = LSM6DSR_GetYaw();
	if((!isfinite(yaw)) || (fabsf(yaw) > 720.0f))
	{
		Uart2_SendString("EVT:T90_YAW_BAD\r\n");
		LSM6DSR_ClearYaw();
		yaw = 0.0f;
	}
	yaw_abs = fabsf(yaw);
	gz_dps = LSE6DSR_data.gz_rads * 57.3f;
	if((!isfinite(gz_dps)) || (fabsf(gz_dps) > CIRCLE_GYRO_VALID_MAX_DPS)) gz_dps = 0.0f;

	/* 临时调试：检测期间每200ms上报一次当前yaw和原始角速度 */
	if((g_systick_ms - s_turn90_dbg_last_ms) >= TURN90_DBG_PERIOD_MS)
	{
		char dbg[72];
		int16_t yaw_i = (int16_t)yaw;
		int16_t gz_i = (int16_t)gz_dps;
		snprintf(dbg, sizeof(dbg), "EVT:T90_YAW:%d,GZ:%d\r\n",
				 (int)yaw_i,
				 (int)gz_i);
		Uart2_SendString(dbg);
		s_turn90_dbg_last_ms = g_systick_ms;
	}

	if(yaw_abs > RIGHT_ANGLE_DONE_YAW_DEG)
	{
		if(!s_right_angle_yaw_reached)
		{
			char evt[40];
			snprintf(evt, sizeof(evt), "EVT:TURN90_YAW_OK:%.1f\r\n", yaw_abs);
			Uart2_SendString(evt);
		}
		s_right_angle_yaw_reached = 1;
	}

	/* 顺序约束：必须先满足 yaw>75，再满足中间两路黑 */
	if(s_right_angle_yaw_reached)
	{
		if(center_black)
		{
			if(s_right_angle_center_confirm_cnt < RIGHT_ANGLE_CENTER_CONFIRM_CNT)
			{
				s_right_angle_center_confirm_cnt++;
			}
		}
		else
		{
			s_right_angle_center_confirm_cnt = 0;
		}

		if(s_right_angle_center_confirm_cnt >= RIGHT_ANGLE_CENTER_CONFIRM_CNT)
		{
			Uart2_SendString("EVT:TURN90_DONE\r\n");
			s_right_angle_detect_started = 0;
			s_right_angle_yaw_reached = 0;
			s_right_angle_center_confirm_cnt = 0;
			printf("[TURN90] done yaw=%.1f\r\n", yaw_abs);
			return 1;
		}
	}

	return 0;
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
	s_seg3_crossline_enabled = 0;
	s_seg3_crossline_confirm_cnt = 0;
	s_seg3_start_odo_mm = 0.0f;
	s_seg3_last_crossline_odo_mm = 0.0f;
	s_seg3_turns_done = 0;
	s_seg3_turn_sequence_active = 0;
	s_seg3_turn_decel_done = 0;
	s_seg3_accel_started = 0;
	s_seg3_right_angle_detect_active = 0;
	s_seg3_right_angle_done = 0;
	s_seg2_decel_started = 0;
	s_seg2_decel_done = 0;
	s_seg4_decel_started = 0;
	s_seg4_decel_done = 0;
	s_seg6_decel_started = 0;
	s_seg6_decel_done = 0;
	s_right_angle_detect_started = 0;
	s_right_angle_yaw_reached = 0;
	s_right_angle_center_confirm_cnt = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_crossline_confirm_cnt = 0;
	s_parking_stopped = 0;
	s_parking_stop_ms = 0;
	s_last_print_ms = 0;
	s_exit_straight_active = 0;
	s_exit_trigger_confirm_cnt = 0;
	s_exit_finish_confirm_cnt = 0;
	s_base_speed_ramp_active = 0;
	s_base_speed_ramp_start_base = 0.0f;
	s_base_speed_ramp_target_base = 0.0f;
	s_base_speed_ramp_distance_mm = 0.0f;
	s_base_speed_ramp_start_odo_mm = 0.0f;
	s_base_speed_ramp_speed_range = 0.0f;
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
	s_seg3_crossline_enabled = 0;
	s_seg3_crossline_confirm_cnt = 0;
	s_seg3_start_odo_mm = 0.0f;
	s_seg3_last_crossline_odo_mm = 0.0f;
	s_seg3_turns_done = 0;
	s_seg3_turn_sequence_active = 0;
	s_seg3_turn_decel_done = 0;
	s_seg3_accel_started = 0;
	s_seg3_right_angle_detect_active = 0;
	s_seg3_right_angle_done = 0;
	s_seg2_decel_started = 0;
	s_seg2_decel_done = 0;
	s_seg4_decel_started = 0;
	s_seg4_decel_done = 0;
	s_seg6_decel_started = 0;
	s_seg6_decel_done = 0;
	s_right_angle_detect_started = 0;
	s_right_angle_yaw_reached = 0;
	s_right_angle_center_confirm_cnt = 0;
	s_was_running = 0;
	s_parking_detection_enabled = 0;
	s_crossline_count = 0;
	s_on_crossline = 0;
	s_parking_crossline_confirm_cnt = 0;
	s_parking_stopped = 0;
	s_last_print_ms = 0;
	s_exit_straight_active = 0;
	s_exit_trigger_confirm_cnt = 0;
	s_exit_finish_confirm_cnt = 0;
	s_base_speed_ramp_active = 0;
	s_base_speed_ramp_start_base = 0.0f;
	s_base_speed_ramp_target_base = 0.0f;
	s_base_speed_ramp_distance_mm = 0.0f;
	s_base_speed_ramp_start_odo_mm = 0.0f;
	s_base_speed_ramp_speed_range = 0.0f;
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
		s_seg3_crossline_enabled = 0;
		s_seg3_crossline_confirm_cnt = 0;
		s_seg3_start_odo_mm = Odometer_GetLocation();
		s_seg3_last_crossline_odo_mm = s_seg3_start_odo_mm;
		s_seg3_turns_done = 0;
		s_seg3_turn_sequence_active = 1;
		s_seg3_turn_decel_done = 0;
		s_seg3_accel_started = 0;
		s_seg3_right_angle_detect_active = 0;
		s_seg3_right_angle_done = 0;
		s_right_angle_detect_started = 0;
		s_right_angle_yaw_reached = 0;
		s_right_angle_center_confirm_cnt = 0;
		s_seg4_decel_started = 0;
		s_seg4_decel_done = 0;
		Circle_StartBaseSpeedRampByDistance(SEG3_TURN_DECEL_TARGET_SPEED, SEG3_TURN_DECEL_DISTANCE_MM);
		printf("[SEG3] Turn#1 start (decel to 80 in 1000mm, then detect)\r\n");
		printf("[CIRCLE] << RIGHT DONE, to SEG3\r\n");
	}
	else
	{
		Circle_SendSegmentEndEvent(6);
		s_phase = PHASE_SEG7_RIGHT_ANGLE_PARK;
		s_seg7_start_ms = g_systick_ms;
		Circle_StartBaseSpeedRampByDistance(SEG7_DECEL_TARGET_SPEED, SEG7_DECEL_DISTANCE_MM);
		printf("[SEG7] start decel to 80 in 1000mm\r\n");
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
		s_base_speed_ramp_active = 0;
		s_right_angle_detect_started = 0;
		s_right_angle_yaw_reached = 0;
		s_right_angle_center_confirm_cnt = 0;
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
		s_seg3_crossline_enabled = 0;
		s_seg3_crossline_confirm_cnt = 0;
		s_seg3_start_odo_mm = 0.0f;
		s_seg3_last_crossline_odo_mm = 0.0f;
		s_seg3_turns_done = 0;
		s_seg3_turn_sequence_active = 0;
		s_seg3_turn_decel_done = 0;
		s_seg3_accel_started = 0;
		s_seg3_right_angle_detect_active = 0;
		s_seg3_right_angle_done = 0;
		s_seg2_decel_started = 0;
		s_seg2_decel_done = 0;
		s_seg4_decel_started = 0;
		s_seg4_decel_done = 0;
		s_seg6_decel_started = 0;
		s_seg6_decel_done = 0;
		s_right_angle_detect_started = 0;
		s_right_angle_yaw_reached = 0;
		s_right_angle_center_confirm_cnt = 0;
		s_parking_detection_enabled = 0;
		s_crossline_count = 0;
		s_on_crossline = 0;
		s_parking_crossline_confirm_cnt = 0;
		{
			float cur_base = 0.0f;
			float cur_range = 0.0f;
			PID_GetLineSpeedParams(&cur_base, &cur_range);
			PID_SetLineSpeedParams(SEG1_BASE_SPEED, cur_range);
			Uart2_SendString("EVT:SEG1_BASE:150\r\n");
			printf("[SEG1] base speed set to 150\r\n");
		}
		printf("[CIRCLE] Car started, enter SEG1\r\n");
	}
	
	/* ======== 停车区域检测 (仅第7段并行启用) ======== */
	if(s_phase == PHASE_SEG7_RIGHT_ANGLE_PARK &&
	   s_parking_detection_enabled &&
	   s_state == CIRCLE_STATE_NORMAL)
	{
		uint8_t cross_raw = Circle_DetectCrossLine(adc_values);
		uint8_t cross = 0;

		if(cross_raw)
		{
			if(s_parking_crossline_confirm_cnt < PARKING_CROSSLINE_CONFIRM_CNT)
			{
				s_parking_crossline_confirm_cnt++;
			}
			if(s_parking_crossline_confirm_cnt >= PARKING_CROSSLINE_CONFIRM_CNT)
			{
				cross = 1;
			}
		}
		else
		{
			s_parking_crossline_confirm_cnt = 0;
		}
		
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
					/* 达到目标横线数量, 立即抱死停车 */
					s_parking_stopped = 1;
					s_parking_stop_ms = g_systick_ms;
					s_parking_detection_enabled = 0;
					M3PWM_SetDutyCycle(0);  /* 触发停车时立即关闭负压风扇 */
					WheelLock_Enable();
					printf("[PARKING] %d lines reached, STOP!\r\n", CROSSLINE_TARGET_COUNT);
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
			s_parking_crossline_confirm_cnt = 0;
			printf("[PARKING] Detection enabled\r\n");
		}
	}
	
	/* ======== 陀螺仪死区滤波 ======== */
	gz_filtered = LSE6DSR_data.gz_rads;
	if(gz_filtered > -CIRCLE_GYRO_DEADZONE && gz_filtered < CIRCLE_GYRO_DEADZONE)
	{
		gz_filtered = 0.0f;
	}

	/* ======== 全流程线性调速更新 ======== */
	Circle_UpdateBaseSpeedRampByDistance();
	
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
							s_seg2_decel_started = 0;
							s_seg2_decel_done = 0;
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
						/* 第3段：三横线完成后，降速+直角检测；直角后再加速，最后进入第4段 */
						{
							float seg3_traveled_mm = Odometer_GetLocation() - s_seg3_start_odo_mm;

							/* 第1次直角完成后，满10000mm前屏蔽外侧6路，避免邻道干扰 */
							if(s_seg3_turns_done >= 1 &&
							   !s_seg3_turn_sequence_active &&
							   seg3_traveled_mm < SEG3_CROSSLINE_ENABLE_DISTANCE_MM)
							{
								adc_values[0] = SEG3_MASK_WHITE_ADC_VALUE;
								adc_values[1] = SEG3_MASK_WHITE_ADC_VALUE;
								adc_values[2] = SEG3_MASK_WHITE_ADC_VALUE;
								adc_values[13] = SEG3_MASK_WHITE_ADC_VALUE;
								adc_values[14] = SEG3_MASK_WHITE_ADC_VALUE;
								adc_values[15] = SEG3_MASK_WHITE_ADC_VALUE;
							}

							/* 三横线完成后进入“降速+检测+加速”序列 */
							if(s_seg3_turn_sequence_active)
							{
								if(!s_seg3_turn_decel_done)
								{
									if(!s_base_speed_ramp_active)
									{
										s_seg3_turn_decel_done = 1;
										s_seg3_right_angle_detect_active = 1;
										if(s_seg3_turns_done == 0)
										{
											Uart2_SendString("EVT:SEG3_T1_DECEL_DONE:80\r\n");
											printf("[SEG3] Turn#1 decel done, detect enabled\r\n");
										}
										else
										{
											Uart2_SendString("EVT:SEG3_T2_DECEL_DONE:80\r\n");
											printf("[SEG3] Turn#2 decel done, detect enabled\r\n");
										}
									}
									break;
								}

								if(s_seg3_right_angle_detect_active && !s_seg3_right_angle_done)
								{
									if(Circle_DetectRightAngleDone(adc_values))
									{
										s_seg3_right_angle_done = 1;
										s_seg3_right_angle_detect_active = 0;
										s_seg3_accel_started = 1;
										Circle_StartBaseSpeedRampByDistance(SEG3_TURN_ACCEL_TARGET_SPEED, SEG3_TURN_ACCEL_DISTANCE_MM);
										if(s_seg3_turns_done == 0)
										{
											printf("[SEG3] Turn#1 TURN90 done, start accel to 150 in 1000mm\r\n");
										}
										else
										{
											printf("[SEG3] Turn#2 TURN90 done, start accel to 150 in 1000mm\r\n");
										}
									}
								}

								if(s_seg3_accel_started && !s_base_speed_ramp_active)
								{
									s_seg3_turn_sequence_active = 0;
									s_seg3_turn_decel_done = 0;
									s_seg3_accel_started = 0;
									s_seg3_right_angle_detect_active = 0;
									s_seg3_right_angle_done = 0;
									s_seg3_turns_done++;

									if(s_seg3_turns_done == 1)
									{
										Uart2_SendString("EVT:SEG3_T1_ACCEL_DONE:150\r\n");
										printf("[SEG3] Turn#1 accel done, base=150\r\n");
									}
									else
									{
										Uart2_SendString("EVT:SEG3_T2_ACCEL_DONE:150\r\n");
										printf("[SEG3] Turn#2 accel done, base=150\r\n");
									}

									if(s_seg3_turns_done >= 2)
									{
										Circle_SendSegmentEndEvent(3);
										s_phase = PHASE_SEG4_SHARP_LINE;
										Odometer_Reset();
										s_seg4_decel_started = 0;
										s_seg4_decel_done = 0;
										printf("[SEG3->SEG4] Odometer reset, hold 150 for 3000mm\r\n");
									}
									else
									{
										s_seg3_crossline_count = 0;
										s_seg3_on_crossline = 0;
										s_seg3_crossline_confirm_cnt = 0;
										s_seg3_last_crossline_odo_mm = Odometer_GetLocation();
										printf("[SEG3] Turn#1 done, wait 3 lines for Turn#2\r\n");
									}
								}

								break;
							}

							if(seg3_traveled_mm < SEG3_CROSSLINE_ENABLE_DISTANCE_MM)
							{
								s_seg3_on_crossline = 0;
								s_seg3_crossline_confirm_cnt = 0;
								break;
							}

							if(!s_seg3_crossline_enabled)
							{
								s_seg3_crossline_enabled = 1;
								s_seg3_last_crossline_odo_mm = Odometer_GetLocation();
								Circle_StartBaseSpeedRampByDistance(SEG3_CROSSLINE_ENABLE_ACCEL_TARGET_SPEED,
								                                   SEG3_CROSSLINE_ENABLE_ACCEL_DISTANCE_MM);
								Uart2_SendString("EVT:SEG3_CROSS_EN_ACCEL_START:150\r\n");
								Uart2_SendString("EVT:SEG3_10000\r\n");
								printf("[SEG3] Crossline count enabled at dist=%.1f\r\n", seg3_traveled_mm);
							}

							if(Circle_DetectCrossLine(adc_values))
							{
								if(s_seg3_crossline_confirm_cnt < SEG3_CROSSLINE_CONFIRM_CNT)
								{
									s_seg3_crossline_confirm_cnt++;
								}

								if(s_seg3_crossline_confirm_cnt >= SEG3_CROSSLINE_CONFIRM_CNT)
								{
									if(!s_seg3_on_crossline)
									{
										float odo_now = Odometer_GetLocation();
										if((odo_now - s_seg3_last_crossline_odo_mm) >= SEG3_CROSSLINE_MIN_GAP_MM)
										{
											s_seg3_on_crossline = 1;
											s_seg3_last_crossline_odo_mm = odo_now;
											s_seg3_crossline_count++;
											printf("[SEG3] Crossline %d/%d\r\n",
											       s_seg3_crossline_count,
											       SEG3_CROSSLINE_TARGET_COUNT);
											if(s_seg3_crossline_count >= SEG3_CROSSLINE_TARGET_COUNT)
											{
												if(s_seg3_turns_done < 2 && !s_seg3_turn_sequence_active)
												{
													s_seg3_turn_sequence_active = 1;
													s_seg3_turn_decel_done = 0;
													s_seg3_right_angle_detect_active = 0;
													s_seg3_right_angle_done = 0;
													s_seg3_accel_started = 0;
													s_right_angle_detect_started = 0;
													s_right_angle_yaw_reached = 0;
													s_right_angle_center_confirm_cnt = 0;
													Circle_StartBaseSpeedRampByDistance(SEG3_TURN_DECEL_TARGET_SPEED, SEG3_TURN_DECEL_DISTANCE_MM);
													printf("[SEG3] Turn#2 start (decel to 80 in 1000mm, then detect)\r\n");
												}
											}
										}
									}
								}
							}
							else
							{
								s_seg3_on_crossline = 0;
								s_seg3_crossline_confirm_cnt = 0;
							}
						}
						break;

					case PHASE_SEG4_SHARP_LINE:
						/* 第4段：保持150跑3m，再1m线性减到110，随后进入第5段 */
						if(!s_seg4_decel_started)
						{
							if(Odometer_GetLocation() >= SEG4_HOLD_DISTANCE_MM)
							{
								s_seg4_decel_started = 1;
								s_seg4_decel_done = 0;
								Circle_StartBaseSpeedRampByDistance(SEG4_DECEL_TARGET_SPEED, SEG4_DECEL_DISTANCE_MM);
								printf("[SEG4] hold 3000mm done, start decel to 110 in 1000mm\r\n");
							}
						}
						else if(!s_seg4_decel_done && !s_base_speed_ramp_active)
						{
							s_seg4_decel_done = 1;
							Uart2_SendString("EVT:SEG4_DECEL_DONE:110\r\n");
							Circle_SendSegmentEndEvent(4);
							s_phase = PHASE_SEG5_STRAIGHT_SWITCH;
							printf("[SEG4->SEG5] decel done, keep 110 until LEFT entry\r\n");
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
							Odometer_Reset();
							s_seg6_decel_started = 0;
							s_seg6_decel_done = 0;
							{
								float cur_base = 0.0f;
								float cur_range = 0.0f;
								PID_GetLineSpeedParams(&cur_base, &cur_range);
								PID_SetLineSpeedParams(SEG6_BASE_SPEED, cur_range);
							}
							Uart2_SendString("EVT:SEG6_BASE:110\r\n");
								printf("[SEG5->SEG6] ENTERING (LEFT)\r\n");
							printf("[SEG6] base speed set to 110 (no decel)\r\n");
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
						/* 第7段：停车检测并行逻辑在上方执行 */
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
			uint8_t left_window_trigger = 0;
			uint8_t left_force_trigger = 0;

			/* 正常全幅巡线 + 角度积分 */
			s_accumulated_angle += gz_filtered * CIRCLE_DT * CIRCLE_RAD_TO_DEG;
			Circle_CountBlack(adc_values, &left_black, &right_black, &total_black);
			target_side_black = (s_current_dir == CIRCLE_DIR_RIGHT) ? left_black : right_black;
			in_circle_distance_mm = Odometer_GetLocation();
			if((!isfinite(in_circle_distance_mm)) || in_circle_distance_mm < 0.0f || in_circle_distance_mm > CIRCLE_ODO_VALID_MAX_MM)
			{
				Uart2_SendString("EVT:ODO_BAD\r\n");
				Odometer_Reset();
				in_circle_distance_mm = 0.0f;
			}

			if(s_phase == PHASE_SEG6_LEFT_CIRCLE && s_current_dir == CIRCLE_DIR_LEFT)
			{
				if(!s_seg6_decel_started && in_circle_distance_mm >= SEG6_DECEL_TRIGGER_DISTANCE_MM)
				{
					s_seg6_decel_started = 1;
					s_seg6_decel_done = 0;
					Circle_StartBaseSpeedRampByDistance(SEG6_DECEL_TARGET_SPEED, SEG6_DECEL_DISTANCE_MM);
					printf("[SEG6] dist>=2000, start decel to 80 in 1000mm\r\n");
				}

				if(s_seg6_decel_started && !s_seg6_decel_done && !s_base_speed_ramp_active)
				{
					s_seg6_decel_done = 1;
					Uart2_SendString("EVT:SEG6_DECEL_DONE:80\r\n");
					printf("[SEG6] decel done, base=80\r\n");
				}
			}

			if(s_phase == PHASE_SEG2_RIGHT_CIRCLE && s_current_dir == CIRCLE_DIR_RIGHT)
			{
				if(!s_seg2_decel_started && in_circle_distance_mm >= SEG2_DECEL_TRIGGER_DISTANCE_MM)
				{
					s_seg2_decel_started = 1;
					s_seg2_decel_done = 0;
					Circle_StartBaseSpeedRampByDistance(SEG2_DECEL_TARGET_SPEED, SEG2_DECEL_DISTANCE_MM);
					printf("[SEG2] dist>=3000, start decel to 110 in 1000mm\r\n");
				}

				if(s_seg2_decel_started && !s_seg2_decel_done && !s_base_speed_ramp_active)
				{
					s_seg2_decel_done = 1;
					Uart2_SendString("EVT:SEG2_DECEL_DONE:110\r\n");
					printf("[SEG2] decel done, base=110\r\n");
				}
			}

			right_window_trigger = (s_current_dir == CIRCLE_DIR_RIGHT &&
			                        s_seg2_decel_done &&
			                        in_circle_distance_mm > EXIT_TRIGGER_RIGHT_DISTANCE_MM &&
			                        in_circle_distance_mm < EXIT_TRIGGER_RIGHT_DISTANCE_MAX_MM &&
			                        left_black >= EXIT_TRIGGER_RIGHT_LEFTBLACK_MIN &&
			                        left_black <= EXIT_TRIGGER_RIGHT_LEFTBLACK_MAX) ? 1 : 0;

			right_force_trigger = (s_current_dir == CIRCLE_DIR_RIGHT &&
			                       s_seg2_decel_done &&
			                       in_circle_distance_mm >= EXIT_TRIGGER_RIGHT_DISTANCE_MAX_MM) ? 1 : 0;

			left_window_trigger = (s_current_dir == CIRCLE_DIR_LEFT &&
			                       in_circle_distance_mm > EXIT_TRIGGER_LEFT_DISTANCE_MM &&
			                       in_circle_distance_mm < EXIT_TRIGGER_LEFT_DISTANCE_MAX_MM &&
			                       target_side_black >= EXIT_TRIGGER_EDGE_BLACK_MIN) ? 1 : 0;

			left_force_trigger = (s_current_dir == CIRCLE_DIR_LEFT &&
			                      in_circle_distance_mm >= EXIT_TRIGGER_LEFT_DISTANCE_MAX_MM) ? 1 : 0;

			/* 右环：窗口内按光电判定；超上限强制触发。左环：3000~4000窗口+上限强制触发 */
			if(right_window_trigger ||
			   right_force_trigger ||
			   left_window_trigger ||
			   left_force_trigger)
			{
				if(right_force_trigger)
				{
					s_exit_trigger_confirm_cnt = EXIT_TRIGGER_CONFIRM_CNT;
				}
				else if(left_force_trigger)
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
						printf("[CIRCLE] LEFT EXIT TRIG odo=%.1fmm\r\n", in_circle_distance_mm);
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
				float finish_distance_min_mm;
				uint32_t exit_timeout_ms;
				uint8_t exit_dir_ok = 0;

				/* 直行阶段：忽略光电干涉，强制中心位置 */
				position_get = (s_current_dir == CIRCLE_DIR_RIGHT) ? EXIT_FORCE_POS_RIGHT_X10 : EXIT_FORCE_POS_LEFT_X10;
				g_lose_time = 0;
				total_black_all = Circle_CountBlackAll16(adc_values);
				exit_distance_mm = Odometer_GetLocation();
				finish_distance_min_mm = (s_current_dir == CIRCLE_DIR_RIGHT) ?
					EXIT_STRAIGHT_FINISH_DISTANCE_RIGHT_MM : EXIT_STRAIGHT_FINISH_DISTANCE_LEFT_MM;
				exit_timeout_ms = (s_current_dir == CIRCLE_DIR_RIGHT) ?
					EXIT_STRAIGHT_TIMEOUT_RIGHT_MS : EXIT_STRAIGHT_TIMEOUT_LEFT_MS;
				if((!isfinite(exit_distance_mm)) || exit_distance_mm < 0.0f || exit_distance_mm > CIRCLE_ODO_VALID_MAX_MM)
				{
					Uart2_SendString("EVT:ODO_BAD\r\n");
					Odometer_Reset();
					exit_distance_mm = 0.0f;
				}

				exit_dir_ok = (s_current_dir == CIRCLE_DIR_RIGHT) ? 1 :
					((left_black >= right_black) ? 1 : 0);

				/* 退出双判定：里程达到门槛且16路总黑在2~3，需连续确认 */
				if(exit_distance_mm > finish_distance_min_mm &&
				   total_black_all >= EXIT_STRAIGHT_FINISH_BLACK_MIN &&
				   total_black_all <= EXIT_STRAIGHT_FINISH_BLACK_MAX &&
				   exit_dir_ok)
				{
					if(s_exit_finish_confirm_cnt < EXIT_STRAIGHT_FINISH_CONFIRM_CNT)
					{
						s_exit_finish_confirm_cnt++;
					}

					if(s_exit_finish_confirm_cnt >= EXIT_STRAIGHT_FINISH_CONFIRM_CNT)
					{
						printf("[CIRCLE] EXIT STRAIGHT OFF dist=%.1f total16=%d\r\n",
						       exit_distance_mm,
						       total_black_all);
						Circle_LogBlackFlags("EXIT_DONE", adc_values);
						Circle_CompleteExit();
					}
				}
				else
				{
					s_exit_finish_confirm_cnt = 0;
				}

				/* 超时兜底 */
				if(s_state == CIRCLE_STATE_EXITING &&
				   (g_systick_ms - s_exit_straight_start_ms) >= exit_timeout_ms)
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
