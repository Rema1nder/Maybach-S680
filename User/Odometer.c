/**
 * @file    Odometer.c
 * @brief   里程计模块实现
 * @details 基于编码器和陀螺仪实现位置跟踪
 */

#include "Odometer.h"
#include "ABEncoder.h"
#include "LSM6DSR_Config.h"
#include <math.h>
#include <stddef.h>

/* ========================================================================== */
/*                              私有宏定义                                     */
/* ========================================================================== */

#define PI              3.14159265358979f
#define DEG_TO_RAD      (PI / 180.0f)
#define RAD_TO_DEG      (180.0f / PI)
#define DT              0.002f              /* SysTick 周期: 2ms */

/* ========================================================================== */
/*                              私有变量                                       */
/* ========================================================================== */

/* 里程计状态 */
static Odometer_Data_t s_odom_data = {0};

/* 上一次的编码器值 */
static int32_t s_last_left_cnt = 0;
static int32_t s_last_right_cnt = 0;

/* 陀螺仪融合参数 */
static uint8_t s_gyro_fusion_enabled = 1;   /* 默认启用陀螺仪融合 */
static float s_gyro_fusion_weight = 0.9f;   /* 陀螺仪权重 */

/* 校准状态 */
static Odometer_Calibration_t s_calibration = {0};

/* 当前使用的编码器参数 (可在运行时更新) */
static float s_pulse_per_mm = ODOM_ENCODER_PULSE_PER_MM;

/* ========================================================================== */
/*                              私有函数                                       */
/* ========================================================================== */

/**
 * @brief  角度归一化到 -180 ~ 180 度
 */
static float NormalizeAngle(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

/* ========================================================================== */
/*                              公有函数实现                                   */
/* ========================================================================== */

void Odometer_Init(void)
{
    /* 清零所有数据 */
    s_odom_data.location = 0.0f;
    s_odom_data.x = 0.0f;
    s_odom_data.y = 0.0f;
    s_odom_data.theta = 0.0f;
    
    /* 记录当前编码器值 */
    s_last_left_cnt = left_ecoder_cnt;
    s_last_right_cnt = right_ecoder_cnt;
    
    /* 默认启用陀螺仪融合 */
    s_gyro_fusion_enabled = 1;
    s_gyro_fusion_weight = 0.9f;
    
    /* 使用默认校准参数 */
    s_pulse_per_mm = ODOM_ENCODER_PULSE_PER_MM;
    
    /* 清除校准状态 */
    s_calibration.active = 0;
}

void Odometer_Update(void)
{
    /* 获取编码器增量 */
    int32_t left_delta = left_ecoder_cnt - s_last_left_cnt;
    int32_t right_delta = right_ecoder_cnt - s_last_right_cnt;
    
    s_last_left_cnt = left_ecoder_cnt;
    s_last_right_cnt = right_ecoder_cnt;
    
    /* 计算左右轮位移 (mm) */
    float left_dist = (float)left_delta / s_pulse_per_mm;
    float right_dist = (float)right_delta / s_pulse_per_mm;
    
    /* 计算中心位移 (两轮平均) */
    float center_dist = (left_dist + right_dist) / 2.0f;
    
    /* 累计行驶距离 (取绝对值累加) */
    s_odom_data.location += fabsf(center_dist);
    
    /* -------------------------------------------------------------------- */
    /*                         航向角更新                                    */
    /* -------------------------------------------------------------------- */
    
    /* 方法1: 使用编码器差速计算航向变化 */
    float delta_theta_encoder = (right_dist - left_dist) / ODOM_WHEEL_BASE_MM;
    delta_theta_encoder *= RAD_TO_DEG;  /* 转换为度 */
    
    /* 方法2: 使用陀螺仪积分 */
    float delta_theta_gyro = 0.0f;
    if(s_gyro_fusion_enabled)
    {
        /* 陀螺仪 Z 轴角速度积分 (LSE6DSR_data.gz_rads 单位: rad/s) */
        delta_theta_gyro = LSE6DSR_data.gz_rads * DT * RAD_TO_DEG;
    }
    
    /* 融合两种航向估计 */
    float delta_theta;
    if(s_gyro_fusion_enabled)
    {
        /* 互补滤波融合: 陀螺仪权重 + 编码器权重 */
        delta_theta = s_gyro_fusion_weight * delta_theta_gyro + 
                      (1.0f - s_gyro_fusion_weight) * delta_theta_encoder;
    }
    else
    {
        /* 仅使用编码器 */
        delta_theta = delta_theta_encoder;
    }
    
    /* 更新航向角 */
    s_odom_data.theta += delta_theta;
    s_odom_data.theta = NormalizeAngle(s_odom_data.theta);
    
    /* -------------------------------------------------------------------- */
    /*                         位置坐标更新                                  */
    /* -------------------------------------------------------------------- */
    
    /* 使用中点积分法更新位置 */
    /* 使用更新前后航向角的中点作为这段位移的方向 */
    float mid_theta = (s_odom_data.theta - delta_theta / 2.0f) * DEG_TO_RAD;
    
    s_odom_data.x += center_dist * cosf(mid_theta);
    s_odom_data.y += center_dist * sinf(mid_theta);
}

void Odometer_Reset(void)
{
    /* 完全重置 */
    s_odom_data.location = 0.0f;
    s_odom_data.x = 0.0f;
    s_odom_data.y = 0.0f;
    s_odom_data.theta = 0.0f;
    
    /* 更新编码器参考值 */
    s_last_left_cnt = left_ecoder_cnt;
    s_last_right_cnt = right_ecoder_cnt;
}

void Odometer_ResetCoordinate(void)
{
    /* 仅重置坐标，保留里程 */
    s_odom_data.x = 0.0f;
    s_odom_data.y = 0.0f;
    s_odom_data.theta = 0.0f;
    
    /* 更新编码器参考值 */
    s_last_left_cnt = left_ecoder_cnt;
    s_last_right_cnt = right_ecoder_cnt;
}

float Odometer_GetLocation(void)
{
    return s_odom_data.location;
}

float Odometer_GetX(void)
{
    return s_odom_data.x;
}

float Odometer_GetY(void)
{
    return s_odom_data.y;
}

float Odometer_GetTheta(void)
{
    return s_odom_data.theta;
}

void Odometer_GetData(Odometer_Data_t *data)
{
    if(data != NULL)
    {
        data->location = s_odom_data.location;
        data->x = s_odom_data.x;
        data->y = s_odom_data.y;
        data->theta = s_odom_data.theta;
    }
}

/* ========================================================================== */
/*                              校准函数实现                                   */
/* ========================================================================== */

void Odometer_StartCalibration(void)
{
    s_calibration.active = 1;
    s_calibration.start_left_cnt = left_ecoder_cnt;
    s_calibration.start_right_cnt = right_ecoder_cnt;
}

float Odometer_EndCalibration(float actual_distance_mm)
{
    if(!s_calibration.active || actual_distance_mm <= 0.0f)
    {
        s_calibration.active = 0;
        return 0.0f;
    }
    
    /* 计算校准期间的编码器脉冲数 */
    int32_t left_pulses = left_ecoder_cnt - s_calibration.start_left_cnt;
    int32_t right_pulses = right_ecoder_cnt - s_calibration.start_right_cnt;
    
    /* 取绝对值 (可能是倒车) */
    if(left_pulses < 0) left_pulses = -left_pulses;
    if(right_pulses < 0) right_pulses = -right_pulses;
    
    /* 计算平均脉冲数 */
    float avg_pulses = (float)(left_pulses + right_pulses) / 2.0f;
    
    /* 计算每毫米的脉冲数 */
    float new_pulse_per_mm = avg_pulses / actual_distance_mm;
    
    /* 更新运行时参数 */
    s_pulse_per_mm = new_pulse_per_mm;
    
    /* 结束校准 */
    s_calibration.active = 0;
    
    return new_pulse_per_mm;
}

uint8_t Odometer_IsCalibrating(void)
{
    return s_calibration.active;
}

int32_t Odometer_GetCalibrationPulseCount(void)
{
    if(!s_calibration.active) return 0;
    
    int32_t left_pulses = left_ecoder_cnt - s_calibration.start_left_cnt;
    int32_t right_pulses = right_ecoder_cnt - s_calibration.start_right_cnt;
    
    /* 取绝对值 */
    if(left_pulses < 0) left_pulses = -left_pulses;
    if(right_pulses < 0) right_pulses = -right_pulses;
    
    return (left_pulses + right_pulses) / 2;
}

/* ========================================================================== */
/*                              陀螺仪融合设置                                 */
/* ========================================================================== */

void Odometer_EnableGyroFusion(uint8_t enable)
{
    s_gyro_fusion_enabled = enable;
}

void Odometer_SetGyroFusionWeight(float weight)
{
    if(weight < 0.0f) weight = 0.0f;
    if(weight > 1.0f) weight = 1.0f;
    s_gyro_fusion_weight = weight;
}
