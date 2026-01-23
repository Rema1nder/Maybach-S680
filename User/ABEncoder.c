#include "stm32f10x.h"
#include "ABEncoder.h"

static int16_t last_count_left = 0,last_count_right = 0;
int16_t speed_left = 0,speed_right = 0;
int32_t left_ecoder_cnt = 0, right_ecoder_cnt = 0;
void ABEncoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //PA6 PA7 编码器1，PB6 PB7 编码器2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    // PA6/PA7 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	    // PB6/PB7 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    // 定时器基本参数
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		    // 定时器基本参数
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // 编码器接口模式
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
		    // 编码器接口模式
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

void ABEncoder_UpdateSpeed(void) // 计算速度
{
    // 定义静态浮点变量保存滤波后的值，防止精度丢失
    static float speed_left_filter = 0.0f;
    static float speed_right_filter = 0.0f;

    int16_t now_left = -TIM_GetCounter(TIM4);
    int16_t raw_speed_left = now_left - last_count_left;
    last_count_left = now_left;
    
    // 一阶低通滤波：新值占30%，旧值占70%
    speed_left_filter = speed_left_filter * 0.7f + (float)raw_speed_left * 0.3f;
    speed_left = (int16_t)speed_left_filter;

    int16_t now_right =  TIM_GetCounter(TIM3);
    int16_t raw_speed_right = now_right - last_count_right;
    last_count_right = now_right;

    // 一阶低通滤波
    speed_right_filter = speed_right_filter * 0.7f + (float)raw_speed_right * 0.3f;
    speed_right = (int16_t)speed_right_filter;

    left_ecoder_cnt += speed_left;
    right_ecoder_cnt += speed_right;
}