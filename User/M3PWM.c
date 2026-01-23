#include "M3PWM.h"

// 全局变量
static uint32_t g_pwm_frequency = PWM_FREQUENCY_17KHZ;  // 当前PWM频率
static uint8_t g_pwm_duty_cycle = PWM_DUTY_CYCLE_50;    // 当前PWM占空比

/**
 * @brief  PWM初始化函数
 * @note   配置PB11为TIM2_CH4的PWM输出，频率17KHz
 * @param  无
 * @retval 无
 */
void M3PWM_Init(void)
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);  // 开启GPIOB和AFIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);                         // 开启TIM2时钟
    
    // TIM2重映射配置 - PB11作为TIM2_CH4需要部分重映射2或完全重映射
    GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);  // 部分重映射2: CH1/PA0, CH2/PA1, CH3/PB10, CH4/PB11
    
    // GPIO配置 - PB11配置为复用推挽输出
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           // PB11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // 50MHz速度
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // TIM2时基配置 - 17KHz频率计算
    // 频率 = 72MHz / ((PSC + 1) × (ARR + 1))
    // 17KHz = 72000000 / ((0 + 1) × (4234 + 1)) = 72000000 / 4235 ≈ 17000Hz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 4234;             // ARR值，决定PWM周期 (4234 + 1 = 4235)
    TIM_TimeBaseStructure.TIM_Prescaler = 0;             // PSC值，决定时钟分频 (0 + 1 = 1)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频因子
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;     // 重复计数器值
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // TIM2通道4 PWM配置
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;  // 互补输出禁用
    TIM_OCInitStructure.TIM_Pulse = 2117;                // CCR值，决定PWM占空比 (50% = 4234/2)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  // 输出极性为高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  // 互补输出极性
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;  // 空闲状态输出
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;  // 空闲状态互补输出
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);             // 初始化通道4
    
    // 使能TIM2的PWM输出
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    
    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
		M3PWM_SetDutyCycle(0);
}

/**
 * @brief  设置PWM占空比
 * @note   占空比范围0-100
 * @param  duty: 占空比值 (0-100)
 * @retval 无
 */
void M3PWM_SetDutyCycle(uint16_t duty)
{
    uint16_t compare_value;
    
    // 限制占空比范围
    if(duty > 1000) duty = 1000;
    
    g_pwm_duty_cycle = duty;
    
    // 计算比较值
    compare_value = (uint32_t)((TIM2->ARR + 1) * duty / 1000);
    
    // 设置比较值
    TIM_SetCompare4(TIM2, compare_value);
}

/**
 * @brief  启动PWM输出
 * @note   无
 * @param  无
 * @retval 无
 */
void M3PWM_Start(void)
{
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

/**
 * @brief  停止PWM输出
 * @note   无
 * @param  无
 * @retval 无
 */
void M3PWM_Stop(void)
{
    TIM_CtrlPWMOutputs(TIM2, DISABLE);
    TIM_Cmd(TIM2, DISABLE);
}

/**
 * @brief  设置PWM频率
 * @note   频率范围1Hz-1MHz
 * @param  freq: 频率值 (Hz)
 * @retval 无
 */
void M3PWM_SetFrequency(uint32_t freq)
{
    uint32_t psc, arr;
    uint32_t timer_clock = 72000000;  // TIM2时钟频率72MHz (APB1)
    
    // 限制频率范围
    if(freq < 1) freq = 1;
    if(freq > 1000000) freq = 1000000;
    
    g_pwm_frequency = freq;
    
    // 计算预分频值和自动重装值
    // 频率 = 定时器时钟 / ((PSC + 1) × (ARR + 1))
    // 为了获得最佳精度，先计算PSC，再计算ARR
    psc = timer_clock / (freq * 1000) - 1;  // 目标ARR约为1000
    if(psc > 65535) psc = 65535;
    
    arr = timer_clock / (freq * (psc + 1)) - 1;
    if(arr > 65535) arr = 65535;
    
    // 更新定时器配置
    TIM_PrescalerConfig(TIM2, psc, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(TIM2, arr);
    
    // 重新设置占空比
    M3PWM_SetDutyCycle(g_pwm_duty_cycle);
} 