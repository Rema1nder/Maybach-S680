#include "Motor_ctr.h"

// 全局变量
static uint32_t g_motor_frequency = MOTOR_FREQUENCY_17KHZ;  // 当前PWM频率
static uint16_t g_motor1_duty = 0;                          // 电机1当前占空比
static uint16_t g_motor2_duty = 0;                          // 电机2当前占空比
static uint8_t g_motor1_direction = MOTOR_DIR_FORWARD;      // 电机1当前方向
static uint8_t g_motor2_direction = MOTOR_DIR_FORWARD;      // 电机2当前方向
uint16_t uart_rev_tiem = 0;
/**
 * @brief  电机初始化函数
 * @note   配置PA12为电机使能，PA8/PA10为方向控制，PA9/PA11为PWM输出
 * @param  无
 * @retval 无
 */
void Motor_Init(void)
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  // 开启GPIOA和AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);                         // 开启TIM1时钟
    
    // GPIO配置 - 电机使能引脚PA12
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           // PA12 - 电机使能
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // 50MHz速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // GPIO配置 - 电机方向控制引脚PA8和PA10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10;  // PA8 - 电机1方向, PA10 - 电机2方向
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // 50MHz速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // GPIO配置 - PWM输出引脚PA9和PA11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;  // PA9 - 电机1 PWM, PA11 - 电机2 PWM
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // 50MHz速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // TIM1时基配置 - 17KHz频率计算
    // 频率 = 72MHz / ((PSC + 1) × (ARR + 1))
    // 17KHz = 72000000 / ((0 + 1) × (4234 + 1)) = 72000000 / 4235 ≈ 17000Hz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 4234;             // ARR值，决定PWM周期 (4234 + 1 = 4235)
    TIM_TimeBaseStructure.TIM_Prescaler = 0;             // PSC值，决定时钟分频 (0 + 1 = 1)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频因子
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;     // 重复计数器值
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    // TIM1通道2 PWM配置 (电机1 - PA9)
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  // 输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;  // 互补输出禁用
    TIM_OCInitStructure.TIM_Pulse = 0;                   // CCR值，初始占空比为0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  // 输出极性为高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  // 互补输出极性
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;  // 空闲状态输出
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;  // 空闲状态互补输出
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);             // 初始化通道2
    
    // TIM1通道4 PWM配置 (电机2 - PA11)
    TIM_OCInitStructure.TIM_Pulse = 0;                   // CCR值，初始占空比为0
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);             // 初始化通道4
    
    // 使能TIM1的PWM输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    // 启动TIM1
    TIM_Cmd(TIM1, ENABLE);
    Motor_SetDirection(MOTOR_R,0);
		Motor_SetDirection(MOTOR_L,0);
    // 初始化电机状态
    Motor_Disable();  // 初始状态禁用电机
    Motor_StopAll();  // 停止所有电机
}

/**
 * @brief  使能电机
 * @note   设置PA12为高电平
 * @param  无
 * @retval 无
 */
void Motor_Enable(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_12);
}

/**
 * @brief  禁用电机
 * @note   设置PA12为低电平
 * @param  无
 * @retval 无
 */
void Motor_Disable(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}

/**
 * @brief  设置电机速度
 * @note   占空比范围0-10000
 * @param  motor_id: 电机编号 (MOTOR_R 或 MOTOR_L)
 * @param  duty: 占空比值 (0-10000)
 * @retval 无
 */
void Motor_SetSpeed(uint8_t motor_id, uint16_t duty)
{
    uint16_t compare_value;
    
    // 限制占空比范围
    if(duty > MOTOR_DUTY_MAX) duty = MOTOR_DUTY_MAX;
    
    // 计算比较值 (将0-10000映射到0-4234)
    compare_value = (uint16_t)((TIM1->ARR + 1) * duty / MOTOR_DUTY_MAX);
    
    switch(motor_id)
    {
        case MOTOR_R:
            g_motor1_duty = duty;
            TIM_SetCompare2(TIM1, compare_value);  // 通道2 - 电机1
            break;
            
        case MOTOR_L:
            g_motor2_duty = duty;
            TIM_SetCompare4(TIM1, compare_value);  // 通道4 - 电机2
            break;
            
        default:
            break;
    }
}

/**
 * @brief  设置电机方向
 * @note   设置对应的方向引脚
 * @param  motor_id: 电机编号 (MOTOR_R 或 MOTOR_L)
 * @param  direction: 方向 (MOTOR_DIR_FORWARD 或 MOTOR_DIR_BACKWARD)
 * @retval 无
 */
void Motor_SetDirection(uint8_t motor_id, uint8_t direction)
{
    switch(motor_id)
    {
        case MOTOR_R:
            g_motor1_direction = direction;
            if(direction == MOTOR_DIR_FORWARD)
            {
                GPIO_SetBits(GPIOA, GPIO_Pin_8);     // PA8高电平 - 电机1正转
            }
            else
            {
                GPIO_ResetBits(GPIOA, GPIO_Pin_8);   // PA8低电平 - 电机1反转
            }
            break;
            
        case MOTOR_L:
            g_motor2_direction = direction;
            if(direction != MOTOR_DIR_FORWARD)
            {
                GPIO_SetBits(GPIOA, GPIO_Pin_10);    // PA10高电平 - 电机2正转
            }
            else
            {
                GPIO_ResetBits(GPIOA, GPIO_Pin_10);  // PA10低电平 - 电机2反转
            }
            break;
            
        default:
            break;
    }
}

uint16_t Motor_GetDuty(uint8_t motor_id)
{
    if(motor_id == MOTOR_R)
    {
        return g_motor1_duty;
    }
    else if(motor_id == MOTOR_L)
    {
        return g_motor2_duty;
    }
    return 0;
}

uint8_t Motor_GetDirection(uint8_t motor_id)
{
    if(motor_id == MOTOR_R)
    {
        return g_motor1_direction;
    }
    else if(motor_id == MOTOR_L)
    {
        return g_motor2_direction;
    }
    return MOTOR_DIR_FORWARD;
}

/**
 * @brief  停止指定电机
 * @note   设置PWM占空比为0
 * @param  motor_id: 电机编号 (MOTOR_R 或 MOTOR_L)
 * @retval 无
 */
void Motor_Stop(uint8_t motor_id)
{
    Motor_SetSpeed(motor_id, 0);
}

/**
 * @brief  停止所有电机
 * @note   设置所有电机PWM占空比为0
 * @param  无
 * @retval 无
 */
void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_R);
    Motor_Stop(MOTOR_L);
}

/**
 * @brief  设置PWM频率
 * @note   频率范围1Hz-1MHz
 * @param  freq: 频率值 (Hz)
 * @retval 无
 */
void Motor_SetFrequency(uint32_t freq)
{
    uint32_t psc, arr;
    uint32_t timer_clock = 72000000;  // TIM1时钟频率72MHz (APB2)
    
    // 限制频率范围
    if(freq < 1) freq = 1;
    if(freq > 1000000) freq = 1000000;
    
    g_motor_frequency = freq;
    
    // 计算预分频值和自动重装值
    // 频率 = 定时器时钟 / ((PSC + 1) × (ARR + 1))
    // 为了获得最佳精度，先计算PSC，再计算ARR
    psc = timer_clock / (freq * 1000) - 1;  // 目标ARR约为1000
    if(psc > 65535) psc = 65535;
    
    arr = timer_clock / (freq * (psc + 1)) - 1;
    if(arr > 65535) arr = 65535;
    
    // 更新定时器配置
    TIM_PrescalerConfig(TIM1, psc, TIM_PSCReloadMode_Immediate);
    TIM_SetAutoreload(TIM1, arr);
    
    // 重新设置电机速度以保持占空比
    Motor_SetSpeed(MOTOR_R, g_motor1_duty);
    Motor_SetSpeed(MOTOR_L, g_motor2_duty);
} 