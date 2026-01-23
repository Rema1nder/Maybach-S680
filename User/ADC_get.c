#include "ADC_get.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "Delay.h"

// 模拟复用器选择引脚：S0..S3 -> PB10, PB2, PB1, PB0
#define MUX_GPIO_PORT		GPIOB
#define MUX_GPIO_CLK		RCC_APB2Periph_GPIOB
#define MUX_S0_PIN			GPIO_Pin_10
#define MUX_S1_PIN			GPIO_Pin_2
#define MUX_S2_PIN			GPIO_Pin_1
#define MUX_S3_PIN			GPIO_Pin_0

// ADC输入：PA1 -> ADC12_IN1 (ADC1通道1)
#define ADC_GPIO_PORT		GPIOA
#define ADC_GPIO_CLK		RCC_APB2Periph_GPIOA

volatile uint16_t g_mux_adc_values[17] = {0};

static void MuxADC_SetSelectLines(uint8_t channel)
{
	// channel: 0~15，对应S3..S0为bit3..bit0
	// 写入顺序：S0(PB10) <- bit0, S1(PB2) <- bit1, S2(PB1) <- bit2, S3(PB0) <- bit3
	BitAction b0 = (channel & 0x01) ? Bit_SET : Bit_RESET; // S0
	BitAction b1 = (channel & 0x02) ? Bit_SET : Bit_RESET; // S1
	BitAction b2 = (channel & 0x04) ? Bit_SET : Bit_RESET; // S2
	BitAction b3 = (channel & 0x08) ? Bit_SET : Bit_RESET; // S3
	GPIO_WriteBit(MUX_GPIO_PORT, MUX_S0_PIN, b0);
	GPIO_WriteBit(MUX_GPIO_PORT, MUX_S1_PIN, b1);
	GPIO_WriteBit(MUX_GPIO_PORT, MUX_S2_PIN, b2);
	GPIO_WriteBit(MUX_GPIO_PORT, MUX_S3_PIN, b3);
}

void MuxADC_Init(void)
{
	GPIO_InitTypeDef gpio;
	ADC_InitTypeDef adc;

	// 开启GPIO与ADC时钟
	RCC_APB2PeriphClockCmd(MUX_GPIO_CLK | ADC_GPIO_CLK | RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 72MHz/6 = 12MHz，ADC时钟不超过14MHz

	// 复用器选择脚配置为推挽输出
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = MUX_S0_PIN | MUX_S1_PIN | MUX_S2_PIN | MUX_S3_PIN;
	GPIO_Init(MUX_GPIO_PORT, &gpio);
	// 默认拉低
	GPIO_ResetBits(MUX_GPIO_PORT, MUX_S0_PIN | MUX_S1_PIN | MUX_S2_PIN | MUX_S3_PIN);

	// ADC通道引脚PA1/PA0为模拟输入
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0; // PA1, PA0
	GPIO_Init(ADC_GPIO_PORT, &gpio);

	// ADC1配置
	ADC_DeInit(ADC1);
	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = DISABLE;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &adc);

	// 使能ADC并校准
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}

static uint16_t MuxADC_ReadOnce(void)
{
	volatile uint32_t timeout = 10000;
	// 选择规则通道1 (ADC_Channel_1 -> PA1)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
	{
		if(--timeout == 0) return 0; // 超时保护
	}
	return ADC_GetConversionValue(ADC1);
}

static uint16_t ADC_Read_PA0(void)
{
	volatile uint32_t timeout = 10000;
	// 读取ADC_Channel_0 -> PA0
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
	{
		if(--timeout == 0) return 0; // 超时保护
	}
	return ADC_GetConversionValue(ADC1);
}

uint16_t MuxADC_ReadChannel(uint8_t channel)
{
	if(channel > 15) channel = 15;
	MuxADC_SetSelectLines(channel);
	// 允许通道切换后的模拟保持-建立时间，依据MUX与源阻抗，微调
	Delay_us(3);
	// 扔掉一次以消除残留
	(void)MuxADC_ReadOnce();
	Delay_us(2);
	return MuxADC_ReadOnce();
}

void MuxADC_SampleAll(void)
{
	uint8_t i;
	for(i = 0; i < 16; i++)
	{
		g_mux_adc_values[i] = MuxADC_ReadChannel(i);
	}
	// 额外采集PA0(ADC12_IN0)
	g_mux_adc_values[16] = ADC_Read_PA0();
}


