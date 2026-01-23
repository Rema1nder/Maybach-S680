#include "RGB_Led.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

// 共阳极LED：低电平点亮，高电平熄灭

static inline void RGB_WriteR(BitAction v) { GPIO_WriteBit(GPIOB, GPIO_Pin_8, v); }
static inline void RGB_WriteG(BitAction v) { GPIO_WriteBit(GPIOB, GPIO_Pin_3, v); }
static inline void RGB_WriteB(BitAction v) { GPIO_WriteBit(GPIOA, GPIO_Pin_15, v); }

static void RGB_AllOff(void)
{
	RGB_WriteR(Bit_SET);  // 高电平熄灭
	RGB_WriteG(Bit_SET);
	RGB_WriteB(Bit_SET);
}

void RGB_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	// 释放PB3、PB4、PA15复用功能：关闭JTAG但保留SWD，J-Link可用
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// PB8(R)，PB3(G) 推挽输出
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_3;
	GPIO_Init(GPIOB, &gpio);

	// PA15(B) 推挽输出
	gpio.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &gpio);

	RGB_AllOff();
}

void RGB_SetColor(RGB_Color_t color)
{
	switch(color)
	{
		case RGB_COLOR_OFF:
			RGB_AllOff();
			break;
		case RGB_COLOR_R:
			RGB_WriteR(Bit_RESET); RGB_WriteG(Bit_SET); RGB_WriteB(Bit_SET);  // 低电平点亮
			break;
		case RGB_COLOR_G:
			RGB_WriteR(Bit_SET); RGB_WriteG(Bit_RESET); RGB_WriteB(Bit_SET);
			break;
		case RGB_COLOR_B:
			RGB_WriteR(Bit_SET); RGB_WriteG(Bit_SET); RGB_WriteB(Bit_RESET);
			break;
		case RGB_COLOR_YELLOW: // R+G
			RGB_WriteR(Bit_RESET); RGB_WriteG(Bit_RESET); RGB_WriteB(Bit_SET);
			break;
		case RGB_COLOR_CYAN: // G+B
			RGB_WriteR(Bit_SET); RGB_WriteG(Bit_RESET); RGB_WriteB(Bit_RESET);
			break;
		case RGB_COLOR_MAGENTA: // R+B
			RGB_WriteR(Bit_RESET); RGB_WriteG(Bit_SET); RGB_WriteB(Bit_RESET);
			break;
		case RGB_COLOR_On: // R+B
			RGB_WriteR(Bit_RESET); RGB_WriteG(Bit_RESET); RGB_WriteB(Bit_RESET);
			break;
		default:
			RGB_AllOff();
			break;
	}
}


