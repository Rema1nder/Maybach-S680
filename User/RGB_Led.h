#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include "stm32f10x.h"

// 引脚映射：PB3(G)，PA15(B)，PB8(R)

typedef enum {
	RGB_COLOR_OFF = 0,
	RGB_COLOR_R,
	RGB_COLOR_G,
	RGB_COLOR_B,
	RGB_COLOR_YELLOW,   // R + G
	RGB_COLOR_CYAN,     // G + B
	RGB_COLOR_MAGENTA,   // R + B
	RGB_COLOR_On
} RGB_Color_t;

void RGB_Init(void);
void RGB_SetColor(RGB_Color_t color);

#endif

