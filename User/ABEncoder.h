#ifndef __ABENCODER_H
#define __ABENCODER_H

#include "stm32f10x.h"

void ABEncoder_Init(void);
void ABEncoder_UpdateSpeed(void);
int16_t ABEncoder_GetSpeed_left(void);
int16_t ABEncoder_GetSpeed_right(void);
extern int16_t speed_left,speed_right;
extern int32_t left_ecoder_cnt, right_ecoder_cnt;
#endif 