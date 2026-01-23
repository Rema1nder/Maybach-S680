#ifndef __ADC_GET_H__
#define __ADC_GET_H__

#include "stm32f10x.h"
#include <stdint.h>

// 16通道多路复用ADC + 1路PA0缓存（单位：12-bit原始值）
// 索引0~15为模拟复用器通道，索引16为PA0(ADC12_IN0)
extern volatile uint16_t g_mux_adc_values[17];

// 初始化：配置PB10/PB2/PB1/PB0为S0..S3，配置PA1(ADC12_IN1)为ADC输入，并完成ADC1校准
void MuxADC_Init(void);

// 采集单个通道：channel取值0~15，返回一次转换值
uint16_t MuxADC_ReadChannel(uint8_t channel);

// 扫描并填充全部16路到g_mux_adc_values[]
void MuxADC_SampleAll(void);

#endif

