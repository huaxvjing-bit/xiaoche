#ifndef __TCRT_DEBUG_H
#define __TCRT_DEBUG_H
#include "stm32f10x.h"
void TCRT_Debug_Init(void);
void TCRT_Debug_Print(void);

/* 输出 PA11/PA12 原始电平，不执行电机控制。 */
#endif
