#ifndef __CLIFF_SENSOR_H
#define __CLIFF_SENSOR_H

#include "stm32f10x.h"
#include "sys.h"

/*
 * 根据《Schematic_QST-鸿蒙小车》原理图精准硬件引脚定义:
 * 车头底板 TCRT5000 双路光电/寻迹/防跌落传感器:
 * - 左路探头 (TC_OUT_L): PA11
 * - 右路探头 (TC_OUT_R): PA12
 */
#define CLIFF_PIN_L     GPIO_Pin_11
#define CLIFF_PIN_R     GPIO_Pin_12
#define CLIFF_PORT      GPIOA
#define CLIFF_RCC       RCC_APB2Periph_GPIOA

/*
 * TCRT5000 + LM393 逻辑:
 * - 正常地面 (白色/浅色反射光线强): LM393 输出低电平 (0), 传感器指示灯点亮
 * - 悬空边缘 / 黑色区域 (无反射): LM393 输出高电平 (1), 传感器指示灯熄灭
 */
#define CLIFF_LEFT_VAL   GPIO_ReadInputDataBit(CLIFF_PORT, CLIFF_PIN_L)
#define CLIFF_RIGHT_VAL  GPIO_ReadInputDataBit(CLIFF_PORT, CLIFF_PIN_R)

void Cliff_Sensor_Init(void);
u8 Check_Is_Cliff(void);

#endif

