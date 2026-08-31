#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

// 电机方向引脚定义 (位带操作)
#define AIN PBout(14)  // 电机2(右轮)方向控制引脚
#define BIN PBout(13)  // 电机1(左轮)方向控制引脚

// 电机PWM比较寄存器定义 (TIM4)
// PB6 对应 TIM4_CH1 (右轮PWM)
// PB7 对应 TIM4_CH2 (左轮PWM)
#define PWMA TIM4->CCR1  // 右轮PWM (PB6 -> TIM4_CH1)
#define PWMB TIM4->CCR2  // 左轮PWM (PB7 -> TIM4_CH2)

// PWM最大重装载值
#define PWM_MAX_VAL 7199

// 函数声明
u32 myabs(long int a);
void Set_Pwm(int moto1, int moto2);
void Motor_Init(void);
void PWM_Init(u16 arr, u16 psc);

#endif
