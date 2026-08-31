#include "motor.h"

/**
 * @brief  绝对值函数
 * @param  a: 输入参数
 * @retval u32: 绝对值
 */
u32 myabs(long int a)
{
    u32 temp;
    if(a < 0)
    {
        temp = -a;
    }
    else
    {
        temp = a;
    }
    return temp;
}

/**
 * @brief  电机PWM与转向控制
 * @param  moto1: 左轮速度及方向 (正数正转，负数反转，范围 -7199 ~ 7199)
 * @param  moto2: 右轮速度及方向 (正数正转，负数反转，范围 -7199 ~ 7199)
 */
void Set_Pwm(int moto1, int moto2)
{
    // 电机2控制 (右轮 AIN, PWMA -> PB14, PB6/TIM4_CH1)
    if(moto2 >= 0) {
        AIN = 0;
        PWMA = myabs(moto2);
    } else {
        AIN = 1;
        PWMA = 7199 - myabs(moto2);
    }

    // 电机1控制 (左轮 BIN, PWMB -> PB13, PB7/TIM4_CH2)
    if(moto1 >= 0) {
        BIN = 0;
        PWMB = myabs(moto1);
    } else {
        BIN = 1;
        PWMB = 7199 - myabs(moto1);
    }
}

/**
 * @brief  电机方向控制IO初始化
 * @note   配置 GPIOB Pin 13, Pin 14 为推挽输出 (AIN, BIN)
 */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 使能PB端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13; // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // 50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // 初始化GPIOB

    AIN = 0;
    BIN = 0;
}

/**
 * @brief  定时器TIM4 PWM初始化
 * @param  arr: 自动重装载值
 * @param  psc: 预分频系数
 * @note   使用 TIM4 CH1(PB6) 和 CH2(PB7)
 */
void PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    Motor_Init(); // 初始化电机方向IO口

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  // 使能TIM4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 使能GPIOB时钟

    // 配置PB6(TIM4_CH1)与PB7(TIM4_CH2)为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = arr;                      // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                   // 预分频系数
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                 // 时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                            // 初始占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     // 输出极性高
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);                     // 初始化TIM4 CH1 (对应PB6)
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);                     // 初始化TIM4 CH2 (对应PB7)

    TIM_CtrlPWMOutputs(TIM4, ENABLE);                            // 主输出使能

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);            // 使能CH1预装载
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);            // 使能CH2预装载

    TIM_ARRPreloadConfig(TIM4, ENABLE);                          // 使能ARR预装载

    TIM_Cmd(TIM4, ENABLE);                                       // 使能TIM4
}
