#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "motor.h"
#include "encoder.h"
#include "colorful_led.h"
#include "control_system.h"
#include "line_tracking.h"
#include <stdio.h>

/* PID 参数 (待实车标定) */
PID_ParamTypeDef PID_Motor_L = { 7.5f, 0.7f, 0.35f };
PID_ParamTypeDef PID_Motor_R = { 7.5f, 0.7f, 0.35f };
Motor_SpeedTypeDef Car_Speed = { 0.0f, 0.0f };
float Pos_Sync_Kp = 0.45f;

#define UART_COMM_BAUDRATE 115200U

int main(void)
{
    u8 led_tick = 0U;

    RCC->CSR |= 1 << 24;
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);

    uart_init(UART_COMM_BAUDRATE);
    PWM_Init(7199, 9);
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();

    /* 彩灯初始化 */
    colorful_led_Init();
    Ring_Flowing_Led_Init();

    Set_Pwm(0, 0);
    Line_Tracking_Init();

    delay_ms(500);
    while (1)
    {
        /* 10ms 事件驱动循迹闭环 */
        Line_Tracking_Task();

        /* 彩灯维护 (100ms) */
        led_tick++;
        if (led_tick >= 10)
        {
            led_tick = 0;
            Ring_Flowing_Led_Process();
        }

        delay_ms(10);
    }
}
