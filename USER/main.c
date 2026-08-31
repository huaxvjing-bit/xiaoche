#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "encoder.h"
#include "control_system.h"
#include "colorful_led.h"
#include "cliff_sensor.h"
#include "usart.h"
#include "delay.h"
#include <stdio.h>

PID_ParamTypeDef PID_Motor_L = { 7.5f, 0.7f, 0.35f };
PID_ParamTypeDef PID_Motor_R = { 7.5f, 0.7f, 0.35f };
float Pos_Sync_Kp = 0.45f;

#define PULSE_BACK_10CM        1370L
#define SPEED_SAFETY_BACK      0.60f
#define ACTION_INTERVAL_MS     500
#define UART_COMM_BAUDRATE     115200

Motor_SpeedTypeDef Car_Speed = { 0.0f, 0.0f };

void Safe_Retreat_Handler(void)
{
    Set_Pwm(0, 0);
    Car_Speed.target_speed_L = 0.0f;
    Car_Speed.target_speed_R = 0.0f;
    PID_Reset();
    delay_ms(100);

    Move_Backward(PULSE_BACK_10CM, SPEED_SAFETY_BACK);
    Car_Brake(ACTION_INTERVAL_MS);

    Car_Speed.target_speed_L = 0.0f;
    Car_Speed.target_speed_R = 0.0f;
    PID_Reset();
}

int main(void)
{
    u8 led_cycle_count = 0;

    RCC->CSR |= 1 << 24;
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);
    
    uart_init(UART_COMM_BAUDRATE);
    
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    PWM_Init(7199, 9);
    colorful_led_Init();
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    Cliff_Sensor_Init();
    
    PID_Reset();
    Car_Speed.target_speed_L = 0.0f;
    Car_Speed.target_speed_R = 0.0f;
    Set_Pwm(0, 0);

    // Initial flow light state
    Colorful_Ring_Flow_1s_Update();
    delay_ms(500);

    while (1)
    {
        // 1. Cliff sensor safety check
        if (Check_Is_Cliff())
        {
            Safe_Retreat_Handler();
        }
        else
        {
            // 2. Closed-loop motion control (100ms cycle)
            System_Control();
        }

        // 3. Parallel 1s Flowing Light (10 * 100ms = 1s)
        led_cycle_count++;
        if (led_cycle_count >= 10)
        {
            led_cycle_count = 0;
            Colorful_Ring_Flow_1s_Update();
        }

        delay_ms(100);
    }
}
