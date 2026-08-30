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

/**
 * 1. PID 参数与同步配置
 */
PID_ParamTypeDef PID_Motor_L = { 7.5f, 0.7f, 0.35f };  // 左轮 PID
PID_ParamTypeDef PID_Motor_R = { 7.5f, 0.7f, 0.35f };  // 右轮 PID
float Pos_Sync_Kp = 0.45f;                             // 直线同步纠偏系数

/**
 * 2. 运动脉冲与避险距离定义
 */
#define PULSE_1_METER          13700L   // 1米距离目标脉冲数
#define PULSE_BACK_10CM        1370L    // 避险后退10cm脉冲数 (13700 / 10)
#define SPEED_SAFETY_BACK      0.60f    // 跌落撤退速度
#define ACTION_INTERVAL_MS     500      // 动作停顿时间

/**
 * 3. 串口通信波特率: 设置为板载标准 115200 bps
 */
#define UART_COMM_BAUDRATE     115200

Motor_SpeedTypeDef Car_Speed = { 0.0f, 0.0f };

/**
 * @brief  防跌落避险处理函数 (车头悬空时触发急刹与定距后退10cm)
 */
void Safe_Retreat_Handler(void)
{
    // 1. 立即强制急刹
    Set_Pwm(0, 0);
    Car_Speed.target_speed_L = 0.0f;
    Car_Speed.target_speed_R = 0.0f;
    PID_Reset();
    delay_ms(100);

    // 2. 闭环精准后退 10cm
    Move_Backward(PULSE_BACK_10CM, SPEED_SAFETY_BACK);

    // 3. 阻尼刹车停稳
    Car_Brake(ACTION_INTERVAL_MS);

    // 4. 重置目标速度为 0 (静止待命)
    Car_Speed.target_speed_L = 0.0f;
    Car_Speed.target_speed_R = 0.0f;
    PID_Reset();
}

int main(void)
{
    /* 硬件底层与外设初始化 */
    RCC->CSR |= 1 << 24;            // 清除复位标志位
    Stm32_Clock_Init(9);            // 外部时钟8Mhz 9倍频 8*9=72Mhz
    MY_NVIC_PriorityGroupConfig(2); // 中断优先级分组
    
    // 初始化串口1 (PA9/PA10 采用 115200 高速标准波特率)
    uart_init(UART_COMM_BAUDRATE);  
    
    JTAG_Set(JTAG_SWD_DISABLE);     // 关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           // 打开SWD调试接口

    PWM_Init(7199, 9);              // TIM4 PWM初始化 (PB6/PB7, 1kHz)
    colorful_led_Init();            // 炫彩灯初始化
    Encoder_Init_TIM2();            // 编码器初始化 (左轮 TIM2: PA0/PA1)
    Encoder_Init_TIM3();            // 编码器初始化 (右轮 TIM3: PA6/PA7)
    Cliff_Sensor_Init();            // 车头 TCRT5000 探头初始化 (PA11/PA12)
    
    // 初始状态强制锁定为 0，上电绝对静止
    PID_Reset();
    Car_Speed.target_speed_L = 0.0f;
    Car_Speed.target_speed_R = 0.0f;
    Set_Pwm(0, 0);

    R_led_mode();                   // 尾灯点亮表示系统待命
    delay_ms(1000);                 // 上电稳态等待

    /*
     * 主循环：防跌落实时守护 + 蓝牙/鸿蒙速度闭环响应 (100ms周期，与 PID OverFlowTime 严格对齐)
     */
    while (1)
    {
        // 1. 最高优先级安全守护：车头探头 (PA11/PA12) 悬空检测
        if (Check_Is_Cliff())
        {
            Safe_Retreat_Handler(); // 立即后退 10cm 避险
        }
        else
        {
            // 2. 正常状态：执行速度闭环 (实时响应串口下发的指令)
            System_Control();
        }

        delay_ms(100); // 100ms 采样周期，与 PID CPR 转换精确对齐
    }
}
