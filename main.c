#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"
#include "encoder.h"
#include "control_system.h"
/**
 * 1. PID 参数调节区 */
PID_ParamTypeDef PID_Motor_L = { 7.5f, 0.7f, 0.35f };  // 左轮 PID
PID_ParamTypeDef PID_Motor_R = { 7.5f, 0.7f, 0.35f };  // 右轮 PID

float Pos_Sync_Kp = 0.45f; 

/**
 * 3. 目标速度与 1 米目标距离脉冲数
 */
#define TARGET_DISTANCE_PULSE  13700L  // 1米距离目标脉冲数

#define FORWARD_SPEED   0.75f   // 前进速度: 0.75 转/秒
#define BACKWARD_SPEED -0.75f   // 后退速度: -0.75 转/秒
#define STOP_SPEED      0.0f    // 停止速度: 0.0 转/秒

Motor_SpeedTypeDef Car_Speed = { 0.0f, 0.0f };
int main(void)
{
    u16 i = 0;

    RCC->CSR |= 1 << 24;            // 清除复位标志位
    Stm32_Clock_Init(9);            // 外部时钟8Mhz 9倍频 8*9=72Mhz
    MY_NVIC_PriorityGroupConfig(2); // 中断优先级分组
    uart_init(115200);              // 串口初始化为115200
    JTAG_Set(JTAG_SWD_DISABLE);     // 关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           // 打开SWD接口，可以利用主板的SWD接口调试

    PWM_Init(7199, 9);              // 定时器TIM4初始化，PWM频率 = 72MHz / ((7199+1)*(9+1)) = 1kHz
    colorful_led_Init();            // 炫彩灯初始化

    Encoder_Init_TIM2();            // 初始化编码器（左电机 TIM2: PA0/PA1）
    Encoder_Init_TIM3();            // 初始化编码器（右电机 TIM3: PA6/PA7）

    PID_Reset();                    // 复位 PID 状态与电机输出

    printf("QST Pioneer Car - High Precision Forward/Backward Motion Start\r\n");

    while (1)
    {
        // ------------------ 1. 前进 1 米 ------------------
        printf("=== Status: Forward 1 Meter ===\r\n");
        Read_Encoder(2);
        Read_Encoder(3);
        Distance_Reset();
        PID_Reset();

        Car_Speed.target_speed_L = FORWARD_SPEED;
        Car_Speed.target_speed_R = FORWARD_SPEED;

        while (Get_Average_Distance_Pulse() < TARGET_DISTANCE_PULSE)
        {
            System_Control();
            delay_ms(100);
        }

        printf("=== Reached 1 Meter -> Status: Stop ===\r\n");
        Car_Speed.target_speed_L = STOP_SPEED;
        Car_Speed.target_speed_R = STOP_SPEED;
        PID_Reset();
        for (i = 0; i < 20; i++)
        {
            System_Control();
            delay_ms(100);
        }
        printf("=== Status: Backward 1 Meter ===\r\n");
        Read_Encoder(2);
        Read_Encoder(3);
        Distance_Reset();
        PID_Reset();

        Car_Speed.target_speed_L = BACKWARD_SPEED;
        Car_Speed.target_speed_R = BACKWARD_SPEED;

        while (Get_Average_Distance_Pulse() < TARGET_DISTANCE_PULSE)
        {
            System_Control();
            delay_ms(100);
        }
        printf("=== Reached 1 Meter -> Status: Stop ===\r\n");
        Car_Speed.target_speed_L = STOP_SPEED;
        Car_Speed.target_speed_R = STOP_SPEED;
        PID_Reset();
        for (i = 0; i < 20; i++)
        {
            System_Control();
            delay_ms(100);
        }
    }
}
