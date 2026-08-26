#include "stm32f10x.h"
#include "sys.h"
#include "motor.h"

int main(void)
{
    RCC->CSR |= 1 << 24;            // 清除复位标志位
    Stm32_Clock_Init(9);            // 外部时钟8Mhz 9倍频 8*9=72Mhz
    MY_NVIC_PriorityGroupConfig(2); // 中断优先级分组
    uart_init(115200);              // 串口初始化为115200
    JTAG_Set(JTAG_SWD_DISABLE);     // 关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           // 打开SWD接口，可以利用主板的SWD接口调试

    PWM_Init(7199, 9);              // 定时器TIM4初始化，PWM频率 = 72MHz / ((7199+1)*(9+1)) = 1kHz
    colorful_led_Init();            // 炫彩灯初始化

    printf("QST先锋号鸿蒙智能小车 - 轮子运动控制程序启动\r\n");

    /**
     * 控制轮子运动周期循环：
     * 设电机控制速度为 2500 (PWM满量程7199，较小速度平稳运行)
     * 1. 1s 前进 (左轮正转, 右轮正转)
     * 2. 1s 后退 (左轮反转, 右轮反转)
     * 3. 1s 左轮前转, 右轮后转
     * 4. 1s 左轮后转, 右轮前转
     */
    while(1)
    {
        // 1. 前进 1s (左轮正向 2500，右轮正向 2500)
        printf("状态: 前进 1s\r\n");
        Set_Pwm(2500, 2500);
        delay_ms(1000);

        // 2. 后退 1s (左轮反向 -2500，右轮反向 -2500)
        printf("状态: 后退 1s\r\n");
        Set_Pwm(-2500, -2500);
        delay_ms(1000);

        // 3. 左轮前转，右轮后转 1s (左轮正向 2500，右轮反向 -2500)
        printf("状态: 左轮前转, 右轮后转 1s\r\n");
        Set_Pwm(2500, -2500);
        delay_ms(1000);

        // 4. 左轮后转，右轮前转 1s (左轮反向 -2500，右轮正向 2500)
        printf("状态: 左轮后转, 右轮前转 1s\r\n");
        Set_Pwm(-2500, 2500);
        delay_ms(1000);
    }
}
