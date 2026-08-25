#include "stm32f10x.h"
#include "sys.h"

int main(void)
  {
    u8 g, k;
    Stm32_Clock_Init(9);						//外部时钟8Mhz 9倍频  8*9= 72mhz倍频72mhz
		MY_NVIC_PriorityGroupConfig(2);	//=====中断优先级分组		
		uart_init(115200);	            //=====串口初始化为115200
		JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
		JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

		colorful_led_Init();            //=====炫彩灯初始化

		printf("QST青软\r\n");
		/**主要程序**/
    while(1)
    {
        for(g = 1; g <= 6; g++)
        {
            for(k = 1; k <= led_num; k++)
            {
                if(g <= 3)
                {
                    if(k == (2 * g - 1) || k == (2 * g))
                    {
                        L_ws2812_rgb(k, WS_RED);
                    }
                    else
                    {
                        L_ws2812_rgb(k, WS_DARK);
                    }
                    R_ws2812_rgb(k, WS_DARK);
                }
                else
                {
                    L_ws2812_rgb(k, WS_DARK);
                    if(k == (2 * (g - 3) - 1) || k == (2 * (g - 3)))
                    {
                        R_ws2812_rgb(k, WS_RED);
                    }
                    else
                    {
                        R_ws2812_rgb(k, WS_DARK);
                    }
                }
            }
            L_ws2812_refresh(led_num);
            R_ws2812_refresh(led_num);
            delay_ms(325);
        }
    }
}