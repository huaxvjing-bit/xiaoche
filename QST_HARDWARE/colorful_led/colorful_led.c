#include "colorful_led.h"
#include "delay.h"

u8 L_ws_data[ws_num];
u8 R_ws_data[ws_num];

void colorful_led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    PCout(13) = 0;
    PCout(14) = 0;
}

void L_send_0(void)
{
    DIL = 1;
    Wait400ns;
    DIL = 0;
    Wait850ns;
}

void L_send_1(void)
{
    DIL = 1;
    Wait850ns;
    DIL = 0;
    Wait400ns;
}

void R_send_0(void)
{
    DIR = 1;
    Wait400ns;
    DIR = 0;
    Wait850ns;
}

void R_send_1(void)
{
    DIR = 1;
    Wait850ns;
    DIR = 0;
    Wait400ns;
}

void L_ws2812_reset(void)
{
    DIL = 0;
    delay_us(66);
}

void R_ws2812_reset(void)
{
    DIR = 0;
    delay_us(66);
}

void L_ws2812_rgb(u8 L_ws_num, u8 ws_r, u8 ws_g, u8 ws_b)
{
    if (L_ws_num == 0 || L_ws_num > 6) return;
    L_ws_data[(L_ws_num - 1) * 3]     = ws_g;
    L_ws_data[(L_ws_num - 1) * 3 + 1] = ws_r;
    L_ws_data[(L_ws_num - 1) * 3 + 2] = ws_b;
}

void R_ws2812_rgb(u8 R_ws_num, u8 ws_r, u8 ws_g, u8 ws_b)
{
    if (R_ws_num == 0 || R_ws_num > 6) return;
    R_ws_data[(R_ws_num - 1) * 3]     = ws_g;
    R_ws_data[(R_ws_num - 1) * 3 + 1] = ws_r;
    R_ws_data[(R_ws_num - 1) * 3 + 2] = ws_b;
}

void L_ws2812_refresh(u8 ws_count)
{
    u8 i;
    for (i = 0; i < ws_count * 3; i++)
    {
        if ((L_ws_data[i] & 0x80) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[i] & 0x40) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[i] & 0x20) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[i] & 0x10) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[i] & 0x08) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[i] & 0x04) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[i] & 0x02) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[i] & 0x01) == 0) L_send_0(); else L_send_1();
    }
    L_ws2812_reset();
}

void R_ws2812_refresh(u8 ws_count)
{
    u8 i;
    for (i = 0; i < ws_count * 3; i++)
    {
        if ((R_ws_data[i] & 0x80) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[i] & 0x40) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[i] & 0x20) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[i] & 0x10) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[i] & 0x08) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[i] & 0x04) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[i] & 0x02) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[i] & 0x01) == 0) R_send_0(); else R_send_1();
    }
    R_ws2812_reset();
}

/*
 * 12-LED Circular Flowing Light:
 * 0: Red (255,0,0)
 * 1: Blue (0,0,255)
 * 2: Green (0,255,0)
 * Initial state at shift = 0:
 *   LED 1: 0 (Red)
 *   LED 2: 1 (Blue)
 *   LED 3: 2 (Green)
 *   LED 4: 0 (Red) ...
 * Every 1s (shift++), each LED advances color = (color + 1) % 3
 */
static void Set_Ring_Led_Color(u8 led_idx, u8 color_code)
{
    u8 r = 0, g = 0, b = 0;
    switch (color_code)
    {
        case 0: r = 255; g = 0;   b = 0;   break; // Red
        case 1: r = 0;   g = 0;   b = 255; break; // Blue
        case 2: r = 0;   g = 255; b = 0;   break; // Green
        default: break;
    }

    if (led_idx <= 6)
    {
        L_ws2812_rgb(led_idx, r, g, b);
    }
    else
    {
        R_ws2812_rgb(led_idx - 6, r, g, b);
    }
}

void Colorful_Ring_Flow_1s_Update(void)
{
    static u8 flow_shift = 0;
    u8 i;

    for (i = 1; i <= 12; i++)
    {
        u8 color_code = (u8)(((i - 1) + flow_shift) % 3);
        Set_Ring_Led_Color(i, color_code);
    }

    L_ws2812_refresh(led_num);
    R_ws2812_refresh(led_num);

    flow_shift = (u8)((flow_shift + 1) % 3);
}
