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

    DIL = 0;
    DIR = 0;
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
    if (L_ws_num < 1 || L_ws_num > 6) return;
    L_ws_data[(L_ws_num - 1) * 3]     = ws_g;
    L_ws_data[(L_ws_num - 1) * 3 + 1] = ws_r;
    L_ws_data[(L_ws_num - 1) * 3 + 2] = ws_b;
}

void R_ws2812_rgb(u8 R_ws_num, u8 ws_r, u8 ws_g, u8 ws_b)
{
    if (R_ws_num < 1 || R_ws_num > 6) return;
    R_ws_data[(R_ws_num - 1) * 3]     = ws_g;
    R_ws_data[(R_ws_num - 1) * 3 + 1] = ws_r;
    R_ws_data[(R_ws_num - 1) * 3 + 2] = ws_b;
}

void L_ws2812_refresh(u8 ws_count)
{
    u8 L_ws_ri = 0;
    for (; L_ws_ri < ws_count * 3; L_ws_ri++)
    {
        if ((L_ws_data[L_ws_ri] & 0x80) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[L_ws_ri] & 0x40) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[L_ws_ri] & 0x20) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[L_ws_ri] & 0x10) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[L_ws_ri] & 0x08) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[L_ws_ri] & 0x04) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[L_ws_ri] & 0x02) == 0) L_send_0(); else L_send_1();
        if ((L_ws_data[L_ws_ri] & 0x01) == 0) L_send_0(); else L_send_1();
    }
    L_ws2812_reset();
}

void R_ws2812_refresh(u8 ws_count)
{
    u8 R_ws_ri = 0;
    for (; R_ws_ri < ws_count * 3; R_ws_ri++)
    {
        if ((R_ws_data[R_ws_ri] & 0x80) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[R_ws_ri] & 0x40) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[R_ws_ri] & 0x20) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[R_ws_ri] & 0x10) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[R_ws_ri] & 0x08) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[R_ws_ri] & 0x04) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[R_ws_ri] & 0x02) == 0) R_send_0(); else R_send_1();
        if ((R_ws_data[R_ws_ri] & 0x01) == 0) R_send_0(); else R_send_1();
    }
    R_ws2812_reset();
}

static u8 Ring_Led_Step = 0;

void Set_Ring_Led_Color(u8 index, u8 ws_r, u8 ws_g, u8 ws_b)
{
    if (index >= 1 && index <= 6)
    {
        L_ws2812_rgb(index, ws_r, ws_g, ws_b);
    }
    else if (index >= 7 && index <= 12)
    {
        R_ws2812_rgb(index - 6, ws_r, ws_g, ws_b);
    }
}

void Refresh_Ring_Led(void)
{
    L_ws2812_refresh(led_num);
    R_ws2812_refresh(led_num);
}

void Ring_Flowing_Led_Step(void)
{
    u8 i;
    for (i = 1; i <= 12; i++)
    {
        if ((i + Ring_Led_Step) % 2 == 1)
        {
            Set_Ring_Led_Color(i, 255, 0, 0);       // 纯红
        }
        else
        {
            Set_Ring_Led_Color(i, 255, 255, 255);   // 纯白
        }
    }
    Refresh_Ring_Led();
    Ring_Led_Step = (Ring_Led_Step + 1) % 2;
}

void Ring_Flowing_Led_Init(void)
{
    Ring_Led_Step = 0;
    Ring_Flowing_Led_Step();
}

void Ring_Flowing_Led_Process(void)
{
    static u8 tick_counter = 0;
    tick_counter++;
    if (tick_counter >= 10)
    {
        tick_counter = 0;
        Ring_Flowing_Led_Step();
    }
}

