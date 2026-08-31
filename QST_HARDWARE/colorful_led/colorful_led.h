#ifndef __COLORFUL_LED_H
#define __COLORFUL_LED_H

#include "sys.h"

#define ws_num 24
#define led_num 6
#define DIL PCout(13) 
#define DIR PCout(14) 

// RGB Colors (R, G, B)
#define WS_DARK   0,0,0
#define WS_RED    255,0,0
#define WS_GREEN  0,255,0
#define WS_BLUE   0,0,255

#define Wait10nop  {__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();}
#define Wait250ns  {__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();}
#define Wait400ns  {Wait250ns;Wait10nop;}
#define Wait850ns  {Wait250ns;Wait10nop;Wait10nop;Wait10nop;Wait10nop;__NOP();__NOP();__NOP();__NOP();__NOP();}

void colorful_led_Init(void);
void L_ws2812_rgb(u8 L_ws_num, u8 ws_r, u8 ws_g, u8 ws_b);
void R_ws2812_rgb(u8 R_ws_num, u8 ws_r, u8 ws_g, u8 ws_b);
void L_ws2812_refresh(u8 ws_count);
void R_ws2812_refresh(u8 ws_count);

/* Non-blocking 1s circular flowing water light (12 LEDs) */
void Colorful_Ring_Flow_1s_Update(void);

#endif
