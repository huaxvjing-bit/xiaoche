#include "tcrt_debug.h"
#include "usart.h"
#include <stdio.h>

#define TCRT_DEBUG_PORT GPIOA
#define TCRT_DEBUG_RCC  RCC_APB2Periph_GPIOA
#define TCRT_DEBUG_L_PIN GPIO_Pin_11
#define TCRT_DEBUG_R_PIN GPIO_Pin_12

void TCRT_Debug_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(TCRT_DEBUG_RCC, ENABLE);
    gpio.GPIO_Pin = TCRT_DEBUG_L_PIN | TCRT_DEBUG_R_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TCRT_DEBUG_PORT, &gpio);
    printf("[TCRT_DEBUG] init PA11(left), PA12(right)\r\n");
}

void TCRT_Debug_Print(void)
{
    u8 left = GPIO_ReadInputDataBit(TCRT_DEBUG_PORT, TCRT_DEBUG_L_PIN) ? 1 : 0;
    u8 right = GPIO_ReadInputDataBit(TCRT_DEBUG_PORT, TCRT_DEBUG_R_PIN) ? 1 : 0;
    printf("[TCRT_RAW] left_PA11=%u right_PA12=%u\r\n",
           (unsigned)left, (unsigned)right);
    printf("[TCRT_MEANING] low=0 high=1; record ground and black tape separately\r\n");
}
