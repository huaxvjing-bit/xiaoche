#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include <stdio.h>
#define UART_COMM_BAUDRATE 115200
#define TCRT_PORT GPIOA
#define TCRT_RCC RCC_APB2Periph_GPIOA
#define TCRT_LEFT_PIN GPIO_Pin_11
#define TCRT_RIGHT_PIN GPIO_Pin_12
static void TCRT_Init(void){GPIO_InitTypeDef gpio;RCC_APB2PeriphClockCmd(TCRT_RCC,ENABLE);gpio.GPIO_Pin=TCRT_LEFT_PIN|TCRT_RIGHT_PIN;gpio.GPIO_Mode=GPIO_Mode_IPU;gpio.GPIO_Speed=GPIO_Speed_50MHz;GPIO_Init(TCRT_PORT,&gpio);}
int main(void){u8 left,right;RCC->CSR|=1<<24;Stm32_Clock_Init(9);MY_NVIC_PriorityGroupConfig(2);uart_init(UART_COMM_BAUDRATE);TCRT_Init();delay_ms(200);printf("TCRT TEST START\r\n");printf("PA11=LEFT PA12=RIGHT BAUD=115200\r\n");while(1){left=GPIO_ReadInputDataBit(TCRT_PORT,TCRT_LEFT_PIN)?1:0;right=GPIO_ReadInputDataBit(TCRT_PORT,TCRT_RIGHT_PIN)?1:0;printf("TCRT L=%u R=%u\r\n",(unsigned)left,(unsigned)right);delay_ms(500);}}
