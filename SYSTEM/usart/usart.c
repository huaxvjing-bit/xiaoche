#include "sys.h"
#include "usart.h"	  
#include "control_system.h"
#include "colorful_led.h"

#if 1
#pragma import(__use_no_semihosting)             
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
_sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

#if EN_USART1_RX

u8 USART_RX_BUF[USART_REC_LEN];
u16 USART_RX_STA = 0;
volatile u8 Remote_Cmd_Received = 0;

static char Cmd_Buffer[32];
static u8   Cmd_Len = 0;

void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
  
    // USART1_TX: PA9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    // USART1_RX: PA10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // NVIC
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    
    USART_ClearFlag(USART1, USART_FLAG_TC | USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_IDLE);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

/* USART1 Interrupt Handler */
void USART1_IRQHandler(void)
{
	u8 Res;

    if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART1->SR;
        (void)USART1->DR;
    }

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Res = (u8)USART_ReceiveData(USART1);
        Remote_Cmd_Received = 1;

        if (Res == '0' || Res == 'x' || Res == 'X' || Res == 'S' || Res == ' ')
        {
            Execute_Motion_Command("0", 1);
            Cmd_Len = 0;
        }
        else if (Res >= '1' && Res <= '4')
        {
            if (Cmd_Len < sizeof(Cmd_Buffer) - 1)
            {
                Cmd_Buffer[Cmd_Len++] = (char)Res;
            }
        }
        else if (Res == 0x0D || Res == 0x0A)
        {
            if (Cmd_Len > 0)
            {
                Execute_Motion_Command(Cmd_Buffer, Cmd_Len);
                Cmd_Len = 0;
            }
        }
	}

    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        u32 temp_sr = USART1->SR;
        u32 temp_dr = USART1->DR;
        (void)temp_sr;
        (void)temp_dr;

        if (Cmd_Len > 0)
        {
            Execute_Motion_Command(Cmd_Buffer, Cmd_Len);
            Cmd_Len = 0;
        }
    }
}
#endif
