#include "sys.h"
#include "usart.h"
#include "control_system.h"
#include "colorful_led.h"

#if 1
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
_sys_exit(int x) { x = x; }
int fputc(int ch, FILE *f) { while((USART1->SR & 0X40)==0); USART1->DR=(u8)ch; return ch; }
#endif

#if EN_USART1_RX
u8 USART_RX_BUF[USART_REC_LEN];
u16 USART_RX_STA=0;
volatile u8 Remote_Cmd_Received=0;
volatile u8 g_UartCmdHead=0;
volatile u8 g_UartCmdTail=0;
volatile char g_UartCmdQueue[UART_CMD_QUEUE_SIZE];

static u8 IsMotionCommand(u8 c) { return (c>='0' && c<='4') ? 1 : 0; }
static u8 IsStopCommand(u8 c) { return (c=='0' || c=='x' || c=='X' || c=='S' || c==' ') ? 1 : 0; }

void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure; USART_InitTypeDef USART_InitStructure; NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9; GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; GPIO_Init(GPIOA,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10; GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU; GPIO_Init(GPIOA,&GPIO_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn; NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; NVIC_Init(&NVIC_InitStructure);
    USART_InitStructure.USART_BaudRate=bound; USART_InitStructure.USART_WordLength=USART_WordLength_8b; USART_InitStructure.USART_StopBits=USART_StopBits_1; USART_InitStructure.USART_Parity=USART_Parity_No; USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    USART_Init(USART1,&USART_InitStructure); USART_ClearFlag(USART1,USART_FLAG_TC|USART_FLAG_RXNE|USART_FLAG_ORE|USART_FLAG_IDLE); USART_ITConfig(USART1,USART_IT_RXNE,ENABLE); USART_ITConfig(USART1,USART_IT_IDLE,ENABLE); USART_Cmd(USART1,ENABLE);
}

/* 单生产者/单消费者命令队列：重复心跳合并，停车/后退/转向顺序不丢失 */
void USART1_IRQHandler(void)
{
    u8 c;
    if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)!=RESET){(void)USART1->SR;(void)USART1->DR;}
    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        c=(u8)USART_ReceiveData(USART1); Remote_Cmd_Received=1;
        if(IsStopCommand(c)) c='0';
        if(IsMotionCommand(c)){
            u8 next=(u8)((g_UartCmdHead+1U)%UART_CMD_QUEUE_SIZE);
            u8 last=(g_UartCmdHead==g_UartCmdTail)?0:g_UartCmdQueue[(g_UartCmdHead+UART_CMD_QUEUE_SIZE-1U)%UART_CMD_QUEUE_SIZE];
            if(last!=c && next!=g_UartCmdTail){g_UartCmdQueue[g_UartCmdHead]=c; g_UartCmdHead=next;}
        }
    }
    if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET){(void)USART1->SR;(void)USART1->DR;}
}

void Process_Pending_Uart_Command(void)
{
    char c;
    if(g_UartCmdTail!=g_UartCmdHead){
        c=g_UartCmdQueue[g_UartCmdTail];
        g_UartCmdTail=(u8)((g_UartCmdTail+1U)%UART_CMD_QUEUE_SIZE);
        Execute_Motion_Command(&c,1);
    }
}
#endif

