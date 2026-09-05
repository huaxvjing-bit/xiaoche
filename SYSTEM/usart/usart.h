#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"
#define USART_REC_LEN 200
#define EN_USART1_RX 1
extern u8 USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;
extern volatile u8 Remote_Cmd_Received;
#define UART_CMD_QUEUE_SIZE 16
extern volatile u8 g_UartCmdHead;
extern volatile u8 g_UartCmdTail;
extern volatile char g_UartCmdQueue[UART_CMD_QUEUE_SIZE];
void uart_init(u32 bound);
void Process_Pending_Uart_Command(void);
#endif

